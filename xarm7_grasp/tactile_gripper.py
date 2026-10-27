"""Xela feedback loop using xarm_api services, independently of MoveIt arm motion."""
import math
import threading
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from xarm_msgs.srv import GetFloat32, GripperMove, SetFloat32, SetInt16
from xela_server_ros2.msg import SensStream

from .tactile_force import ForceRule


class TactileGripper:
    def __init__(self, node):
        self.node = node
        def param(name, default):
            return node.declare_parameter('force.' + name, default).value
        self.ids = [param('left_sensor', -1), param('right_sensor', -1)]
        self.signs = [param('left_sign', 1.0), param('right_sign', 1.0)]
        self.axis = param('normal_axis', 'z')
        self.confirmed = param('calibration_confirmed', False)
        self.rule = ForceRule(**{k: param(k, v) for k, v in vars(ForceRule()).items()})
        self.period = float(param('period', 0.1))
        self.stale = float(param('stale_timeout', 0.5))
        self.acquire_timeout = float(param('acquire_timeout', 20.0))
        self.speed = float(param('speed', 100.0))
        self.settle = float(param('settle_time', 0.5))
        if (self.axis not in ('x', 'y', 'z') or any(s not in (-1., 1.) for s in self.signs)
                or not all(math.isfinite(v) and v > 0 for v in
                           (self.period, self.stale, self.acquire_timeout, self.speed, self.settle))):
            raise ValueError('Invalid tactile timing, speed, axis or sign')
        namespace = param('driver_namespace', '/xarm').rstrip('/')
        group = ReentrantCallbackGroup()
        self.clients = {name: node.create_client(typ, namespace + '/' + name, callback_group=group)
                        for name, typ in [('get_gripper_position', GetFloat32),
                                          ('set_gripper_position', GripperMove),
                                          ('set_gripper_speed', SetFloat32),
                                          ('set_gripper_mode', SetInt16),
                                          ('set_gripper_enable', SetInt16), ('set_state', SetInt16)]}
        self.lock = threading.Lock()
        self.samples = {}
        self.last_keys = {}
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.thread = None
        self.fault = None  # latched; restart required
        node.create_subscription(SensStream, param('topic', '/xServTopic'), self._sample,
                                 qos_profile_sensor_data, callback_group=group)

    def _sample(self, msg):
        now = time.monotonic()
        with self.lock:
            for sid in self.ids:
                matches = [s for s in msg.sensors if s.sensor_pos == sid]
                if len(matches) != 1:
                    self.samples.pop(sid, None)
                    continue
                s = matches[0]
                key = (s.message, s.time)
                if self.last_keys.get(sid) == key:
                    continue  # replayed frames cannot refresh freshness
                self.last_keys[sid] = key
                if not s.taxels or len(s.forces) != len(s.taxels):
                    self.samples.pop(sid, None)
                    continue  # raw counts are never treated as N
                sign = self.signs[self.ids.index(sid)]
                vals = [sign * getattr(f, self.axis) for f in s.forces]
                if not all(math.isfinite(v) for v in vals):
                    self.samples.pop(sid, None)
                    continue
                # Clamp each taxel after sign correction; do not cancel compression with tension.
                self.samples[sid] = (now, sum(max(0., v) for v in vals), max(vals))

    def reading(self):
        with self.lock:
            samples = [self.samples.get(i) for i in self.ids]
        if any(s is None or time.monotonic() - s[0] > self.stale for s in samples):
            raise RuntimeError('Xela missing, uncalibrated or stale')
        return tuple(s[1] for s in samples), max(s[2] for s in samples), tuple(s[0] for s in samples)

    def call(self, name, request):
        client = self.clients[name]
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Driver service unavailable: ' + client.srv_name)
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(2.0):
            raise RuntimeError('Driver service timeout (outcome unknown): ' + name)
        result = future.result()
        if result is None or result.ret != 0:
            raise RuntimeError(f'{name} failed: {result}')
        return result

    def check(self):
        if self.fault:
            raise RuntimeError(self.fault)
        if (not self.confirmed or min(self.ids) < 0 or self.ids[0] == self.ids[1]):
            raise RuntimeError('Configure two sensor IDs and confirm force calibration first')
        return self.reading()

    def wait_ready(self):
        # Allow the executor to deliver the first sensor message at startup.
        deadline = time.monotonic() + 3.0
        while True:
            try:
                return self.check()
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)

    def position(self):
        pos = self.call('get_gripper_position', GetFloat32.Request()).data
        if not math.isfinite(pos) or not 0 <= pos <= 850:
            raise RuntimeError('Invalid actual gripper position')
        return pos

    def move(self, position):
        req = GripperMove.Request()
        req.pos, req.wait, req.timeout = float(position), False, 1.0
        self.call('set_gripper_position', req)

    def open(self):
        self.stop()
        self.check()
        for name, value in [('set_gripper_mode', 0), ('set_gripper_enable', 1)]:
            req = SetInt16.Request(); req.data = value
            self.call(name, req)
        req = SetFloat32.Request(); req.data = self.speed
        self.call('set_gripper_speed', req)
        self.move(self.rule.open_position)
        deadline = time.monotonic() + 15.0
        while abs(self.position() - self.rule.open_position) > self.rule.step:
            self.check()
            if time.monotonic() > deadline:
                raise RuntimeError('Gripper open timeout')
            time.sleep(self.period)
        # Verify unloaded pads before approach; no automatic zeroing under load.
        forces, _, _ = self.check()
        if max(forces) >= self.rule.contact:
            raise RuntimeError('Open gripper is loaded; verify mounting and sensor zero')

    def acquire(self):
        self.check()
        self.stop_event.clear()
        self.ready.clear()
        self.thread = threading.Thread(target=self._regulate, daemon=True)
        self.thread.start()
        deadline = time.monotonic() + self.acquire_timeout + 5.0
        while not self.ready.wait(0.05):
            self.check()
            if time.monotonic() > deadline:
                self.trip('Force acquisition worker timeout')
        self.check()

    def _regulate(self):
        deadline = time.monotonic() + self.acquire_timeout
        stable_since = None
        previous = None
        last_command = None
        command_time = 0.
        try:
            while not self.stop_event.is_set():
                forces, peak, frames = self.check()
                position = self.position()
                if self.ready.is_set() and min(forces) < self.rule.contact:
                    raise RuntimeError('Lost bilateral contact during transport')
                target, stable = self.rule.decide(forces, peak, position)
                # Never accumulate a deeper target while a previous step is still moving.
                arrived = last_command is None or abs(position - last_command) <= 0.5
                if last_command is not None and not arrived and time.monotonic() - command_time > 2.0:
                    raise RuntimeError('Gripper step stalled')
                if frames != previous:
                    previous = frames
                    if stable and arrived:
                        stable_since = stable_since or time.monotonic()
                        if time.monotonic() - stable_since >= self.settle:
                            self.ready.set()
                    else:
                        stable_since = None
                    if target != position and (arrived or target > position):
                        self.move(target)
                        last_command, command_time = target, time.monotonic()
                if not self.ready.is_set() and time.monotonic() > deadline:
                    raise RuntimeError('Bilateral target force not reached in time')
                self.stop_event.wait(self.period)
        except Exception as exc:
            self.trip(str(exc))

    def trip(self, reason):
        self.fault = reason
        self.stop_event.set()
        self.node.get_logger().error('Force control fault (latched): ' + reason)
        # Stop arm first; retain object rather than automatically dropping it.
        try:
            req = SetInt16.Request(); req.data = 4
            self.call('set_state', req)
        except Exception as exc:
            self.node.get_logger().error('Arm stop failed: ' + str(exc))
        try:
            self.move(self.position())  # best effort halt of remaining short finger step
        except Exception as exc:
            self.node.get_logger().error('Gripper hold failed: ' + str(exc))

    def stop(self):
        self.stop_event.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(8.0)
            if self.thread.is_alive():
                raise RuntimeError('Force worker did not stop')

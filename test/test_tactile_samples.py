"""Use actual ROS messages, without moving hardware or starting a ROS node."""
import threading
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('xela_server_ros2.msg')
from xela_server_ros2.msg import SensStream, SensorFull, Forces, Taxel
from xarm7_grasp.tactile_gripper import TactileGripper


def controller():
    c = TactileGripper.__new__(TactileGripper)
    c.ids, c.signs, c.axis = [1, 2], [1., -1.], 'z'
    c.lock, c.samples, c.last_keys, c.stale = threading.Lock(), {}, {}, .5
    return c


def message():
    return SensStream(sensors=[SensorFull(sensor_pos=1, message=1, time=1.,
        taxels=[Taxel()], forces=[Forces(z=.4)]),
        SensorFull(sensor_pos=2, message=1, time=1.,
        taxels=[Taxel()], forces=[Forces(z=-.3)])])


def test_signed_force_and_replay_expiry():
    c = controller()
    m = message()
    c._sample(m)
    assert c.reading()[0] == pytest.approx((.4, .3))
    with c.lock:
        c.samples = {k: (time.monotonic() - 1., v[1], v[2]) for k, v in c.samples.items()}
    c._sample(m)
    with pytest.raises(RuntimeError):
        c.reading()


@pytest.mark.parametrize('fault', ['missing', 'raw', 'partial', 'nan', 'duplicate'])
def test_invalid_stream_rejected(fault):
    c, m = controller(), message()
    if fault == 'missing':
        m.sensors = m.sensors[:1]
    elif fault == 'raw':
        m.sensors[0].forces = []
    elif fault == 'partial':
        m.sensors[0].taxels.append(Taxel())
    elif fault == 'nan':
        m.sensors[0].forces[0].z = float('nan')
    else:
        m.sensors.append(m.sensors[0])
    c._sample(m)
    with pytest.raises(RuntimeError):
        c.reading()

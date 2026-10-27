# 集成机械臂的抓取控制策略代码 (ROS2) —— IK 关节目标版
# 接收 GSNet 发布的抓取姿态，执行抓取动作
# 使用 xarm_ros2 的 xarm_planner 规划机械臂运动
#
# 【本文件与 grasp_policy_xarm.py 的区别（方案1：pose目标 → 关节目标）】
#   原版 _move_arm_free 直接调用 xarm_pose_plan（setPoseTarget + OMPL）。
#   xArm7 有 7 个关节，而位姿目标只有 6 维约束（位置3+姿态3），IK 解有无穷多组，
#   OMPL 的 IK 约束采样器用"随机种子"反复采样目标构型，经常采到与当前构型
#   差异巨大的解（如肘部翻转、腕关节转180°），导致规划出的路径"很离谱"。
#
#   本版改为三步走：
#     1. 订阅 /joint_states 获取当前 7 关节角；
#     2. 调用 move_group 的 /compute_ik 服务（GetPositionIK），
#        用【当前关节角作为 IK 种子】求解目标位姿 —— 数值解天然贴近当前构型，
#        这正是 RViz 中拖动交互标记时路径质量高的本质原因；
#     3. 把解出的 7 个关节角作为目标调用 xarm_joint_plan（setJointValueTarget + OMPL），
#        冗余自由度被锁定在当前构型附近，路径自然、平滑。
#   若 IK 求解失败，自动回退到原版 xarm_pose_plan 位姿规划，保证鲁棒性。
#
# 用法:
#   ros2 run xarm7_grasp grasp_policy_xarm_joint
#
# 前提:
#   1. MoveIt2 + xArm7 + xarm_planner_node 已启动
#      (grasp_simulation.launch.py 或 grasp_real.launch.py 会自动启动规划服务节点)
#   2. RealSense 相机已启动，手眼标定 TF 已发布
#   3. detect_grasp.py 节点已运行（发布 /grasp_pose 和 /grasp_width）

import os
import sys
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# 增加 tool.py 文件所在的路径（本文件所在目录，跨机器可用）
sys.path.append('/media/gao/2/Grasp/grasp_ws/src/xarm7_grasp')
from utils import tool as utool

import tf2_ros
from std_msgs.msg import Float32
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from xarm_msgs.srv import PlanPose, PlanJoint, PlanExec, PlanSingleStraight
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes


# ============================================================
#  抓取策略节点
# ============================================================
class GraspPolicyXarmJoint(Node):
    """
    接收 GSNet 检测节点发布的抓取姿态，
    通过 TF2 完成坐标系变换，按 7 步序列执行抓取。

    抓取序列（参考 grasp_policy.py / ROS1 Policy_xArm7）：
      1. 移动到预抓取点上方 5cm（OMPL 关节目标规划，IK 以当前构型为种子）
      2. 直线下降到抓取点（笛卡尔直线）
      3. 闭合夹爪
      4. 直线抬起 10cm（笛卡尔直线）
      5. 移动到放置点（OMPL 关节目标规划，IK 以当前构型为种子）
      6. 张开夹爪
      7. 回到观察位姿（OMPL 关节目标规划，IK 以当前构型为种子）
    """

    # xarm7 规划组的活动关节（按组内变量顺序）
    # 注：world_joint / joint_eef / gripper_fix / joint_tcp 均为 fixed 关节，
    #     不是组变量，setJointValueTarget 只需 7 个值。
    ARM_JOINT_NAMES = [f'joint{i}' for i in range(1, 8)]

    # 固定位姿（link_base 坐标系）
    # 观察位姿：与 go_home.py 的 home_pose 一致  px=0.35, py=0.2, pz=0.38, qx=1.0, qy=0.0, qz=0.0, qw=0.0
    WATCH_POSE = Pose(
        position=Point(x=0.35, y=0.2, z=0.38),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    )                     
    # (px=0.450, py=-0.25, pz=0.25, qx=1.0, qy=0.0, qz=0.0, qw=0.0)
    PLACE_POSE = Pose( 
        position=Point(x=0.45, y=-0.25, z=0.25),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    )
    GRIPPER_OPEN  = 0.1
    GRIPPER_CLOSE = 0.8

    def __init__(self):
        super().__init__('grasp_policy_xarm_joint')

        # ---- 抓取状态 ----
        self._grasp_pose_base = None   # Pose，link_base 下的抓取位姿
        self._grasp_width   = 0.0      # 抓取宽度（米）
        self._running       = False    # 抓取序列是否正在执行
        self._lock          = threading.Lock()

        # ---- 当前关节状态缓存（IK 种子用） ----
        self._joint_state_lock   = threading.Lock()
        self._latest_joint_state = None  # sensor_msgs/JointState

        # ---- TF2 ----
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- xarm_planner_node 服务客户端（机械臂） ----
        self._pose_plan_client     = self.create_client(PlanPose, 'xarm_pose_plan')
        self._joint_plan_client    = self.create_client(PlanJoint, 'xarm_joint_plan')
        self._straight_plan_client = self.create_client(PlanSingleStraight, 'xarm_straight_plan')
        self._exec_plan_client     = self.create_client(PlanExec, 'xarm_exec_plan')

        # ---- move_group 的 IK 服务客户端（方案1 核心） ----
        # 服务由 move_group 节点自动提供，无需额外 launch
        self._compute_ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # ---- xarm_gripper_planner_node 服务客户端（夹爪） ----
        self._gripper_joint_plan_client = self.create_client(PlanJoint, 'xarm_gripper_joint_plan')
        self._gripper_exec_plan_client  = self.create_client(PlanExec, 'xarm_gripper_exec_plan')

        # ---- 订阅 ----
        self.create_subscription(Float32,     '/grasp_width', self._cb_grasp_width, 10)
        self.create_subscription(PoseStamped, '/grasp_pose',  self._cb_grasp_pose,  10)
        self.create_subscription(JointState,  '/joint_states', self._cb_joint_states, 10)

        # 【重要】不要在 __init__ 里移动机械臂！
        # 此时 main() 中的执行器未创建、尚未 spin，服务响应回调不会被处理，
        # 同步服务调用会永久阻塞（卡死）。移动到观察位姿放到 main() 中、
        # 执行器开始 spin 之后再执行（见 main 函数）。

    # ============================================================
    #  底层辅助：等待服务并同步调用
    # ============================================================
    def _call_service_full(self, client, request, timeout=60.0):
        """
        等待服务就绪并同步调用，返回响应对象（失败返回 None）。
        注意：需在非 spin 线程中调用（执行器在其他线程中运行，
        会自动处理服务响应回调）。
        【重要】不能使用 client.call()：它内部是无超时的 event.wait()，
        一旦响应无法送达会永久卡死。这里用 call_async + 带超时的
        event.wait(timeout) 手动实现同步等待，超时后返回 None。
        """
        service_name = client.srv_name
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('节点被中断，等待服务时退出。')
                return None
            self.get_logger().warn(f'服务 {service_name} 不可用，继续等待 ...')

        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        if not future.done():
            if not event.wait(timeout):
                self.get_logger().error(f'调用服务 {service_name} 超时 ({timeout}s)')
                return None
        if future.result() is None:
            self.get_logger().error(f'调用服务 {service_name} 失败: {future.exception()}')
            return None
        response = future.result()
        self.get_logger().info(f'调用服务 {service_name}, success={getattr(response, "success", "ok")}')
        return response

    def _call_service(self, client, request, timeout=60.0) -> bool:
        """同步调用服务，返回响应的 success 字段（仅用于 xarm_msgs 服务）。"""
        response = self._call_service_full(client, request, timeout)
        return response is not None and response.success

    # ============================================================
    #  话题回调
    # ============================================================
    def _cb_joint_states(self, msg: JointState):
        """缓存最新关节状态（用于构造 IK 种子）"""
        with self._joint_state_lock:
            self._latest_joint_state = msg

    def _cb_grasp_width(self, msg: Float32):
        self._grasp_width = msg.data
        self.get_logger().info(
            f'收到抓取宽度: '
            f'width={self._grasp_width:.4f}')

    def _cb_grasp_pose(self, msg: PoseStamped):
        """收到抓取姿态（相机坐标系），变换到基坐标系后触发抓取序列"""
        if self._running:
            self.get_logger().warn('上一次抓取序列尚未完成，忽略本次检测结果。')
            return

        self.get_logger().info(
            f'收到抓取姿态（相机坐标系）: '
            f'pos=[{msg.pose.position.x:.4f}, {msg.pose.position.y:.4f}, {msg.pose.position.z:.4f}]  '
            f'ori=[{msg.pose.orientation.x:.4f}, {msg.pose.orientation.y:.4f}, {msg.pose.orientation.z:.4f}, {msg.pose.orientation.w:.4f}]  '
        )

        # 1. 坐标系变换：camera_color_optical_frame → link_base
        # 得到基坐标系下的抓取位姿
        pose_base = self._transform_to_base(msg)
        if pose_base is None:
            return

        self._grasp_pose_base = pose_base
        self.get_logger().info(
            f'转换后抓取姿态（基坐标系）: '
            f'pos=[{pose_base.position.x:.4f}, {pose_base.position.y:.4f}, {pose_base.position.z:.4f}]  '
            f'ori=[{pose_base.orientation.x:.4f}, {pose_base.orientation.y:.4f}, {pose_base.orientation.z:.4f}, {pose_base.orientation.w:.4f}]  '
        )

        # 2. 在后台线程中执行抓取序列（避免阻塞 spin 回调链）
        with self._lock:
            self._running = True
        t = threading.Thread(target=self._run_grasp_sequence, daemon=True)
        t.start()

    # ============================================================
    #  方案1 核心：IK 求解（以当前构型为种子）
    # ============================================================
    def _get_current_arm_joints(self, timeout=5.0):
        """
        从 /joint_states 缓存中提取 xarm7 的 7 个关节角（按 joint1~joint7 顺序）。
        返回 list[float]，超时未收到消息返回 None。
        """
        # 等待第一条 /joint_states
        waited = 0.0
        while rclpy.ok():
            with self._joint_state_lock:
                js = self._latest_joint_state
            if js is not None:
                break
            if waited >= timeout:
                self.get_logger().error(f'等待 /joint_states 超时 ({timeout}s)，无法获取当前关节角。')
                return None
            time.sleep(0.1)  # 睡眠期间执行器线程仍可收到话题消息
            waited += 0.1

        name_to_pos = {}
        for name, pos in zip(js.name, js.position):
            name_to_pos[name] = float(pos)

        missing = [n for n in self.ARM_JOINT_NAMES if n not in name_to_pos]
        if missing:
            self.get_logger().error(f'/joint_states 中缺少机械臂关节: {missing}')
            return None
        return [name_to_pos[n] for n in self.ARM_JOINT_NAMES]

    def _solve_ik_near_current(self, target_pose: Pose):
        """
        调用 move_group 的 /compute_ik 服务，以【当前关节角为种子】求解
        target_pose 对应的 7 关节解。

        这是方案1 的关键：数值 IK（KDL）从当前构型出发迭代收敛，
        得到的解与当前构型连续（不会出现肘部翻转/腕关节转180°），
        从而让 OMPL 的关节目标规划路径自然、平滑。

        返回 list[float]（joint1~joint7，弧度），失败返回 None。
        """
        # 1. 获取当前关节角作为 IK 种子
        current_joints = self._get_current_arm_joints()
        if current_joints is None:
            return None

        # 2. 构造 GetPositionIK 请求
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'xarm7'
        # ik_link_name 留空 → 使用规划组默认末端 link（带夹爪时为 link_tcp），
        # 与原版 xarm_pose_plan 内部 setPoseTarget 的目标 link 完全一致
        req.ik_request.pose_stamped.header.frame_id = 'link_base'
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose = target_pose
        # 用当前关节角作为种子状态（决定解落在哪个冗余分支上）
        req.ik_request.robot_state.joint_state.name = list(self.ARM_JOINT_NAMES)
        req.ik_request.robot_state.joint_state.position = [float(j) for j in current_joints]
        req.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.avoid_collisions = True
        timeout_dur = Duration(sec=0, nanosec=100_000_000)  # 0.1 秒（每次 IK 求解的时限）
        req.ik_request.timeout = timeout_dur

        # 3. 同步调用 /compute_ik
        res = self._call_service_full(self._compute_ik_client, req, timeout=15.0)
        if res is None:
            return None

        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'/compute_ik 求解失败 (error_code={res.error_code.val})，'
                f'目标位姿可能不可达或与种子构型不连续。')
            return None

        # 4. 从解中按 joint1~joint7 顺序提取关节角
        js = res.solution.joint_state
        name_to_pos = {n: float(p) for n, p in zip(js.name, js.position)}
        missing = [n for n in self.ARM_JOINT_NAMES if n not in name_to_pos]
        if missing:
            self.get_logger().error(f'IK 解中缺少关节: {missing}')
            return None
        joint_solution = [name_to_pos[n] for n in self.ARM_JOINT_NAMES]

        # 5. 打印与当前构型的偏差（直观体现"解贴近当前构型"）
        deltas = [sol - cur for sol, cur in zip(joint_solution, current_joints)]
        delta_str = ', '.join(f'{d:+.3f}' for d in deltas)
        self.get_logger().info(
            f'IK 求解成功: joints=[{", ".join(f"{j:.3f}" for j in joint_solution)}]  '
            f'与当前构型关节差=[{delta_str}] rad (Σ|Δ|={sum(abs(d) for d in deltas):.3f})')
        return joint_solution

    # ============================================================
    #  调整旋转矩阵
    # ============================================================
    def adjust_rotation_matrix(self, R):
        # Original coordinate system base vector (Z-axis)
        original_z = np.array([0, 0, 1])
        rotated_z = np.dot(R, original_z)
        # Calculate the angle between the rotated z-axis and the original z-axis
        angle_z = utool.angle_between_vectors(rotated_z, original_z)
        if not (90 < angle_z < 180 or -180 < angle_z < -90):
            self.get_logger().error( f'抓取姿态 Z 轴角度异常 ({angle_z:.1f}°)，跳过本次抓取。')

        # Original coordinate system's x-axis
        original_x = np.array([1, 0, 0])
        rotated_x = np.dot(R, original_x)

        # Calculate the angle between the rotated x-axis and the original x-axis
        angle_x = utool.angle_between_vectors(rotated_x, original_x)
        if not (-90 <= angle_x <= 90):
            # Rotate 180 degrees around the z-axis (adjustment matrix)
            R_adjusted = np.dot(R, np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]))
            R = R_adjusted
            self.get_logger().info(f'X 轴方向调整 (angle_x={angle_x:.1f}°)')

        return angle_x, R

    # ============================================================
    #  TF2 坐标变换：camera_color_optical_frame → link_base
    # ============================================================
    def _transform_to_base(self, pose_msg: PoseStamped):
        """
        将相机坐标系下的 PoseStamped 变换到 link_base 坐标系。
        参考 ROS1 Policy_xArm7.updateGraspPose() 中的 T_base_camera 计算。
        """
        try:
            trans_camera = self._tf_buffer.lookup_transform(
                'link_base', 'camera_color_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except Exception as e:
            self.get_logger().error(f'TF2 查询失败 (link_base ← camera_color_optical_frame): {e}')
            return None

        # 构造 T_base_camera（4×4 齐次变换矩阵）
        t_base_camera = np.array([
            trans_camera.transform.translation.x, 
            trans_camera.transform.translation.y, 
            trans_camera.transform.translation.z])
        q_base_camera = np.array([
           - trans_camera.transform.rotation.x, 
           - trans_camera.transform.rotation.y, 
           - trans_camera.transform.rotation.z,
           - trans_camera.transform.rotation.w])
        T_base_camera = utool.getTransformMat(t_base_camera, utool.quaternion_to_rotation_matrix(q_base_camera))

        # 构造 T_camera_grasp
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        t_camera_grasp = np.array([
            p.x,
            p.y,
            p.z])
        q_camera_grasp = np.array([
            q.x,
            q.y,
            q.z,
            q.w])
        T_camera_grasp = utool.getTransformMat(t_camera_grasp, utool.quaternion_to_rotation_matrix(q_camera_grasp))

        # 得到 T_base_grasp变换矩阵
        T_base_grasp = np.matmul(T_base_camera, T_camera_grasp)

        # ---- 姿态调整（与 ROS1 adjust_rotation_matrix 等效） ----
        # 绕 Y 轴旋转 90°，使夹爪 Z 轴朝下方（符合 xArm 抓取姿态约定）
        T_y90 = np.array([
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1],
        ], dtype=float)
        T_base_grasp = np.matmul(T_base_grasp, T_y90)

        # 验证 Z 轴朝向：抓取方向应与基坐标系 -Z 夹角在 (90°, 180°)
        # 若 X 轴方向异常（不在 ±90° 内），绕 Z 轴翻转 180°
        _, adjusted_R = self.adjust_rotation_matrix(T_base_grasp[:3, :3])
        T_base_grasp[:3, :3] = adjusted_R

        # 转换为 Pose
        q_base_grasp = utool.rotation_matrix_to_quaternion(T_base_grasp[:3, :3])  # 返回 [x, y, z, w]
        q_base_grasp_normalized = q_base_grasp / np.linalg.norm(q_base_grasp)
        result = Pose()
        result.position = Point(
            x=float(T_base_grasp[0, 3]),
            y=float(T_base_grasp[1, 3]),
            z=float(max(T_base_grasp[2, 3], 0.02)),  # 高度下限保护
        )
        result.orientation = Quaternion(
            x=float(q_base_grasp_normalized[0]), y=float(q_base_grasp_normalized[1]),
            z=float(q_base_grasp_normalized[2]), w=float(q_base_grasp_normalized[3]),
        )
        return result

    # ============================================================
    #  运动确认
    # ============================================================
    def _confirm_planned_path(self) -> str:
        """
        规划成功后请求用户三选一确认（规划路径可在 RViz 中查看）：
          y -> 确认执行，返回 'execute'
          r -> 重新规划，返回 'replan'（调用方需重新发起路径规划）
          n -> 取消执行，返回 'cancel'
        """
        while True:
            print('请用户输入确认指令：')
            confirm = input('输入 y 确认执行 / 输入 r 重新规划 / 输入 n 取消执行: ').strip().lower()
            if confirm == 'y':
                return 'execute'
            if confirm == 'r':
                return 'replan'
            if confirm == 'n':
                return 'cancel'
            print('输入无效，请输入 y / r / n。')

    # ============================================================
    #  机械臂运动接口（基于 xarm_planner 服务）
    # ============================================================
    
    def _execute_arm_plan(self, wait: bool = True) -> bool:
        """执行机械臂最近一次规划成功的轨迹，wait=True 时阻塞直到运动完成"""
        req = PlanExec.Request()
        req.wait = wait
        return self._call_service(self._exec_plan_client, req, timeout=60.0)

    def _move_arm_free(self, target_pose: Pose) -> bool:
        """
        【方案1】自由移动到指定位姿（大范围移动用）。
        流程：
          1. /joint_states 获取当前关节角
          2. /compute_ik 以当前构型为种子求目标位姿的 7 关节解
          3. xarm_joint_plan 用关节目标做 OMPL 规划（冗余自由度锁定在当前构型附近）
          4. 用户确认 → xarm_exec_plan(wait=True) 执行
        IK 失败时自动回退到原版 xarm_pose_plan 位姿规划。
        """
        self.get_logger().info(
            f'自由移动到目标 -> 位置: X={target_pose.position.x}, Y={target_pose.position.y}, Z={target_pose.position.z} '
            f'姿态四元数: [{target_pose.orientation.x}, {target_pose.orientation.y}, {target_pose.orientation.z}, {target_pose.orientation.w}]')

        # Phase 0: 以当前构型为种子求解 IK，得到关节目标
        joint_target = self._solve_ik_near_current(target_pose)
        if joint_target is None:
            self.get_logger().warn('IK（当前构型种子）求解失败，回退到 xarm_pose_plan 位姿规划。')
            return self._move_arm_free_pose_fallback(target_pose)

        # 支持重新规划：整个 规划 → 确认 流程放在循环里，
        # 用户选择 r 时回到循环头重新发起规划（同一关节目标，OMPL 重新采样路径）。
        while True:
            # Phase 1: 关节目标 OMPL 规划
            req = PlanJoint.Request()
            req.target = [float(j) for j in joint_target]  # joint1~joint7 共 7 个值
            if not self._call_service(self._joint_plan_client, req, timeout=30.0):
                self.get_logger().error('关节目标 OMPL 规划失败（目标构型不可达或存在碰撞）。')
                return False

            self.get_logger().info('关节目标路径规划成功！请在 RViz 中查看规划路径。')
            # Phase 2: 请求用户确认（三选一）
            decision = self._confirm_planned_path()
            if decision == 'replan':
                self.get_logger().warn('用户选择重新规划，正在重新规划路径...')
                continue
            if decision == 'cancel':
                self.get_logger().warn('用户已取消本次运动指令。')
                return False

            # Phase 3: 用户确认，执行已规划好的轨迹
            self.get_logger().info('用户已确认，正在执行规划路径...')
            if self._execute_arm_plan(wait=True):
                self.get_logger().info('============ 运动顺利到达目标点！ ============')
                return True
            self.get_logger().error('轨迹执行失败！')
            return False

    def _move_arm_free_pose_fallback(self, target_pose: Pose) -> bool:
        """
        原版位姿规划（grasp_policy_xarm.py 的 _move_arm_free），
        作为 IK 求解失败时的回退路径。含用户确认机制。
        调用链: xarm_pose_plan → xarm_exec_plan(wait=True)
        """
        self.get_logger().info('等待 xarm_planner_node 服务响应...')
        while True:
            # Phase 1: 规划
            req = PlanPose.Request()
            req.target = target_pose
            if not self._call_service(self._pose_plan_client, req, timeout=30.0):
                self.get_logger().error('OMPL 位姿规划失败，请检查目标点是否在可达范围内。')
                return False

            self.get_logger().info('路径规划成功！请在 RViz 中查看规划路径。')
            # Phase 2: 请求用户确认（三选一）
            decision = self._confirm_planned_path()
            if decision == 'replan':
                self.get_logger().warn('用户选择重新规划，正在重新规划路径...')
                continue
            if decision == 'cancel':
                self.get_logger().warn('用户已取消本次运动指令。')
                return False

            # Phase 3: 用户确认，执行已规划好的轨迹
            self.get_logger().info('用户已确认，正在执行规划路径...')
            if self._execute_arm_plan(wait=True):
                self.get_logger().info('============ 运动顺利到达目标点！ ============')
                return True
            self.get_logger().error('轨迹执行失败！')
            return False

    def _move_arm_straight(self, target_pose: Pose) -> bool:
        """
        用笛卡尔直线路径移动机械臂到目标位姿（含用户确认机制：确认执行 / 重新规划 / 取消执行）。
        适用于近距离精确移动，如下降抓取/抬起。
        调用链: xarm_straight_plan → xarm_exec_plan(wait=True)
        """
        self.get_logger().info(
            f'直线运动目标 -> 位置: X={target_pose.position.x}, Y={target_pose.position.y}, Z={target_pose.position.z} '
            f'姿态四元数: [{target_pose.orientation.x}, {target_pose.orientation.y}, {target_pose.orientation.z}, {target_pose.orientation.w}]')

        # 支持重新规划：整个 规划 → 确认 流程放在循环里。
        while True:
            # Phase 1: 规划
            req = PlanSingleStraight.Request()
            req.target = target_pose
            if not self._call_service(self._straight_plan_client, req, timeout=30.0):
                self.get_logger().error('笛卡尔直线规划失败（路径覆盖率不足或存在奇异点）。')
                return False

            self.get_logger().info('直线规划成功！请在 RViz 中查看规划路径。')
            # Phase 2: 请求用户确认（三选一）
            decision = self._confirm_planned_path()
            if decision == 'replan':
                self.get_logger().warn('用户选择重新规划，正在重新规划路径...')
                continue
            if decision == 'cancel':
                self.get_logger().warn('用户已取消本次运动指令。')
                return False

            # Phase 3: 用户确认，执行已规划好的轨迹
            self.get_logger().info('用户已确认，正在执行规划路径...')
            if self._execute_arm_plan(wait=True):
                self.get_logger().info('============ 运动顺利到达目标点！ ============')
                return True
            self.get_logger().error('轨迹执行失败！')
            return False


    # ============================================================
    #  夹爪控制（xarm_gripper_planner_node 服务）
    # ============================================================
    def _move_gripper(self, position: float) -> bool:
        """
        控制 xArm 夹爪到指定开合位置（0.0=张开, 0.85=闭合）。
        调用链: xarm_gripper_joint_plan → xarm_gripper_exec_plan(wait=True)
        """
        self.get_logger().info(f'控制夹爪到位置 {position}')
        req = PlanJoint.Request()
        req.target = [position]* 6   # xarm_gripper 组含 1 个活动关节 + 5 个 mimic 关节，变量总数为 6
        if not self._call_service(self._gripper_joint_plan_client, req, timeout=15.0):
            self.get_logger().error('夹爪规划失败')
            return False

        exec_req = PlanExec.Request()
        exec_req.wait = True
        return self._call_service(self._gripper_exec_plan_client, exec_req, timeout=30.0)

    # ============================================================
    #  抓取序列（7 步）
    # ============================================================
    def _run_grasp_sequence(self):
        """
        执行完整抓取序列:
          1. 自由规划到预抓取点（grasp + 5cm）【关节目标，IK 以当前构型为种子】
          2. 直线下降到抓取点
          3. 闭合夹爪
          4. 直线抬起（grasp + 10cm）
          5. 移动到放置点【关节目标】
          6. 张开夹爪
          7. 回到观察位姿【关节目标】
        """
        try:
            grasp = self._grasp_pose_base
            w     = self._grasp_width
            self.get_logger().info('=' * 50)
            self.get_logger().info('  开始抓取序列')
            self.get_logger().info('=' * 50)

            # ---- 计算预抓取点与抬起点位姿 ----
            pre_grasp = Pose()
            pre_grasp.position    = Point(x=grasp.position.x, y=grasp.position.y,
                                          z=grasp.position.z + 0.05)
            pre_grasp.orientation = grasp.orientation

            lift_grasp = Pose()
            lift_grasp.position    = Point(x=grasp.position.x, y=grasp.position.y,
                                           z=grasp.position.z + 0.10)
            lift_grasp.orientation = grasp.orientation

            # Step 1: 预抓取（OMPL 关节目标规划，IK 种子=当前构型）
            self.get_logger().info('[1/7] 移动到预抓取点（OMPL 关节目标路径）')
            if not self._move_arm_free(pre_grasp):
                self._abort()
                return

            # Step 2: 到达抓取点（笛卡尔直线，直插到目标）
            self.get_logger().info('[2/7] 移动到抓取点（笛卡尔直线）')
            if not self._move_arm_straight(grasp):
                self._abort()
                return

            # Step 3: 闭合夹爪
            self.get_logger().info(f'[3/7] 闭合夹爪 (drive_joint={self.GRIPPER_CLOSE})')
            if not self._move_gripper(self.GRIPPER_CLOSE):
                self._abort()
                return

            # Step 4: 抬起（笛卡尔直线，向上 10cm）
            self.get_logger().info('[4/7] 抬起物体（笛卡尔直线）')
            if not self._move_arm_straight(lift_grasp):
                self._abort()
                return

            # Step 5: 移动到放置点（OMPL 关节目标规划，距离较大）
            self.get_logger().info('[5/7] 移动到放置点（OMPL 关节目标）')
            if not self._move_arm_free(self.PLACE_POSE):
                self._abort()
                return

            # Step 6: 张开夹爪，释放物体
            self.get_logger().info(f'[6/7] 张开夹爪 (drive_joint={self.GRIPPER_OPEN})')
            if not self._move_gripper(self.GRIPPER_OPEN):
                self._abort()
                return

            # Step 7: 回到观察位姿（OMPL 关节目标规划）
            self.get_logger().info('[7/7] 回到观察位姿（OMPL 关节目标）')
            if not self._move_arm_free(self.WATCH_POSE):
                self._abort()
                return

            self.get_logger().info('=' * 55)
            self.get_logger().info('  抓取序列完成！等待下一次检测结果...')
            self.get_logger().info('=' * 55)

        except Exception as e:
            self.get_logger().error(f'抓取序列异常中断: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            with self._lock:
                self._running = False

    def _abort(self):
        self.get_logger().info('=' * 55)
        self.get_logger().error('抓取序列中止，等待下一次检测...')
        self.get_logger().info('=' * 55)
        with self._lock:
            self._running = False


# ============================================================
#  入口
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = GraspPolicyXarmJoint()

    # 使用多线程执行器，支持 TF2 回调与抓取序列并发
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # 在后台线程运行 spin，主线程保持可中断
    # （抓取序列线程中的同步服务调用由该执行器负责处理响应）
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # 【重要】移动到观察位姿必须放在执行器开始 spin 之后！
        # 同步服务调用的响应由正在 spin 的执行器送达；若在节点构造函数里
        # （spin 之前）调用，会永久卡死且无任何超时提示。
        node.get_logger().info('  移动到观察位姿并打开夹爪...')
        node._move_arm_free(node.WATCH_POSE)
        node._move_gripper(node.GRIPPER_OPEN)  # 打开夹爪
        node.get_logger().info('='*55)
        node.get_logger().info('  GraspPolicyXarmJoint 节点初始化完成')
        node.get_logger().info('  等待 /grasp_pose 话题（来自 detect_grasp.py）...')
        node.get_logger().info('='*55)

        spin_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，节点退出。')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

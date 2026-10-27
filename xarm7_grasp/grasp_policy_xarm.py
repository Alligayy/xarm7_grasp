
# 实际在使用的抓取策略

# 集成机械臂的抓取控制策略代码 (ROS2)
# 接收 GSNet 发布的抓取姿态，执行抓取动作
# 使用 xarm_ros2 的 xarm_planner 规划机械臂运动
#
# 用法:
#   ros2 run xarm7_grasp grasp_policy_xarm
#
# 前提:
#   1. MoveIt2 + xArm7 + xarm_planner_node 已启动
#      (grasp_simulation.launch.py 或 grasp_real.launch.py 会自动启动规划服务节点)
#   2. RealSense 相机已启动，手眼标定 TF 已发布
#   3. detect_grasp.py 节点已运行（发布 /grasp_pose 和 /grasp_width）
#
# 相比 grasp_policy.py（手动构造 MoveIt2 MoveGroup 动作请求），
# 本文件直接调用 xarm_planner_node / xarm_gripper_planner_node 提供的服务：
#   机械臂 (xarm_planner_node):
#     - xarm_pose_plan       (xarm_msgs/srv/PlanPose)           : 位姿自由规划
#     - xarm_straight_plan   (xarm_msgs/srv/PlanSingleStraight) : 笛卡尔直线规划
#     - xarm_exec_plan       (xarm_msgs/srv/PlanExec)           : 执行已规划轨迹
#   夹爪 (xarm_gripper_planner_node):
#     - xarm_gripper_joint_plan (xarm_msgs/srv/PlanJoint)       : 夹爪关节规划
#     - xarm_gripper_exec_plan  (xarm_msgs/srv/PlanExec)        : 执行夹爪轨迹

import sys
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# 增加tool.py 文件所在的路径
sys.path.append('/media/gao/2/Grasp/grasp_ws/src/xarm7_grasp')
from utils import tool as utool

import tf2_ros
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from xarm_msgs.srv import PlanPose, PlanJoint, PlanExec, PlanSingleStraight
from .tactile_gripper import TactileGripper


# ============================================================
#  抓取策略节点
# ============================================================
class GraspPolicyXarm(Node):
    """
    接收 GSNet 检测节点发布的抓取姿态，
    通过 TF2 完成坐标系变换，按 7 步序列执行抓取。

    抓取序列（参考 grasp_policy.py / ROS1 Policy_xArm7）：
      1. 移动到预抓取点上方 5cm（OMPL 自由规划）
      2. 直线下降到抓取点（笛卡尔直线）
      3. 闭合夹爪
      4. 直线抬起 10cm（笛卡尔直线）
      5. 移动到放置点（OMPL 自由规划）
      6. 张开夹爪
      7. 回到观察位姿（固定关节角，锁定 7 自由度冗余臂自运动）
    """

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

    # 夹爪抓取深度补偿 (米, 沿接近方向, 正=向下加深)。
    # 根因: URDF 中 link_tcp 位于夹爪基座前方 0.172m, 但 GSNet 抓取中心(物体应处位置)
    # 对应物理夹爪指垫中心(约 0.10~0.11m)。带夹爪时 SRDF 将 joint_tcp 并入规划链,
    # MoveIt 会把 link_tcp 放到抓取中心 G, 导致指垫中心停在 G 上方约 6cm,
    # 夹爪在物体上缘/侧缘闭合 -> 擦边。补偿量 delta = 0.172 - 指垫中心到基座距离。
    # 默认 0.05 为估算值, 务必实测校准: 若仍偏浅(夹上缘)则调大, 若偏深(顶到桌面)则调小/取负。
    GRASP_DEPTH_OFFSET = 0.04

    # 观察位姿（固定关节角，joint1~joint7，单位：度）
    # 用固定关节角回观察位姿，可锁定 7 自由度冗余臂的自运动，
    # 保证每次回到完全相同的整臂构型；且关节角目标不涉及 IK，
    # 规避位姿目标随机 IK 种子导致的肘部翻转/腕关节转180°等离谱路径。
    WATCH_JOINTS_DEG = [15.0, -13.0, 12.0, 78.0, 3.0, 91.0, 27.0]
    # 规划服务需要弧度制关节角
    WATCH_JOINTS = [float(np.radians(d)) for d in WATCH_JOINTS_DEG]

    def __init__(self):
        super().__init__('grasp_policy_xarm')

        # ---- 抓取状态 ----
        self._grasp_pose_base = None   # Pose，link_base 下的抓取位姿
        self._grasp_width   = 0.0      # 抓取宽度（米）
        self._running       = True    # 抓取序列是否正在执行
        self._lock          = threading.Lock()

        self._force = TactileGripper(self)

        # ---- TF2 ----
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- xarm_planner_node 服务客户端（机械臂） ----
        self._pose_plan_client     = self.create_client(PlanPose, 'xarm_pose_plan')
        self._joint_plan_client    = self.create_client(PlanJoint, 'xarm_joint_plan')
        self._straight_plan_client = self.create_client(PlanSingleStraight, 'xarm_straight_plan')
        self._exec_plan_client     = self.create_client(PlanExec, 'xarm_exec_plan')

        # ---- xarm_gripper_planner_node 服务客户端（夹爪） ----
        self._gripper_joint_plan_client = self.create_client(PlanJoint, 'xarm_gripper_joint_plan')
        self._gripper_exec_plan_client  = self.create_client(PlanExec, 'xarm_gripper_exec_plan')

        # ---- 订阅 ----
        self.create_subscription(Float32,     '/grasp_width', self._cb_grasp_width, 10)
        self.create_subscription(PoseStamped, '/grasp_pose',  self._cb_grasp_pose,  10)

        # ---- 抓取姿态可视化发布器（RViz MarkerArray） ----
        # 每次收到检测结果并转换到 link_base 后，发布抓取姿态坐标轴与预抓取点，
        # 在 RViz 中添加 MarkerArray 显示、话题选 /grasp_pose_marker 即可看到。
        self._marker_pub = self.create_publisher(MarkerArray, '/grasp_pose_marker', 10)

        # 【重要】不要在 __init__ 里移动机械臂！
        # 此时 main() 中的执行器未创建、尚未 spin，服务响应回调不会被处理，
        # 同步服务调用会永久阻塞（卡死）。移动到观察位姿放到 main() 中、
        # 执行器开始 spin 之后再执行（见 main 函数）。

    # ============================================================
    #  底层辅助：等待服务并同步调用
    # ============================================================
    def _call_service(self, client, request, timeout=60.0) -> bool:
        """
        等待服务就绪并同步调用，返回响应的 success 字段。
        注意：需在非 spin 线程中调用（执行器在其他线程中运行，
        会自动处理服务响应回调）。
        【重要】不能使用 client.call()：它内部是无超时的 event.wait()，
        一旦响应无法送达会永久卡死。这里用 call_async + 带超时的
        event.wait(timeout) 手动实现同步等待，超时后返回 False。
        """
        service_name = client.srv_name
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('节点被中断，等待服务时退出。')
                return False
            self.get_logger().warn(f'服务 {service_name} 不可用，继续等待 ...')

        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        if not future.done():
            if not event.wait(timeout):
                self.get_logger().error(f'调用服务 {service_name} 超时 ({timeout}s)')
                return False
        if future.result() is None:
            self.get_logger().error(f'调用服务 {service_name} 失败: {future.exception()}')
            return False
        response = future.result()
        self.get_logger().info(f'调用服务 {service_name}, success={response.success}')
        return response.success

    # ============================================================
    #  话题回调
    # ============================================================
    def _cb_grasp_width(self, msg: Float32):
        self._grasp_width = msg.data
        self.get_logger().info(
            f'收到抓取宽度: '
            f'width={self._grasp_width:.4f}')

    def _cb_grasp_pose(self, msg: PoseStamped):
        """收到抓取姿态（相机坐标系），变换到基坐标系后触发抓取序列"""
        if self._force.fault:
            self.get_logger().error(f'力控故障已锁定，人工检查后重启: {self._force.fault}')
            return
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

        # 1.5 在 RViz 中可视化本次抓取姿态（坐标轴 + 预抓取点）
        self._publish_grasp_markers(pose_base)

        # 2. 在后台线程中执行抓取序列（避免阻塞 spin 回调链）
        with self._lock:
            self._running = True
        t = threading.Thread(target=self._run_grasp_sequence, daemon=True)
        t.start()

    # ============================================================
    # 调整旋转矩阵
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

        # ---- 夹爪抓取深度补偿 ----
        # 工具 Z 轴(第3列)即接近方向(朝下)。沿该方向平移 GRASP_DEPTH_OFFSET,
        # 使物理指垫中心(而非 link_tcp)落在 GSNet 抓取中心上, 消除擦边系统偏差。
        approach = T_base_grasp[:3, 2]
        T_base_grasp[:3, 3] += approach * self.GRASP_DEPTH_OFFSET
        self.get_logger().info(
            f'已施加抓取深度补偿: {self.GRASP_DEPTH_OFFSET:+.3f} m (沿接近方向)')

        # 转换为 Pose
        q_base_grasp = utool.rotation_matrix_to_quaternion(T_base_grasp[:3, :3])  # 返回 [x, y, z, w]
        q_base_grasp_normalized = q_base_grasp / np.linalg.norm(q_base_grasp)
        result = Pose()
        result.position = Point(
            x=float(T_base_grasp[0, 3]),
            y=float(T_base_grasp[1, 3]),
            z=float(max(T_base_grasp[2, 3], 0.025)),  # 高度下限保护，最低 0.025m
        )
        result.orientation = Quaternion(
            x=float(q_base_grasp_normalized[0]), y=float(q_base_grasp_normalized[1]),
            z=float(q_base_grasp_normalized[2]), w=float(q_base_grasp_normalized[3]),
        )
        return result

    # ============================================================
    #  抓取姿态可视化（发布 MarkerArray 到 RViz）
    # ============================================================
    def _make_arrow_marker(self, marker_id, start, direction, color, length=0.10):
        """构造一个从 start 沿 direction 方向的箭头 Marker（用于表示坐标轴）"""
        m = Marker()
        m.header.frame_id = 'link_base'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'grasp_axes'
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        dir_norm = np.array(direction, dtype=float)
        norm = np.linalg.norm(dir_norm)
        if norm < 1e-9:
            return None
        dir_norm = dir_norm / norm
        end = np.array([start.x, start.y, start.z], dtype=float) + dir_norm * length
        m.points = [
            Point(x=float(start.x), y=float(start.y), z=float(start.z)),
            Point(x=float(end[0]), y=float(end[1]), z=float(end[2])),
        ]
        m.scale.x = 0.008   # 箭杆直径
        m.scale.y = 0.016   # 箭头直径
        m.scale.z = 0.0
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # 0 = 常驻，直到下一次覆盖
        return m

    def _publish_grasp_markers(self, pose_base: Pose):
        """
        将转换到 link_base 的抓取姿态以坐标轴（X红/Y绿/Z蓝）形式发布到 RViz，
        并额外标记预抓取点（抓取点上方 5cm）与抓取点。
        """
        if self._marker_pub is None:
            return

        q = pose_base.orientation
        R = utool.quaternion_to_rotation_matrix(np.array([q.x, q.y, q.z, q.w]))
        x_axis = np.dot(R, np.array([1.0, 0.0, 0.0]))
        y_axis = np.dot(R, np.array([0.0, 1.0, 0.0]))
        z_axis = np.dot(R, np.array([0.0, 0.0, 1.0]))
        start = pose_base.position

        marker_array = MarkerArray()

        # 抓取姿态坐标轴（id 0/1/2）
        for marker_id, axis, color in (
            (0, x_axis, (1.0, 0.0, 0.0, 1.0)),   # X 轴 - 红
            (1, y_axis, (0.0, 1.0, 0.0, 1.0)),   # Y 轴 - 绿
            (2, z_axis, (0.0, 0.0, 1.0, 1.0)),   # Z 轴 - 蓝（抓取接近方向）
        ):
            arrow = self._make_arrow_marker(marker_id, start, axis, color)
            if arrow is not None:
                marker_array.markers.append(arrow)

        # 抓取点（id 3，黄色小球）
        grasp_sphere = Marker()
        grasp_sphere.header.frame_id = 'link_base'
        grasp_sphere.header.stamp = self.get_clock().now().to_msg()
        grasp_sphere.ns = 'grasp_point'
        grasp_sphere.id = 3
        grasp_sphere.type = Marker.SPHERE
        grasp_sphere.action = Marker.ADD
        grasp_sphere.pose.position = Point(x=start.x, y=start.y, z=start.z)
        grasp_sphere.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        grasp_sphere.scale.x = grasp_sphere.scale.y = grasp_sphere.scale.z = 0.02
        grasp_sphere.color.r, grasp_sphere.color.g, grasp_sphere.color.b, grasp_sphere.color.a = (1.0, 1.0, 0.0, 0.9)
        grasp_sphere.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        marker_array.markers.append(grasp_sphere)

        # 预抓取点（id 4，抓取点上方 5cm，青色小球）
        pre_sphere = Marker()
        pre_sphere.header.frame_id = 'link_base'
        pre_sphere.header.stamp = self.get_clock().now().to_msg()
        pre_sphere.ns = 'pre_grasp_point'
        pre_sphere.id = 4
        pre_sphere.type = Marker.SPHERE
        pre_sphere.action = Marker.ADD
        pre_sphere.pose.position = Point(x=start.x, y=start.y, z=start.z + 0.05)
        pre_sphere.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pre_sphere.scale.x = pre_sphere.scale.y = pre_sphere.scale.z = 0.02
        pre_sphere.color.r, pre_sphere.color.g, pre_sphere.color.b, pre_sphere.color.a = (0.0, 1.0, 1.0, 0.9)
        pre_sphere.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        marker_array.markers.append(pre_sphere)

        self._marker_pub.publish(marker_array)
        self.get_logger().info('已发布抓取姿态可视化 Marker 到 /grasp_pose_marker')

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
        self._force.check()
        req = PlanExec.Request()
        req.wait = wait
        return self._call_service(self._exec_plan_client, req, timeout=60.0)

    def _move_arm_to_joints(self, joint_target) -> bool:
        """
        用固定关节角移动机械臂（含用户确认机制：确认执行 / 重新规划 / 取消执行）。
        关节角目标不涉及 IK，冗余自由度被直接锁定，路径平滑可预期，
        适用于回到固定的观察位姿。
        调用链: xarm_joint_plan → xarm_exec_plan(wait=True)
        """
        self.get_logger().info(
            f'关节角运动目标 -> {[round(float(j), 4) for j in joint_target]} rad '
            f'({[round(float(np.degrees(j)), 1) for j in joint_target]}°)')

        # 支持重新规划：整个 规划 → 确认 流程放在循环里。
        while True:
            # Phase 1: 规划
            req = PlanJoint.Request()
            req.target = [float(j) for j in joint_target]  # joint1~joint7 共 7 个值
            if not self._call_service(self._joint_plan_client, req, timeout=30.0):
                self.get_logger().error('关节角规划失败（目标构型不可达或存在碰撞）。')
                return False

            self.get_logger().info('关节角路径规划成功！请在 RViz 中查看规划路径。')
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

    def _move_arm_free(self, target_pose: Pose) -> bool:
        """
        移动到指定位置和姿态（含用户确认机制：确认执行 / 重新规划 / 取消执行）
        用 OMPL 自由规划移动机械臂到目标位姿（适用于大范围移动）。
        调用链: xarm_pose_plan → xarm_exec_plan(wait=True)
        """
        self.get_logger().info('等待 xarm_planner_node 服务响应...')
        self.get_logger().info(
            f'移动到目标位置和姿态 -> 位置: X={target_pose.position.x}, Y={target_pose.position.y}, Z={target_pose.position.z} '
            f'姿态四元数: [{target_pose.orientation.x}, {target_pose.orientation.y}, {target_pose.orientation.z}, {target_pose.orientation.w}]')

        # 支持重新规划：整个 规划 → 确认 流程放在循环里，
        # 用户选择 r 时回到循环头重新发起规划。
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
        req.target = [position]* 6   # xarm_gripper 组只有 drive_joint 一个活动关节
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
          1. 自由规划到预抓取点（grasp + 5cm）
          2. 直线下降到抓取点
          3. 闭合夹爪
          4. 直线抬起（grasp + 10cm）
          5. 移动到放置点
          6. 张开夹爪
          7. 回到观察位姿
        """
        try:
            self._force.check()
            self._force.open()  # 每次接近前确认张开及空载
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

            # Step 1: 预抓取（OMPL 自由规划） 如果_move_arm_free的效果不佳，就将大范围运动规划也改成_move_arm_straight
            self.get_logger().info('[1/7] 移动到预抓取点（OMPL路径）')
            if not self._move_arm_straight(pre_grasp):
                self._abort()
                return

            # Step 2: 到达抓取点（笛卡尔直线，向下 10cm）   
            self.get_logger().info('[2/7] 移动到抓取点（笛卡尔直线）')
            if not self._move_arm_straight(grasp):
                self._abort()
                return

            # Step 3: 闭合夹爪
            self.get_logger().info('[3/7] 触觉闭环夹持，达到两侧目标力后抬起')
            self._force.acquire()

            # Step 4: 抬起（笛卡尔直线，向上 10cm）
            self.get_logger().info('[4/7] 抬起物体（笛卡尔直线）')
            if not self._move_arm_straight(lift_grasp):
                self._abort()
                return

            # Step 5: 移动到放置点（OMPL 自由规划，距离较大）  _move_arm_free
            self.get_logger().info('[5/7] 移动到放置点（OMPL）')
            if not self._move_arm_straight(self.PLACE_POSE):
                self._abort()
                return

            # Step 6: 张开夹爪，释放物体
            self.get_logger().info(f'[6/7] 张开夹爪 (drive_joint={self.GRIPPER_OPEN})')
            self._force.open()

            # Step 7: 回到观察位姿（固定关节角，构型可重复、路径平滑）
            self.get_logger().info('[7/7] 回到观察位姿（固定关节角）')
            if not self._move_arm_to_joints(self.WATCH_JOINTS):
                self._abort()
                return

            self.get_logger().info('=' * 55)
            self.get_logger().info('  抓取序列完成！等待下一次检测结果...')
            self.get_logger().info('=' * 55)

        except Exception as e:
            self._force.trip(str(e))
            self.get_logger().error(f'抓取序列异常中断: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            with self._lock:
                self._running = False

    def _abort(self):
        if self._force.thread and self._force.thread.is_alive():
            self._force.trip('Arm motion failed or cancelled while holding object')
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
    node = GraspPolicyXarm()

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
        node._force.wait_ready()  # 参数、标定及新鲜数据就绪后才允许运动
        node._force.open()
        if not node._move_arm_to_joints(node.WATCH_JOINTS):
            raise RuntimeError('初始化观察位姿运动失败')
        node._running = False
        node.get_logger().info('='*55)
        node.get_logger().info('  GraspPolicyXarm 节点初始化完成')
        node.get_logger().info('  等待 /grasp_pose 话题（来自 detect_grasp.py）...')
        node.get_logger().info('='*55)

        spin_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，节点退出。')
    except Exception as exc:
        node.get_logger().error(f'节点停止: {exc}')
    finally:
        if node._force.thread and node._force.thread.is_alive():
            node._force.trip('Node shutdown during grasp')
        node._force.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

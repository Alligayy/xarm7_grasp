# 集成机械臂的抓取控制策略代码 (ROS2)
# 接收 GSNet 发布的抓取姿态，执行抓取动作
#
# 用法:
#   ros2 run xarm7_grasp grasp_policy
#
# 前提:
#   1. MoveIt2 + xArm7 已启动 (grasp_simulation.launch.py 或 grasp_real.launch.py)
#   2. RealSense 相机已启动，手眼标定 TF 已发布
#   3. detect_grasp.py 节点已运行（发布 /grasp_pose 和 /grasp_width）

import sys
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import tool as utool

import tf2_ros
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    PositionConstraint, OrientationConstraint,
    JointConstraint, BoundingVolume, MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive


# ============================================================
#  四元数 ↔ 旋转矩阵 工具函数
# ============================================================
def angle_between(v1, v2):
    """两个向量间的夹角（度）"""
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 < 1e-12 or n2 < 1e-12:
        return 0.0
    cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_a)))


# ============================================================
#  抓取策略节点
# ============================================================
class GraspPolicy(Node):
    """
    接收 GSNet 检测节点发布的抓取姿态，
    通过 TF2 完成坐标系变换，按 7 步序列执行抓取。

    抓取序列（参考 ROS1 Policy_xArm7）：
      1. 移动到预抓取点上方 5cm（笛卡尔直线）
      2. 直线下降到抓取点（笛卡尔直线）
      3. 闭合夹爪
      4. 直线抬起 10cm（笛卡尔直线）
      5. 移动到放置点（OMPL 自由规划）
      6. 张开夹爪
      7. 回到观察位姿（OMPL 自由规划）
    """

    # 固定位姿（link_base 坐标系）
    # 观察位姿：与 go_home.py 的 home_pose 一致
    WATCH_POSE = Pose(
        position=Point(x=0.333, y=0.155, z=0.377),
        orientation=Quaternion(x=0.999, y=0.013, z=-0.031, w=0.003),
    )
    PLACE_POSE = Pose(
        position=Point(x=0.430, y=-0.219, z=0.275),
        orientation=Quaternion(x=0.997, y=-0.062, z=0.045, w=-0.002),
    )
    GRIPPER_OPEN  = 0.0
    GRIPPER_CLOSE = 0.85

    def __init__(self):
        super().__init__('grasp_policy')

        # ---- 抓取状态 ----
        self._grasp_pose_base = None   # Pose，link_base 下的抓取位姿
        self._grasp_width   = 0.0      # 抓取宽度（米）
        self._running       = False    # 抓取序列是否正在执行
        self._lock          = threading.Lock()

        # ---- TF2 ----
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- MoveIt2 动作客户端 ----
        self._move_client   = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

        # ---- 笛卡尔路径规划服务客户端 ----
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')

        # ---- 订阅 ----
        self.create_subscription(Float32,     '/grasp_width', self._cb_grasp_width, 10)
        self.create_subscription(PoseStamped, '/grasp_pose',  self._cb_grasp_pose,  10)


        # 移动到观察位姿
        self.get_logger().info('  移动到观察位姿...')
        self._move_arm_free(self.WATCH_POSE, speed=0.2)  # _move_arm_cartesian  , speed=0.2
        
        self.get_logger().info('='*55)
        self.get_logger().info('  GraspPolicy 节点初始化完成')
        self.get_logger().info('  等待 /grasp_pose 话题（来自 detect_grasp.py）...')
        self.get_logger().info('='*55)

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

    # # 旋转矩阵转换为四元数（使用 transformations 库）
    # @staticmethod
    # def quaternion_from_rotation(rot):
    #     """3×3 旋转矩阵 → 四元数 [x, y, z, w]"""
    #     M = np.eye(4)
    #     M[:3, :3] = rot
    #     return utool(M)
    
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
        # 绕 Y 轴旋转 90°，使夹爪 Z 轴朝向下方（符合 xArm 抓取姿态约定）
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
    #  机械臂 OMPL 自由规划移动
    # ============================================================
    def _move_arm_free(self, target_pose: Pose, speed: float = 0.2) -> bool:
        """用 OMPL 随机规划移动机械臂到目标位姿（适用于大范围移动）"""
        self._move_client.wait_for_server()

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'xarm7'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor     = speed
        req.max_acceleration_scaling_factor = speed

        constraint = Constraints()
        constraint.name = 'target'

        # 位置约束
        pc = PositionConstraint()
        pc.header.frame_id = 'link_base'
        pc.link_name       = 'link_tcp'
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.005]
        bv.primitives.append(sp)
        target_with_pose = Pose()
        target_with_pose.position    = target_pose.position
        target_with_pose.orientation = target_pose.orientation
        bv.primitive_poses.append(target_with_pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        # 姿态约束
        oc = OrientationConstraint()
        oc.header.frame_id = 'link_base'
        oc.link_name       = 'link_tcp'
        oc.orientation     = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0

        constraint.position_constraints.append(pc)
        constraint.orientation_constraints.append(oc)
        req.goal_constraints.append(constraint)

        goal.request = req
        goal.planning_options.plan_only      = True
        goal.planning_options.replan         = True
        goal.planning_options.replan_attempts = 3

        # 发送规划请求并等待结果
        plan_result = self._send_and_wait(
            self._move_client.send_goal_async(goal), 'OMPL 规划')
        if plan_result is None:
            return False
        if plan_result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'OMPL 规划失败: error_code={plan_result.result.error_code.val}')
            return False
        return self._execute_trajectory(plan_result.result.planned_trajectory)

    # ============================================================
    #  机械臂笛卡尔直线移动
    # ============================================================
    def _move_arm_cartesian(self, target_pose: Pose) -> bool:
        """用笛卡尔路径规划走直线（适用于近距离精确移动，防扭曲）"""
        self._cartesian_client.wait_for_service()

        req = GetCartesianPath.Request()
        req.header.frame_id  = 'link_base'
        req.group_name       = 'xarm7'
        req.link_name        = 'link_tcp'
        req.max_step         = 0.01    # 每 1cm 一个插值点
        req.jump_threshold   = 1.5     # 防扭曲阈值
        req.avoid_collisions = True
        req.waypoints.append(target_pose)

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        result = future.result()

        frac = result.fraction * 100 if result else 0
        self.get_logger().info(f'笛卡尔规划径覆盖率 {frac:.1f}%')

        # if result is None or result.fraction < 0.9:
        #     frac = result.fraction * 100 if result else 0
        #     self.get_logger().error(f'笛卡尔规划失败，路径覆盖率仅 {frac:.1f}%')
        #     return False

        return self._execute_trajectory(result.solution)

    # ============================================================
    #  夹爪控制（JointConstraint）
    # ============================================================
    def _move_gripper(self, position: float) -> bool:
        """控制 xArm 夹爪到指定开合位置（0.0=张开, 0.85=闭合）"""
        self._move_client.wait_for_server()

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name       = 'xarm_gripper'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor     = 0.3
        req.max_acceleration_scaling_factor = 0.3

        constraint = Constraints()
        jc = JointConstraint()
        jc.joint_name       = 'drive_joint'
        jc.position         = position
        jc.tolerance_above  = 0.05
        jc.tolerance_below  = 0.05
        jc.weight           = 1.0
        constraint.joint_constraints.append(jc)
        req.goal_constraints.append(constraint)

        goal.request = req
        goal.planning_options.plan_only      = True
        goal.planning_options.replan         = True
        goal.planning_options.replan_attempts = 3

        plan_result = self._send_and_wait(
            self._move_client.send_goal_async(goal), '夹爪规划')
        if plan_result is None:
            return False
        if plan_result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error('夹爪规划失败')
            return False
        return self._execute_trajectory(plan_result.result.planned_trajectory)

    # ============================================================
    #  底层辅助：发送动作并等待规划结果
    # ============================================================
    def _send_and_wait(self, send_future, label: str):
        """等待异步动作目标被接受，再等待规划结果返回"""
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'{label}请求被拒绝')
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        return result_future.result()

    def _execute_trajectory(self, trajectory) -> bool:
        """执行已规划好的轨迹并等待完成"""
        self._execute_client.wait_for_server()
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = trajectory

        send_future = self._execute_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        exec_handle = send_future.result()

        if exec_handle is None or not exec_handle.accepted:
            self.get_logger().error('轨迹执行请求被拒绝')
            return False

        result_future = exec_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        result = result_future.result().result

        ok = result.error_code.val == MoveItErrorCodes.SUCCESS
        if not ok:
            self.get_logger().error(f'轨迹执行失败: error_code={result.error_code.val}')
        return ok

    # ============================================================
    #  抓取序列（7 步）
    # ============================================================
    def _run_grasp_sequence(self):
        """
        执行完整抓取序列:
          1. 直线下降到预抓取点（grasp + 5cm）
          2. 直线下降到抓取点
          3. 闭合夹爪
          4. 直线抬起（grasp + 10cm）
          5. 移动到放置点
          6. 张开夹爪
          7. 回到观察位姿
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

            # Step 1: 预抓取（笛卡尔直线，从上方 5cm 处直线下落）
            self.get_logger().info('[1/7] 移动到预抓取点（OMPL路径）')
            if not self._move_arm_free(pre_grasp, speed=0.2 ):
                self._abort()
                return

            # Step 2: 到达抓取点（笛卡尔直线，直插到目标）
            self.get_logger().info('[2/7] 移动到抓取点（笛卡尔直线）')
            if not self._move_arm_cartesian(grasp):
                self._abort()
                return

            # Step 3: 闭合夹爪
            self.get_logger().info(f'[3/7] 闭合夹爪 (drive_joint={self.GRIPPER_CLOSE})')
            if not self._move_gripper(self.GRIPPER_CLOSE):
                self._abort()
                return

            # Step 4: 抬起（笛卡尔直线，向上 10cm）
            self.get_logger().info('[4/7] 抬起物体（笛卡尔直线）')
            if not self._move_arm_cartesian(lift_grasp):
                self._abort()
                return

            # Step 5: 移动到放置点（OMPL 自由规划，距离较大）
            self.get_logger().info('[5/7] 移动到放置点（OMPL）')
            if not self._move_arm_free(self.PLACE_POSE, speed=0.2):
                self._abort()
                return

            # Step 6: 张开夹爪，释放物体
            self.get_logger().info(f'[6/7] 张开夹爪 (drive_joint={self.GRIPPER_OPEN})')
            if not self._move_gripper(self.GRIPPER_OPEN):
                self._abort()
                return

            # Step 7: 回到观察位姿
            self.get_logger().info('[7/7] 回到观察位姿（OMPL）')
            if not self._move_arm_free(self.WATCH_POSE, speed=0.2):
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
        self.get_logger().error('抓取序列中止，等待下一次检测...')
        with self._lock:
            self._running = False


# ============================================================
#  入口
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = GraspPolicy()

    # 使用多线程执行器，支持 TF2 回调与抓取序列并发
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # 在后台线程运行 spin，主线程保持可中断
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        spin_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，节点退出。')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

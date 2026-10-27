
# 测试 通过moveit2 移动xArm7 到指定三维坐标

import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# 导入 MoveIt2 标准消息格式
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

class test_move(Node):
    def __init__(self):
        super().__init__('move_test_node')
        # 创建两个动作客户端：一个用于规划，一个用于执行
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def move_to_pose(self, px=0.4, py=0.0, pz=0.3, qx=1.0, qy=0.0, qz=0.0, qw=0.0): # 移动到指定位置和姿态
        """
        移动到指定位置和姿态:

        px, py, pz: 目标位置 (米)
        qx, qy, qz, qw: 目标姿态 (四元数)
        默认：px=0.4, py=0.0, pz=0.3, qx=1.0, qy=0.0, qz=0.0, qw=0.0
        """
        self.get_logger().info('等待 MoveIt2 动作服务器响应...')
        self._action_client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'xarm7'  # xArm7 的默认规划组名称
        req.num_planning_attempts = 10 # 规划尝试次数
        req.allowed_planning_time = 10.0 # 规划允许时间
        # 限制速度和加速度，仿真和真机测试时更安全
        req.max_velocity_scaling_factor = 0.2 # 最大速度缩放因子
        req.max_acceleration_scaling_factor = 0.2 # 最大加速度缩放因子

        # ---------------- 构建目标约束 ----------------
        constraint = Constraints()
        constraint.name = 'target_pose'

        # 1. 空间位置约束 (Position Constraint)
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'link_base' # 参考基座坐标系
        pos_constraint.link_name = 'link_tcp'        # 我们要控制的目标点是 TCP (夹爪中心)
        
        # 允许的误差范围 (创建一个极小的球体作为公差范围)
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]  # 允许 5mm 的位置规划误差
        bv.primitives.append(sphere)
        
        target_pose = Pose()
        target_pose.position = Point(x=px, y=py, z=pz)
        target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        bv.primitive_poses.append(target_pose)
        
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0 # 位置约束权重

        # 2. 空间姿态约束 (Orientation Constraint)
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'link_base'
        ori_constraint.link_name = 'link_tcp'
        ori_constraint.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        # 允许各轴的旋转角度误差 (弧度)
        ori_constraint.absolute_x_axis_tolerance = 0.05
        ori_constraint.absolute_y_axis_tolerance = 0.05
        ori_constraint.absolute_z_axis_tolerance = 0.05
        ori_constraint.weight = 1.0

        # 组合约束并装填到请求中
        constraint.position_constraints.append(pos_constraint)
        constraint.orientation_constraints.append(ori_constraint)
        req.goal_constraints.append(constraint)

        goal_msg.request = req
        
        # 第一阶段：仅规划，不执行（plan_only = True）
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.replan = True # 规划失败时自动重试
        goal_msg.planning_options.replan_attempts = 3 # 重试规划次数

        self.get_logger().info(f'目标位姿 -> 位置: X={px}, Y={py}, Z={pz}  姿态四元数: [{qx}, {qy}, {qz}, {qw}]')

        # 异步发送目标点，并绑定回调函数处理结果
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # -------- Phase 1 规划完成后的回调 --------
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('路径规划请求被 MoveIt2 拒绝')
            rclpy.shutdown()
            return

        self.get_logger().info('规划请求已接受，等待规划结果...')
        self._plan_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.plan_result_callback)

    # -------- 获取规划结果并请求用户确认 --------
    def plan_result_callback(self, future):
        result = future.result().result
        error_val = result.error_code.val

        if error_val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'路径规划失败，MoveIt2 错误代码: {error_val}')
            rclpy.shutdown()
            return

        # 规划成功，保存轨迹供后续执行
        trajectory = result.planned_trajectory
        n_points = len(trajectory.joint_trajectory.points)
        duration = trajectory.joint_trajectory.points[-1].time_from_start
        self._planned_trajectory = trajectory

        print('\n' + '=' * 50)
        print('  路径规划成功！请在 RViz 中查看规划路径。')
        print(f'  轨迹点数: {n_points}')
        print(f'  预计运动时长: {duration.sec}.{duration.nanosec:09d} 秒')
        print('=' * 50)

        # 请求用户确认
        while True:
            confirm = input('输入 y 执行该路径 / 输入 n 取消执行: ').strip().lower()
            if confirm in ('y', 'n'):
                break
            print('输入无效，请输入 y 或 n。')

        if confirm == 'n':
            self.get_logger().warn('用户已取消本次运动指令，节点退出。')
            rclpy.shutdown()
            return

        # Phase 2: 用户确认，执行已规划好的轨迹
        self.get_logger().info('用户已确认，正在执行规划路径...')
        self._execute_planned_trajectory()

    # -------- Phase 2: 通过 ExecuteTrajectory 执行轨迹 --------
    def _execute_planned_trajectory(self):
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = self._planned_trajectory

        self._execute_client.wait_for_server()
        self._exec_future = self._execute_client.send_goal_async(exec_goal)
        self._exec_future.add_done_callback(self._execute_response_callback)

    def _execute_response_callback(self, future):
        exec_handle = future.result()
        if not exec_handle.accepted:
            self.get_logger().error('轨迹执行请求被拒绝')
            rclpy.shutdown()
            return

        self.get_logger().info('轨迹执行已接受，运动中...')
        self._exec_result_future = exec_handle.get_result_async()
        self._exec_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('============ 运动顺利到达目标点！ ============')
        else:
            self.get_logger().error(f'运动中止，MoveIt2 错误代码: {result.error_code.val}')
        
        # 动作完成后安全关闭节点
        rclpy.shutdown()

def main(args=None):
    # 1. 配置参数解析器
    parser = argparse.ArgumentParser(description='测试 xArm7 移动到指定三维坐标')
    parser.add_argument('--px', type=float, default=0.4, help='目标 X 坐标 (米)')
    parser.add_argument('--py', type=float, default=0.0, help='目标 Y 坐标 (米)')
    parser.add_argument('--pz', type=float, default=0.3, help='目标 Z 坐标 (米)')
    
    # 2. 解析系统传入的参数 
    # parse_known_args 可以自动忽略 ROS 2 系统附加的底层参数（如 --ros-args）
    parsed_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    # 3. 初始化 ROS 2 节点
    rclpy.init(args=args)
    node = test_move()
    
    # 设定一个测试目标坐标 (基于 link_base)
    # 接收用户输入的目标坐标，用户不输入时，使用默认值
    # 默认 X=0.4m, Y=0.0m, Z=0.3m
    # 四元数 [1, 0, 0, 0] 通常代表机械臂末端绕 X 轴旋转 180 度，处于朝下抓取姿态
    # 4. 将终端解析到的参数传入运动函数
    node.move_to_pose(
        px=parsed_args.px, 
        py=parsed_args.py, 
        pz=parsed_args.pz, 
        qx=1.0, qy=0.0, qz=0.0, qw=0.0
        )
    rclpy.spin(node)

if __name__ == '__main__':
    main()

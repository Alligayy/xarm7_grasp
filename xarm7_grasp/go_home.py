"""
Go Home Node for XArm7
复位到初始观察姿态
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

class GoHomeNode(Node):
    def __init__(self):
        super().__init__('go_home')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def move_to_home(self):
        self.get_logger().info('等待 MoveIt2 动作服务器响应...')
        self._action_client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'xarm7'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        
        # 真机复位安全限速：限制为 10% 的最大性能
        req.max_velocity_scaling_factor = 0.1 
        req.max_acceleration_scaling_factor = 0.1 

        # ---------------- 构建目标约束 ----------------
        constraint = Constraints()
        constraint.name = 'home_pose'

        # 1. 空间位置约束 (Position Constraint)
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'link_base'    # 参考基座坐标系
        pos_constraint.link_name = 'link_tcp'           # 要控制的目标点是 TCP (夹爪中心)

        # 允许的误差范围 (创建一个极小的球体作为公差范围)
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]  # # 允许 5mm 的位置规划误差
        bv.primitives.append(sphere)
        
        # 注入提取到的 Home 姿态坐标
        target_pose = Pose()
        target_pose.position = Point(x=0.333, y=0.155, z=0.377)
        target_pose.orientation = Quaternion(x=0.999, y=0.013, z=-0.031, w=0.003)
        bv.primitive_poses.append(target_pose)
        
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0     # 位置约束权重 

        # 2. 空间姿态约束 (Orientation Constraint)
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'link_base'
        ori_constraint.link_name = 'link_tcp'
        ori_constraint.orientation = target_pose.orientation
        # 允许各轴的旋转角度误差 (弧度)
        ori_constraint.absolute_x_axis_tolerance = 0.005
        ori_constraint.absolute_y_axis_tolerance = 0.005
        ori_constraint.absolute_z_axis_tolerance = 0.005
        ori_constraint.weight = 1.0     # 姿态约束权重

        # 组合约束并装填到请求中
        constraint.position_constraints.append(pos_constraint)
        constraint.orientation_constraints.append(ori_constraint)
        req.goal_constraints.append(constraint)

        goal_msg.request = req
        goal_msg.planning_options.plan_only = True # 仅规划，不执行（plan_only = True）
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        self.get_logger().info('发起复位规划请求...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future): # 回调函数，处理规划请求的响应
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('路径规划被拒绝。')
            rclpy.shutdown()
            return

        self._plan_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.plan_result_callback)

    def plan_result_callback(self, future): # 回调函数，处理规划结果
        result = future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'复位路径规划失败，错误代码: {result.error_code.val}')
            rclpy.shutdown()
            return

        self._planned_trajectory = result.planned_trajectory
        print('\n' + '=' * 50)
        print('复位轨迹规划成功！请在 RViz2 中确认红色拖尾轨迹无碰撞。')
        print('\n' + '=' * 50)

        while True:
            confirm = input('输入 y 执行复位 / 输入 n 取消复位: ').strip().lower()
            if confirm in ('y', 'n'):
                break

        if confirm == 'n':
            self.get_logger().warn('已取消复位指令，节点退出。')
            rclpy.shutdown()
            return
        
        self.get_logger().info('用户已确认，正在执行规划路径...')
        self._execute_planned_trajectory()

    def _execute_planned_trajectory(self): # 执行规划路径
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = self._planned_trajectory

        self._execute_client.wait_for_server()
        self._exec_future = self._execute_client.send_goal_async(exec_goal)
        self._exec_future.add_done_callback(self._execute_response_callback)

    def _execute_response_callback(self, future): # 回调函数，处理执行响应
        exec_handle = future.result()
        if not exec_handle.accepted:
            self.get_logger().error('轨迹执行请求被拒绝')
            rclpy.shutdown()
            return

        self.get_logger().info('轨迹执行已接受，运动中...')
        self._exec_result_future = exec_handle.get_result_async()
        self._exec_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future): # 回调函数，处理执行结果
        result = future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('============ 已到达初始观察姿态！ ============')
        else:
            self.get_logger().error('执行异常中断。')

        rclpy.shutdown()# 复位完成后安全关闭节点

def main(args=None):
    rclpy.init(args=args)
    node = GoHomeNode()
    node.move_to_home()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
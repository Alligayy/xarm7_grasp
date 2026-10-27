# 测试夹爪开启和关闭
# 用法:
#   ros2 run xarm7_grasp test_gripper                 # 交互式选择 open/close
#   ros2 run xarm7_grasp test_gripper --action open   # 直接开启夹爪
#   ros2 run xarm7_grasp test_gripper --action close  # 直接关闭夹爪

import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.msg import MoveItErrorCodes

# xArm 官方夹爪关节位置
GRIPPER_OPEN  = 0.1    # 张开（0.0）
GRIPPER_CLOSE = 0.8   # 闭合（0.85）


class TestGripper(Node):
    def __init__(self):
        super().__init__('gripper_test_node')
        # 规划动作客户端（move_action 同时服务 xarm7 和 xarm_gripper 两个规划组）
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        # 轨迹执行动作客户端
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def set_gripper(self, position):
        """
        控制 xArm 官方夹爪到指定开合位置:

        position: 夹爪关节目标角度 (弧度)
                  0.0  = 完全张开
                  0.85 = 完全闭合
        """
        action_name = '张开' if position < 0.4 else '闭合'
        self.get_logger().info('等待 MoveIt2 动作服务器响应...')
        self._action_client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'xarm_gripper'  # xArm 夹爪的 MoveIt2 规划组名称
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        # ---------------- 构建关节约束 ----------------
        constraint = Constraints()
        constraint.name = 'gripper_target'

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'drive_joint'   # xArm 夹爪的驱动关节
        joint_constraint.position = position
        joint_constraint.tolerance_above = 0.05       # 允许 +0.05 rad 的正向误差
        joint_constraint.tolerance_below = 0.05       # 允许 -0.05 rad 的负向误差
        joint_constraint.weight = 1.0

        constraint.joint_constraints.append(joint_constraint)
        req.goal_constraints.append(constraint)

        goal_msg.request = req

        # 第一阶段：仅规划，不执行（plan_only = True）
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        self.get_logger().info(f'目标夹爪动作: {action_name}  (关节位置={position:.3f} rad)')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # -------- 规划请求被接受/拒绝的回调 --------
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('夹爪规划请求被 MoveIt2 拒绝')
            rclpy.shutdown()
            return

        self.get_logger().info('规划请求已接受，等待规划结果...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.plan_result_callback)

    # -------- 获取规划结果并请求用户确认 --------
    def plan_result_callback(self, future):
        result = future.result().result
        error_val = result.error_code.val

        if error_val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'夹爪路径规划失败，MoveIt2 错误代码: {error_val}')
            rclpy.shutdown()
            return

        # 规划成功，保存轨迹供执行
        trajectory = result.planned_trajectory
        n_points = len(trajectory.joint_trajectory.points)
        duration = trajectory.joint_trajectory.points[-1].time_from_start
        self._planned_trajectory = trajectory

        print('\n' + '=' * 50)
        print('  夹爪路径规划成功！请在 RViz 中查看。')
        print(f'  轨迹点数: {n_points}')
        print(f'  预计运动时长: {duration.sec}.{duration.nanosec:09d} 秒')
        print('=' * 50)

        # 请求用户确认
        while True:
            confirm = input('输入 y 执行该动作 / 输入 n 取消: ').strip().lower()
            if confirm in ('y', 'n'):
                break
            print('输入无效，请输入 y 或 n。')

        if confirm == 'n':
            self.get_logger().warn('用户已取消本次夹爪动作，节点退出。')
            rclpy.shutdown()
            return

        self.get_logger().info('用户已确认，正在执行夹爪动作...')
        self._execute_planned_trajectory()

    # -------- 通过 ExecuteTrajectory 执行轨迹 --------
    def _execute_planned_trajectory(self):
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = self._planned_trajectory

        self._execute_client.wait_for_server()
        self._exec_future = self._execute_client.send_goal_async(exec_goal)
        self._exec_future.add_done_callback(self._execute_response_callback)

    def _execute_response_callback(self, future):
        exec_handle = future.result()
        if not exec_handle.accepted:
            self.get_logger().error('夹爪轨迹执行请求被拒绝')
            rclpy.shutdown()
            return

        self.get_logger().info('轨迹执行中...')
        self._exec_result_future = exec_handle.get_result_async()
        self._exec_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('============ 夹爪动作执行完成！ ============')
        else:
            self.get_logger().error(f'夹爪运动中止，MoveIt2 错误代码: {result.error_code.val}')

        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description='测试 xArm7 官方夹爪开启/关闭')
    parser.add_argument('--action', type=str, choices=['open', 'close'],
                        help='夹爪动作：open（张开）或 close（闭合），不指定则交互式选择')

    parsed_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    node = TestGripper()

    # 确定夹爪动作
    if parsed_args.action == 'open':
        position = GRIPPER_OPEN
    elif parsed_args.action == 'close':
        position = GRIPPER_CLOSE
    else:
        # 交互式选择
        print('\n' + '=' * 50)
        print('  xArm7 夹爪测试工具')
        print('=' * 50)
        print('  1. 张开夹爪 (drive_joint = 0.0)')
        print('  2. 闭合夹爪 (drive_joint = 0.85)')
        print('=' * 50)
        while True:
            choice = input('请输入选项 (1 或 2): ').strip()
            if choice == '1':
                position = GRIPPER_OPEN
                break
            elif choice == '2':
                position = GRIPPER_CLOSE
                break
            print('输入无效，请输入 1 或 2。')

    node.set_gripper(position)
    rclpy.spin(node)


if __name__ == '__main__':
    main()

# 测试夹爪开启和关闭
# 通过xarm的指令 移动xArm7 到指定三维坐标
#
# 使用 xarm_ros2 官方 xarm_planner 包提供的服务接口控制机械臂

import sys
import argparse
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanJoint, PlanExec


class test_gripper_xarm(Node):
    def __init__(self):
        super().__init__('test_gripper_xarm')
        # 创建 xarm_gripper_planner_node 提供的服务客户端
        # 注意：夹爪由独立的 xarm_gripper_planner_node 节点提供服务，
        # 服务名为 xarm_gripper_joint_plan / xarm_gripper_exec_plan，
        # 而不是机械臂主节点的 xarm_joint_plan / xarm_exec_plan
        self.plan_joint_client = self.create_client(PlanJoint, 'xarm_gripper_joint_plan')
        self.plan_exec_client = self.create_client(PlanExec, 'xarm_gripper_exec_plan')
        # xArm 官方夹爪关节位置

    # -------- 通用：等待服务并同步调用 --------
    def _call_service(self, client, request, timeout=60.0):
        """
        等待服务就绪并同步调用，返回响应的 success 字段。
        """
        service_name = client.srv_name
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('节点被中断，等待服务时退出。')
                return False
            self.get_logger().warn(f'服务 {service_name} 不可用，继续等待 ...')

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error(f'调用服务 {service_name} 失败')
            return False
        success = future.result().success
        self.get_logger().info(f'调用服务 {service_name}, success={success}')
        return success
    
    def plan_gripper(self, position, action_name):
        """
        计划夹爪张开到指定位置。
        """
        req = PlanJoint.Request()
        # 必须传 6 个值！xarm_gripper 规划组共有 6 个关节变量：
        # 1 个有效关节 drive_joint + 5 个 mimic 随动关节（左右指节/手指）。
        # MoveGroupInterface::setJointValueTarget(vector) 要求传入值个数 == 组全部变量数，
        # 只传 1 个值会静默失败（目标不被设置，轨迹终点永远是默认值 0，夹爪不动）。
        # 5 个 mimic 关节与 drive_joint 同步（factor=1），所以 6 个值相同。
        # 参考官方测试：xarm_planner/test/test_xarm_gripper_planner_api_joint.cpp
        req.target = [position] * 6
        self.get_logger().info(f'目标夹爪位置 -> {position}（{action_name}）')
        return self._call_service(self.plan_joint_client, req)


    # -------- 执行上一步规划好的轨迹 --------
    def execute_plan(self, wait=True):
        """
        执行最近一次规划成功的轨迹，wait=True 表示阻塞直到运动完成。
        """
        req = PlanExec.Request()
        req.wait = wait # 是否阻塞直到运动完成
        return self._call_service(self.plan_exec_client, req, timeout=120.0)

    def move_gripper(self, position):
        """
        计划并执行夹爪的开启或闭合动作。
        action (str): 'open' 或 'close'，表示夹爪的开启或闭合动作。
        """
        action_name = '张开' if position < 0.4 else '闭合'
        self.get_logger().info('等待 xarm_planner_node 服务响应...')
        
        if not self.plan_gripper(position, action_name):
            self.get_logger().error(f'计划夹爪 {action_name} 失败')
            return False
        
        self.get_logger().info(f'正在执行夹爪 {action_name} 动作...') # 执行规划好的轨迹
        # 执行规划好的轨迹
        if not self.execute_plan():
            self.get_logger().error('执行规划好的轨迹失败')
            return False


def main(args=None):

    GRIPPER_OPEN  = 0.1    # 张开（0.0）
    GRIPPER_CLOSE = 0.8   # 闭合（0.85）
    parser = argparse.ArgumentParser(description='测试 xArm7 官方夹爪开启/关闭')
    parser.add_argument('--action', type=str, choices=['open', 'close'],
                        help='夹爪动作：open（张开）或 close（闭合），不指定则交互式选择')

    parsed_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    node = test_gripper_xarm()

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

    node.move_gripper(position)
    node.destroy_node()
    rclpy.shutdown()# 关闭节点并释放资源

if __name__ == '__main__':
    main()
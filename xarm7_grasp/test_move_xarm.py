
# 测试 通过xarm的指令 移动xArm7 到指定三维坐标
#
# 使用 xarm_ros2 官方 xarm_planner 包提供的服务接口控制机械臂，
# 相比 test_move.py 手动构造 MoveIt2 MoveGroup 动作请求，这里只需调用两个服务即可完成运动：
#   1. xarm_pose_plan   (xarm_msgs/srv/PlanPose)  : 规划末端位姿目标
#   2. xarm_exec_plan   (xarm_msgs/srv/PlanExec)  : 执行上一步规划好的轨迹
# 另外还支持：
#   3. xarm_joint_plan     (xarm_msgs/srv/PlanJoint)          : 规划关节角度目标
#   4. xarm_straight_plan  (xarm_msgs/srv/PlanSingleStraight) : 规划笛卡尔直线运动
#
# 前置条件：需要先启动 xarm_planner_node（已在仿真/真机 launch 中自动包含），
# 即运行着 MoveIt2 move_group 的同时，还要有 xarm_planner_node 节点提供服务。

import sys
import argparse
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanPose, PlanJoint, PlanExec, PlanSingleStraight


class test_move_xarm(Node):
    def __init__(self):
        super().__init__('move_test_xarm_node')
        # 创建 xarm_planner_node 提供的服务客户端
        self.pose_plan_client = self.create_client(PlanPose, 'xarm_pose_plan')
        self.joint_plan_client = self.create_client(PlanJoint, 'xarm_joint_plan')
        self.straight_plan_client = self.create_client(PlanSingleStraight, 'xarm_straight_plan')
        self.exec_plan_client = self.create_client(PlanExec, 'xarm_exec_plan')

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

    # -------- 位姿规划（规划组默认末端为 link_tcp，参考坐标系 link_base） --------
    def plan_pose(self, px, py, pz, qx, qy, qz, qw):
        """
        规划到指定末端位姿。
        px, py, pz: 目标位置 (米)
        qx, qy, qz, qw: 目标姿态 (四元数)
        """
        req = PlanPose.Request()
        req.target = Pose()
        req.target.position.x = px
        req.target.position.y = py
        req.target.position.z = pz
        req.target.orientation.x = qx
        req.target.orientation.y = qy
        req.target.orientation.z = qz
        req.target.orientation.w = qw
        self.get_logger().info(
            f'目标位姿 -> 位置: X={px}, Y={py}, Z={pz}  姿态四元数: [{qx}, {qy}, {qz}, {qw}]')
        return self._call_service(self.pose_plan_client, req)

    # -------- 关节角度规划 --------
    def plan_joint(self, joint_target):
        """
        规划到指定关节角度，joint_target 为 7 个关节角度列表 (弧度)。
        """
        req = PlanJoint.Request()
        req.target = joint_target
        self.get_logger().info(f'目标关节角度 -> {joint_target}')
        return self._call_service(self.joint_plan_client, req)

    # -------- 笛卡尔直线运动规划 --------
    def plan_straight(self, px, py, pz, qx, qy, qz, qw):
        """
        规划到指定位姿的笛卡尔直线路径（末端沿直线移动，适合抓取接近动作）。
        """
        req = PlanSingleStraight.Request()
        req.target = Pose()
        req.target.position.x = px
        req.target.position.y = py
        req.target.position.z = pz
        req.target.orientation.x = qx
        req.target.orientation.y = qy
        req.target.orientation.z = qz
        req.target.orientation.w = qw
        self.get_logger().info(
            f'直线运动目标 -> 位置: X={px}, Y={py}, Z={pz}  姿态四元数: [{qx}, {qy}, {qz}, {qw}]')
        return self._call_service(self.straight_plan_client, req)

    # -------- 执行上一步规划好的轨迹 --------
    def execute_plan(self, wait=True):
        """
        执行最近一次规划成功的轨迹，wait=True 表示阻塞直到运动完成。
        """
        req = PlanExec.Request()
        req.wait = wait # 是否阻塞直到运动完成
        return self._call_service(self.exec_plan_client, req, timeout=120.0)

    # -------- 一步到位：规划 + 确认后执行 --------
    def move_to_pose(self, px=0.4, py=0.0, pz=0.3, qx=1.0, qy=0.0, qz=0.0, qw=0.0):
        """
        移动到指定位置和姿态:

        px, py, pz: 目标位置 (米)
        qx, qy, qz, qw: 目标姿态 (四元数)
        默认：px=0.4, py=0.0, pz=0.3, qx=1.0, qy=0.0, qz=0.0, qw=0.0
        """
        self.get_logger().info('等待 xarm_planner_node 服务响应...')

        # Phase 1: 规划
        if not self.plan_straight(px, py, pz, qx, qy, qz, qw):
            self.get_logger().error('路径规划失败，请检查目标点是否在可达范围内。')
            return False

        print('\n' + '=' * 50)
        print('  路径规划成功！请在 RViz 中查看规划路径。')
        print('=' * 50)

        # 请求用户确认
        while True:
            confirm = input('输入 y 执行该路径 / 输入 n 取消执行: ').strip().lower()
            if confirm in ('y', 'n'):
                break
            print('输入无效，请输入 y 或 n。')

        if confirm == 'n':
            self.get_logger().warn('用户已取消本次运动指令。')
            return False

        # Phase 2: 用户确认，执行已规划好的轨迹
        self.get_logger().info('用户已确认，正在执行规划路径...')
        if self.execute_plan(wait=True):
            self.get_logger().info('============ 运动顺利到达目标点！ ============')
            return True
        self.get_logger().error('轨迹执行失败！')
        return False


def main(args=None):
    # 1. 配置参数解析器
    parser = argparse.ArgumentParser(description='测试 xArm7 移动到指定三维坐标 (xarm_planner 服务版)')
    parser.add_argument('--px', type=float, default=0.4, help='目标 X 坐标 (米)')
    parser.add_argument('--py', type=float, default=0.0, help='目标 Y 坐标 (米)')
    parser.add_argument('--pz', type=float, default=0.3, help='目标 Z 坐标 (米)')

    # 2. 解析系统传入的参数
    # parse_known_args 可以自动忽略 ROS 2 系统附加的底层参数（如 --ros-args）
    parsed_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    # 3. 初始化 ROS 2 节点
    rclpy.init(args=args)
    node = test_move_xarm()

    try:
        # 设定一个测试目标坐标 (基于 link_base)
        # 接收用户输入的目标坐标，用户不输入时，使用默认值
        # 默认 X=0.4m, Y=0.0m, Z=0.3m
        # 四元数 [1, 0, 0, 0] 通常代表机械臂末端绕 X 轴旋转 180 度，处于朝下抓取姿态
        node.move_to_pose(
            px=parsed_args.px,
            py=parsed_args.py,
            pz=parsed_args.pz,
            qx=1.0, qy=0.0, qz=0.0, qw=0.0
            )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

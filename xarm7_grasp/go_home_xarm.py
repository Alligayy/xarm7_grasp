"""
Go Home Node for XArm7
复位到初始观察姿态
使用xarm_ros2的xarm_planner进行路径规划
"""


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


from geometry_msgs.msg import Pose, Point, Quaternion
from xarm_msgs.srv import PlanPose, PlanJoint, PlanExec, PlanSingleStraight


class go_home_xarm(Node):
    def __init__(self):
        super().__init__('go_home_xarm_node')
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
        默认关节角度为 [15.0, -13.0, 12.0, 78.0, 3.0, 91.0, 27.0]
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

    # -------- 执行规划路径 --------
    def execute_plan(self, wait=True):
        """
        执行规划的路径。
        """
        req = PlanExec.Request()
        # 这里设置为 True 表示等待运动完成
        req.wait = wait
        self.get_logger().info('执行规划路径')
        return self._call_service(self.exec_plan_client, req)

    # -------- 一步到位：规划 + 确认后执行 --------
    def move_to_home(self, px=0.333, py=0.155, pz=0.377, qx=1.0, qy=0.0, qz=0.0, qw=0.0):
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

    # -------- ：规划 + 确认后执行 --------
    def move_to_joint(self, px=0.333, py=0.155, pz=0.377, qx=1.0, qy=0.0, qz=0.0, qw=0.0):
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
    rclpy.init(args=args)
    node = go_home_xarm()
    # 复位到初始观察姿态
    node.move_to_home(px=0.35, py=0.2, pz=0.38, qx=1.0, qy=0.0, qz=0.0, qw=0.0)
    # 复位到放置物体姿态
    #node.move_to_home(px=0.450, py=-0.25, pz=0.25, qx=1.0, qy=0.0, qz=0.0, qw=0.0)
    rclpy.shutdown()# 复位完成后安全关闭节点


if __name__ == '__main__':
    main()

# test_move_cartesian.py
# 测试机械臂笛卡尔运动,调用笛卡尔路径规划服务
#防止机械臂运动时关节扭曲
"""
由于开启了防扭曲约束并强制走直线，
如果目标点与当前点的连线中途遇到了关节极限或者奇点，
fraction (覆盖率) 可能会达不到 100% 从而报错。
在实际抓取任务中，策略通常是：

1,大范围移动：先用最初那版不带直线的代码（OMPL 随机规划），移动到物体上方 10cm 的“预抓取点”。

2,末端直线靠近：使用当前这版带 jump_threshold 和 max_step 的代码，
    垂直向下直插 10cm 到达“抓取点”，确保关节不会在靠近物体的最后阶段突然翻转打翻物品。
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# 导入 MoveIt2 动作和轨迹执行相关消息
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose, Point, Quaternion

class test_move_cartesian(Node):
    def __init__(self):
        super().__init__('move_test_node')
        
        # 创建笛卡尔路径规划服务客户端 (负责计算直线)
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        # 创建轨迹执行动作客户端 (负责让机械臂动起来)
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def move_to_pose_cartesian(self, px=0.4, py=0.0, pz=0.3, qx=1.0, qy=0.0, qz=0.0, qw=0.0):
        """
        使用笛卡尔空间走直线，并开启防扭曲约束
        """
        self.get_logger().info('等待 MoveIt2 笛卡尔规划服务...')
        self._cartesian_client.wait_for_service()

        req = GetCartesianPath.Request()
        req.header.frame_id = 'link_base'
        req.group_name = 'xarm7'
        req.link_name = 'link_tcp'
        # 限制速度和加速度，仿真和真机测试时更安全
        
        # ================= 核心防扭曲与直线参数 =================
        # 1. max_step: 直线插补步长 (米)。设为 0.01 表示每隔 1 厘米生成一个密集路点
        req.max_step = 0.01
        
        # 2. jump_threshold: 防扭曲阈值。限制关节空间跳变。
        # 设置为大于 0 的数即可开启 (1.5 ~ 2.0 是常用安全值)。
        # 如果系统发现关节需要“大跳”才能到达下一个点，它会立刻判定规划失败，从而防止机械臂乱扭。
        req.jump_threshold = 1.5 
        
        req.avoid_collisions = True
        # ========================================================

        # 构建目标点 (Waypoints)。
        # 笛卡尔规划会自动从当前机械臂的实际位置，连一条直线到这里设置的 Waypoints
        target_pose = Pose()
        target_pose.position = Point(x=px, y=py, z=pz)
        target_pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        req.waypoints.append(target_pose)

        self.get_logger().info(f'目标位姿 -> 位置: X={px}, Y={py}, Z={pz}  姿态: [{qx}, {qy}, {qz}, {qw}]')
        
        # 异步调用服务
        self._cartesian_future = self._cartesian_client.call_async(req)
        self._cartesian_future.add_done_callback(self.cartesian_response_callback)

    # -------- Phase 1: 笛卡尔规划完成后的回调 --------
    def cartesian_response_callback(self, future):
        result = future.result()
        fraction = result.fraction  # 路径覆盖率 (0.0 ~ 1.0)

        # fraction 小于 1.0 意味着无法走完整的直线 (可能遇到了运动学死角或碰撞)
        if fraction < 0.9: 
            self.get_logger().error(f'直线规划失败！仅能完成 {fraction*100:.2f}% 的路径。目标点可能不可达或受奇异点限制。')
            rclpy.shutdown()
            return

        self.get_logger().info(f'直线规划成功！路径覆盖率: {fraction*100:.2f}%')
        self._planned_trajectory = result.solution

        n_points = len(self._planned_trajectory.joint_trajectory.points)
        if n_points > 0:
            duration = self._planned_trajectory.joint_trajectory.points[-1].time_from_start
            print('\n' + '=' * 50)
            print('  笛卡尔直线路径规划成功！(防扭曲已开启)')
            print(f'  轨迹路点数: {n_points} (点数越多说明切分越细腻)')
            print(f'  预计运动时长: {duration.sec}.{duration.nanosec:09d} 秒')
            print('=' * 50)

        # 请求用户确认
        while True:
            confirm = input('输入 y 执行该直线路径 / 输入 n 取消执行: ').strip().lower()
            if confirm in ('y', 'n'):
                break
            print('输入无效，请输入 y 或 n。')

        if confirm == 'n':
            self.get_logger().warn('用户已取消本次运动指令，节点退出。')
            rclpy.shutdown()
            return

        # Phase 2: 用户确认，执行规划好的直线轨迹
        self.get_logger().info('用户已确认，正在执行轨迹...')
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
            self.get_logger().info('============ 直线运动顺利到达目标点！ ============')
        else:
            self.get_logger().error(f'运动中止，MoveIt2 错误代码: {result.error_code.val}')
        
        # 动作完成后安全关闭节点
        rclpy.shutdown()

def main(args=None):
    parser = argparse.ArgumentParser(description='测试 xArm7 移动到指定三维坐标(直线靠近)')
    parser.add_argument('--px', type=float, default=0.4, help='目标 X 坐标 (米)')
    parser.add_argument('--py', type=float, default=0.0, help='目标 Y 坐标 (米)')
    parser.add_argument('--pz', type=float, default=0.3, help='目标 Z 坐标 (米)')
    
    parsed_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    node = test_move_cartesian()
    
    # 调用新的笛卡尔直线规划函数
    node.move_to_pose_cartesian(
        px=parsed_args.px, 
        py=parsed_args.py, 
        pz=parsed_args.pz, 
        qx=1.0, qy=0.0, qz=0.0, qw=0.0
    )
    rclpy.spin(node)

if __name__ == '__main__':
    main()
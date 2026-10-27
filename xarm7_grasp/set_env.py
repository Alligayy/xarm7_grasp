#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA

class CollisionEnvSetup(Node):
    """
    这个节点用于向 MoveIt 添加虚拟桌面和墙体，以实现全方位防撞。
    """
    def __init__(self):
        super().__init__('set_env')
        # 创建发布者，向 MoveIt 的 collision_object 话题发布障碍物
        # self.publisher_ = self.create_publisher(CollisionObject, '/collision_object', 10)
        # 由于CollisionObject消息没有颜色属性，因此发布话题从 /collision_object 改为 /planning_scene
        # 创建 PlanningScene 发布者，用于设置障碍物颜色
        self.publisher_ = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # =========================================================
        # 【修复时序隐患】原先固定延时 2 秒后发布，若此时 move_group 的
        # 场景监视器还没启动完成，这条一次性消息会永久丢失（无任何报错，
        # 表现为“墙体偶尔不生效”）。
        # 改为主动探测就绪条件后再发布：
        #   1. move_group 的 /get_planning_scene 服务可用（说明 move_group 已起来）
        #   2. /planning_scene 话题至少有 1 个订阅者（说明场景监视器在监听）
        # 每 0.5 秒轮询一次，超时 30 秒后放弃等待强制发布并给出警告。
        # =========================================================
        self.has_published = False
        self.start_time = self.get_clock().now()
        self.WAIT_TIMEOUT_SEC = 30.0  # 最长等待 move_group 就绪的时间
        self.timer = self.create_timer(0.5, self.try_publish)

    def try_publish(self):
        """轮询检查 move_group 是否就绪，就绪后才真正发布障碍物"""
        if self.has_published:
            return

        # 超时保护：即使没等到就绪也强制发布一次（保留旧行为，至少试一次）
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        timed_out = elapsed > self.WAIT_TIMEOUT_SEC

        # 就绪条件：/get_planning_scene 服务存在 且 话题有订阅者
        # get_service_names_and_types() 返回当前图上全部服务（含 move_group 提供的）
        service_names = [name for name, _ in self.get_service_names_and_types()]
        scene_service_ready = '/get_planning_scene' in service_names
        has_subscriber = self.publisher_.get_subscription_count() > 0

        if not scene_service_ready or not has_subscriber:
            if timed_out:
                self.get_logger().warn(
                    '等待 move_group 超时(%.0fs)，强制发布障碍物（可能不会被接收！）' % self.WAIT_TIMEOUT_SEC)
            else:
                self.get_logger().info(
                    '等待 move_group 就绪（%.0fs/%.0fs）...' % (elapsed, self.WAIT_TIMEOUT_SEC),
                    throttle_duration_sec=5.0)
                return  # 未就绪，下个周期继续轮询
        else:
            self.get_logger().info('move_group 已就绪（服务可用且有订阅者），开始发布障碍物')

        # 已就绪（或等待超时），调用一次性发布方法
        self.publish_objects()

    def publish_objects(self):
        """一次性发布全部障碍物到规划场景（由 try_publish 确认就绪后调用）"""
        if self.has_published:
            return

        # =========================================================
        # 在这里微调墙体的尺寸和位置参数 (单位: 米)
        # =========================================================
        
        # 1. 基础尺寸参数
        thickness = 0.02        # 墙体的厚度 (2cm)
        box_x_length = 1.1      # 工作空间的 X 轴总跨度 (前后方向)
        box_y_length = 0.6      # 工作空间的 Y 轴总跨度 (左右方向)
        box_z_height = 1.0      # 墙体的 Z 轴总高度
        
        # 2. 各个面在 base_link 坐标系下的中心点位置
        # (默认值已完全对齐你原来 ROS 1 C++ 代码中的坐标)
        bottom_pos = {'x': 0.25,  'y': 0.30, 'z': -0.01} # 底部桌面中心
        back_pos   = {'x': 0.25,  'y': 0.59, 'z': 0.50}  # 背面墙体中心
        left_pos   = {'x': -0.29, 'y': 0.30, 'z': 0.50}  # 左侧墙体中心
        right_pos  = {'x': 0.79,  'y': 0.30, 'z': 0.50}  # 右侧墙体中心

        # =========================================================
        # 创建一个列表用于统一发布
        objects_to_publish = []

        # 工具函数：快速生成一个碰撞面
        def create_wall(id_name, dimensions, position):
            obj = CollisionObject()
            obj.header.frame_id = 'link_base'   # 坐标系基于机械臂底座
            obj.id = id_name                    # 障碍物 ID
            obj.operation = CollisionObject.ADD

            shape = SolidPrimitive()
            shape.type = SolidPrimitive.BOX
            shape.dimensions = dimensions

            pose = Pose()
            pose.position.x = position['x']
            pose.position.y = position['y']
            pose.position.z = position['z']
            pose.orientation.w = 1.0

            obj.primitives.append(shape)
            obj.primitive_poses.append(pose)
            return obj

        # ==================== 添加虚拟桌面和墙体 ====================
        # 1. 生成桌面 (X跨度, Y跨度, 厚度)
        objects_to_publish.append(
            create_wall('workspace_bottom', [box_x_length, box_y_length, thickness], bottom_pos)
        )
        # 2. 生成背面 (X跨度, 厚度, 高度)
        objects_to_publish.append(
            create_wall('workspace_back', [box_x_length, thickness, box_z_height], back_pos)
        )
        # 3. 生成左面 (厚度, Y跨度, 高度)
        objects_to_publish.append(
            create_wall('workspace_left', [thickness, box_y_length, box_z_height], left_pos)
        )
        # 4. 生成右面 (厚度, Y跨度, 高度)
        objects_to_publish.append(
            create_wall('workspace_right', [thickness, box_y_length, box_z_height], right_pos)
        )

        # 发布所有墙体
        # 通过 PlanningScene 设置障碍物颜色为灰色
        scene_msg = PlanningScene()
        scene_msg.is_diff = True  # 增量更新，不影响场景中已有的其他内容
        gray_color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # 灰色，透明度 0.8
        for obj in objects_to_publish:
            # 1. 把物体加到场景世界的碰撞体列表中
            scene_msg.world.collision_objects.append(obj)
            # 2. 为该物体的 ID 绑定刚才设置的高级灰色
            obj_color = ObjectColor()
            obj_color.id = obj.id
            obj_color.color = gray_color
            scene_msg.object_colors.append(obj_color)

        # 只需要发布这一个 Scene 消息即可
        self.publisher_.publish(scene_msg)

        self.get_logger().info('成功向 MoveIt 注入虚拟安全边界！')

        self.has_published = True
        self.timer.cancel() # 发布一次即可，关闭定时器

        # 短暂延时确保消息已送达 MoveIt，然后自动关闭节点
        time.sleep(0.5)
        raise SystemExit  # 触发 main() 中的退出流程

def main(args=None):
    rclpy.init(args=args)
    node = CollisionEnvSetup()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info('障碍物发布完毕，节点自动退出。')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
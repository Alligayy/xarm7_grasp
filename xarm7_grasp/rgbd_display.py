# -*- coding: utf-8 -*-
"""
RGB-D 同步接收节点,只是用来进行展示
OpenCV 弹窗会带来计算成本，且在 ROS 的多线程中容易卡死
推荐使用rqt_image_view进行展示，已在xarm7_grasp_all.launch.xml设置
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import message_filters

class RGBDSyncSubscriber(Node):
    def __init__(self):
        super().__init__('rgbd_display')
        self.bridge = CvBridge()

        # 1. 创建两个订阅器，但不直接绑定回调函数
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        # 2. 使用 ApproximateTimeSynchronizer 进行时间戳近似同步
        # queue_size=10: 缓存队列大小
        # slop=0.05: 允许的最大时间误差(秒)，50ms
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.05)
        
        # 3. 绑定同步后的回调函数
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info("RGB-D 同步接收节点已启动，等待图像数据...")

    def sync_callback(self, color_msg, depth_msg):
        # 此函数触发时，说明成功匹配到了一对时间戳相近的 RGB 和深度图
        try:
            # 将 ROS 的 Image 消息转换为 OpenCV 的 numpy 矩阵
            # 彩色图通常是 bgr8 格式
            cv_color_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            # 深度图通常是 16UC1 (16位无符号单通道，单位为毫米)
            cv_depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

            # === 在这里添加你的抓取检测算法 ===
            # 例如：获取图像中心点 (y=240, x=320) 的深度值 (假设分辨率为 640x480)
            center_depth = cv_depth_img[240, 320] 
            
            # 打印中心点深度值
            self.get_logger().info(f"图像中心点距离相机: {center_depth} 毫米")

            # 实时显示画面 (按 'q' 键退出可视化窗口)
            cv2.imshow("RGB Image", cv_color_img)
            # 深度图直接显示会是一片黑（因为数值太大），这里做归一化处理仅为了肉眼观看
            depth_colormap = cv2.normalize(cv_depth_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)
            cv2.imshow("Depth Image", depth_colormap)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RGBDSyncSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
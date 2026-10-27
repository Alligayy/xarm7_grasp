# -*- coding: utf-8 -*-
"""
ChArUco 标定板检测节点 (眼在手 Eye-in-Hand 手眼标定专用)

功能:
    订阅 RealSense D455 彩色图像与相机内参, 检测 ChArUco 标定板,
    使用 solvePnP 解算标定板相对于相机光心坐标系的 6D 位姿,
    并发布 TF: camera_color_optical_frame -> camera_marker

    easy_handeye2 只依赖该 TF, 不关心位姿由谁提供, 因此本节点可直接
    替换原来的 aruco_ros single 节点, easy_handeye2 部分无需任何修改。

特点:
    - 自动兼容 OpenCV 新旧 ArUco API (4.5 与 4.7+ 接口不同)。
    - 相机内参从 camera_info 话题读取 (使用相机出厂内参)。
    - 发布叠加了角点/坐标轴的调试图像, 可用 rqt_image_view 观察。

ChArUco 相比单个 ArUco 或普通棋盘格的优势:
    - 拥有棋盘格级别的大量亚像素角点 (PnP 更鲁棒、精度更高);
    - 每个白格内嵌 ArUco 码, 具备唯一 ID、抗遮挡、尺度确定。
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster


class CharucoDetectorNode(Node):
    def __init__(self):
        super().__init__('charuco_detector')

        # ==================== 可配置参数 (可在 launch 中覆盖) ====================
        # 标定板几何参数: 打印后必须与实际物理尺寸一致 (单位: 米)
        self.declare_parameter('squares_x', 8)          # X 方向方格数 (列)
        self.declare_parameter('squares_y', 6)          # Y 方向方格数 (行)
        self.declare_parameter('square_length', 0.030)  # 单个方格边长 30mm
        self.declare_parameter('marker_length', 0.022)  # 单个 ArUco 码边长 22mm
        self.declare_parameter('dictionary', 'DICT_5X5_100')  # ArUco 字典
        # 至少检测到多少个 ChArUco 角点才接受该帧的位姿解算
        self.declare_parameter('min_corners', 6)

        # 话题与坐标系
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('base_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_frame', 'camera_marker')
        self.declare_parameter('result_image_topic', '/charuco/result_image')
        # 是否弹出 OpenCV 窗口 (在 ROS 多线程中易卡死, 默认关闭, 推荐用 rqt_image_view)
        self.declare_parameter('show_window', False)

        squares_x = int(self.get_parameter('squares_x').value)
        squares_y = int(self.get_parameter('squares_y').value)
        square_length = float(self.get_parameter('square_length').value)
        marker_length = float(self.get_parameter('marker_length').value)
        dict_name = str(self.get_parameter('dictionary').value)
        self.min_corners = int(self.get_parameter('min_corners').value)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.marker_frame = str(self.get_parameter('marker_frame').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_info = None  # 缓存最新的相机内参

        # ==================== 构建 ChArUco 标定板 ====================
        if not hasattr(cv2.aruco, dict_name):
            raise ValueError(f'无效的 ArUco 字典名称: {dict_name}')
        dict_id = getattr(cv2.aruco, dict_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)

        # 4.7+ 引入 CharucoDetector, 以此判别新旧 API
        self.use_new_api = hasattr(cv2.aruco, 'CharucoDetector')

        if self.use_new_api:
            # 新 API (OpenCV >= 4.7)
            self.board = cv2.aruco.CharucoBoard(
                (squares_x, squares_y), square_length, marker_length, self.dictionary)
            self.charuco_detector = cv2.aruco.CharucoDetector(self.board)
        else:
            # 旧 API (OpenCV <= 4.6, Ubuntu 22.04 系统自带 4.5.4)
            self.board = cv2.aruco.CharucoBoard_create(
                squares_x, squares_y, square_length, marker_length, self.dictionary)

        # ==================== 订阅与发布 ====================
        # 传感器数据用 SensorData 风格 QoS (BEST_EFFORT), 与 RealSense 发布端匹配
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        image_topic = str(self.get_parameter('image_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        result_image_topic = str(self.get_parameter('result_image_topic').value)

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, sensor_qos)
        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, sensor_qos)

        self.result_pub = self.create_publisher(Image, result_image_topic, 10)

        self.get_logger().info(
            f'ChArUco 检测节点已启动 | OpenCV {"新" if self.use_new_api else "旧"} API | '
            f'板={squares_x}x{squares_y}, 方格={square_length*1000:.1f}mm, '
            f'码={marker_length*1000:.1f}mm, 字典={dict_name}')
        self.get_logger().info(
            f'订阅图像: {image_topic} | 发布 TF: {self.base_frame} -> {self.marker_frame}')

    def camera_info_callback(self, msg: CameraInfo):
        # 缓存内参, 供图像回调解算 PnP 使用
        self.camera_info = msg

    def image_callback(self, msg: Image):
        if self.camera_info is None:
            self.get_logger().warn('尚未收到 camera_info, 等待相机内参...', throttle_duration_sec=2.0)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ==================== 检测 ChArUco 角点 ====================
        charuco_corners, charuco_ids = self._detect_charuco(gray)

        # 相机内参矩阵与畸变系数 (D455 彩色图通常已rectify, D 多为 0)
        cam_matrix = np.array(self.camera_info.k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.array(self.camera_info.d, dtype=np.float64)
        if dist_coeffs.size == 0:
            dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        if (charuco_corners is not None and charuco_ids is not None
                and len(charuco_corners) >= self.min_corners):

            # ==================== solvePnP 解算标定板位姿 ====================
            valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, self.board,
                cam_matrix, dist_coeffs, None, None)

            if valid:
                self._publish_tf(rvec, tvec, msg.header.stamp)

                # 叠加坐标轴用于可视化验证
                axis_len = float(self.get_parameter('square_length').value) * 2.0
                cv2.drawFrameAxes(frame, cam_matrix, dist_coeffs, rvec, tvec, axis_len)

        # 叠加检测到的 ArUco 码与 ChArUco 角点
        self._draw_debug(frame, gray, charuco_corners, charuco_ids, cam_matrix, dist_coeffs)

        # 发布调试图像 (可用 rqt_image_view 订阅查看)
        out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out_msg.header = msg.header
        self.result_pub.publish(out_msg)

        if self.show_window:
            cv2.imshow('ChArUco Detection', frame)
            cv2.waitKey(1)

    def _detect_charuco(self, gray):
        """兼容新旧 API 的 ChArUco 角点检测, 返回 (corners, ids)。"""
        if self.use_new_api:
            charuco_corners, charuco_ids, _marker_corners, _marker_ids = \
                self.charuco_detector.detectBoard(gray)
            return charuco_corners, charuco_ids
        else:
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
            if marker_ids is None or len(marker_ids) == 0:
                return None, None
            retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, self.board)
            if retval <= 0:
                return None, None
            return charuco_corners, charuco_ids

    def _draw_debug(self, frame, gray, charuco_corners, charuco_ids, cam_matrix, dist_coeffs):
        """在画面上绘制检测到的 ChArUco 角点与角点数量。"""
        if self.use_new_api:
            # 重新跑一次纯 ArUco 检测以绘制码框 (开销很小, 仅用于可视化)
            _c_corners, _c_ids, marker_corners, marker_ids = \
                self.charuco_detector.detectBoard(gray)
            if marker_corners is not None and marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
        else:
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

        if charuco_corners is not None:
            cv2.aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)

        n = 0 if charuco_corners is None else len(charuco_corners)
        color = (0, 255, 0) if n >= self.min_corners else (0, 0, 255)
        cv2.putText(frame, f'ChArUco corners: {n}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)

    def _publish_tf(self, rvec, tvec, stamp):
        """将旋转向量/平移向量转换为 TF 并发布 (base_frame -> marker_frame)。"""
        rotation_matrix, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64))
        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(rotation_matrix)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.marker_frame
        t.transform.translation.x = float(tvec[0][0])
        t.transform.translation.y = float(tvec[1][0])
        t.transform.translation.z = float(tvec[2][0])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _rotation_matrix_to_quaternion(m):
        """3x3 旋转矩阵转四元数 (x, y, z, w), 避免额外依赖 tf_transformations。"""
        trace = m[0, 0] + m[1, 1] + m[2, 2]
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (m[2, 1] - m[1, 2]) / s
            qy = (m[0, 2] - m[2, 0]) / s
            qz = (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            qw = (m[2, 1] - m[1, 2]) / s
            qx = 0.25 * s
            qy = (m[0, 1] + m[1, 0]) / s
            qz = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            qw = (m[0, 2] - m[2, 0]) / s
            qx = (m[0, 1] + m[1, 0]) / s
            qy = 0.25 * s
            qz = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            qw = (m[1, 0] - m[0, 1]) / s
            qx = (m[0, 2] + m[2, 0]) / s
            qy = (m[1, 2] + m[2, 1]) / s
            qz = 0.25 * s
        return float(qx), float(qy), float(qz), float(qw)


def main(args=None):
    rclpy.init(args=args)
    node = CharucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

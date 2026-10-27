# -*- coding: utf-8 -*-
"""
ChArUco 标定板图片生成脚本 (支持 PDF 精确尺寸打印)

用途:
    生成用于打印的 ChArUco 标定板, 保证与检测节点使用完全一致的参数。
    推荐输出 PDF: PDF 使用绝对物理单位, 打印时选"实际大小/100%"即可 1:1 输出,
    彻底避免 PNG 因缺少物理尺寸而被打印软件自动缩放的问题。

    PDF 由 Python 标准库 (zlib) 直接写出, 不依赖 PIL / reportlab / img2pdf 等第三方库。

运行 (无需启动 ROS 主程序, 直接跑即可):
    ros2 run xarm7_grasp generate_charuco_board --ros-args \
        -p squares_x:=8 -p squares_y:=6 \
        -p square_length:=0.030 -p marker_length:=0.022 \
        -p dictionary:=DICT_5X5_100 \
        -p output_path:=$HOME/charuco_board.pdf \
        -p page_size:=a4 -p pdf_dpi:=300

打印提醒:
    - 打印对话框务必选择 "实际大小 / 100% / 无缩放", 关闭 "适应页面"。
    - 打印后用卡尺复测单个方格边长必须正好等于 square_length (默认 30.0mm)。
"""

import zlib

import numpy as np
import cv2

import rclpy
from rclpy.node import Node


# 常见纸张物理尺寸 (毫米, 竖放: 宽 x 高)
_PAPER_SIZES_MM = {
    'a4': (210.0, 297.0),
    'a3': (297.0, 420.0),
    'a2': (420.0, 594.0),
}


class CharucoBoardGenerator(Node):
    def __init__(self):
        super().__init__('generate_charuco_board')

        # ==================== 标定板参数 (须与检测节点一致) ====================
        self.declare_parameter('squares_x', 8)
        self.declare_parameter('squares_y', 6)
        self.declare_parameter('square_length', 0.030)  # 米
        self.declare_parameter('marker_length', 0.022)  # 米
        self.declare_parameter('dictionary', 'DICT_5X5_100')

        # ==================== 输出参数 ====================
        self.declare_parameter('output_path', 'charuco_board.pdf')  # 默认 PDF
        self.declare_parameter('page_size', 'a4')   # a4 / a3 / a2 / auto, 页面放不下会自动升级
        self.declare_parameter('pdf_dpi', 300)      # PDF 内图像分辨率, 300 足够清晰, 可设 600
        self.declare_parameter('dpi', 300)          # 仅 PNG 输出时使用
        self.declare_parameter('page_margin_mm', 8.0)  # 页面上标定板四周的最小留白

        squares_x = int(self.get_parameter('squares_x').value)
        squares_y = int(self.get_parameter('squares_y').value)
        square_length = float(self.get_parameter('square_length').value)
        marker_length = float(self.get_parameter('marker_length').value)
        dict_name = str(self.get_parameter('dictionary').value)
        output_path = str(self.get_parameter('output_path').value)
        page_size = str(self.get_parameter('page_size').value).lower()
        pdf_dpi = int(self.get_parameter('pdf_dpi').value)
        png_dpi = int(self.get_parameter('dpi').value)
        page_margin_mm = float(self.get_parameter('page_margin_mm').value)

        if marker_length >= square_length:
            raise ValueError('marker_length 必须小于 square_length')
        if not hasattr(cv2.aruco, dict_name):
            raise ValueError(f'无效的 ArUco 字典名称: {dict_name}')

        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        use_new_api = hasattr(cv2.aruco, 'CharucoDetector')

        # ==================== 构建标定板 (无额外留白, 物理尺寸精确) ====================
        if use_new_api:
            board = cv2.aruco.CharucoBoard(
                (squares_x, squares_y), square_length, marker_length, dictionary)
        else:
            board = cv2.aruco.CharucoBoard_create(
                squares_x, squares_y, square_length, marker_length, dictionary)

        # 标定板内容的物理尺寸 (毫米)
        content_w_mm = squares_x * square_length * 1000.0
        content_h_mm = squares_y * square_length * 1000.0

        is_pdf = output_path.lower().endswith('.pdf')
        render_dpi = pdf_dpi if is_pdf else png_dpi

        # 像素尺寸 = 物理尺寸(英寸) * dpi
        px_per_mm = render_dpi / 25.4
        img_w_px = int(round(content_w_mm * px_per_mm))
        img_h_px = int(round(content_h_mm * px_per_mm))

        # 渲染标定板灰度图 (margin=0, 由页面/留白提供白边)
        if use_new_api:
            img = board.generateImage((img_w_px, img_h_px), 0, 1)
        else:
            img = board.draw((img_w_px, img_h_px), 0, 1)
        img = np.ascontiguousarray(img, dtype=np.uint8)

        if is_pdf:
            page_w_mm, page_h_mm, used_page = self._select_page(
                content_w_mm, content_h_mm, page_size, page_margin_mm)
            self._write_pdf(
                output_path, img,
                content_w_mm, content_h_mm, page_w_mm, page_h_mm)
        else:
            cv2.imwrite(output_path, img)
            self._set_png_dpi(output_path, render_dpi)
            used_page = 'PNG(无页面)'
            page_w_mm = page_h_mm = 0.0

        # ==================== 打印信息 ====================
        self.get_logger().info('=' * 62)
        self.get_logger().info(f'ChArUco 标定板已生成: {output_path}')
        self.get_logger().info(
            f'  方格数: {squares_x} x {squares_y} | 字典: {dict_name}')
        self.get_logger().info(
            f'  方格边长: {square_length*1000:.1f} mm | ArUco 码边长: {marker_length*1000:.1f} mm')
        self.get_logger().info(
            f'  标定板物理尺寸: {content_w_mm:.1f} mm x {content_h_mm:.1f} mm')
        self.get_logger().info(
            f'  图像分辨率: {img_w_px} x {img_h_px} px @ {render_dpi} dpi')
        if is_pdf:
            self.get_logger().info(
                f'  PDF 页面: {used_page} ({page_w_mm:.0f} x {page_h_mm:.0f} mm), 标定板已按精确尺寸居中')
        self.get_logger().info('-' * 62)
        self.get_logger().info('打印: 选"实际大小/100%/无缩放", 关闭"适应页面"; 打印后卡尺复测方格边长!')
        self.get_logger().info('=' * 62)

    def _select_page(self, content_w_mm, content_h_mm, pref, min_margin_mm):
        """选择能容纳标定板的纸张(含最小留白), 自动尝试横放/竖放, 放不下则升级纸张或自定义。"""
        if pref in _PAPER_SIZES_MM:
            order = [pref] + [k for k in ('a4', 'a3', 'a2') if k != pref]
        else:
            order = ['a4', 'a3', 'a2']

        need_w = content_w_mm + 2 * min_margin_mm
        need_h = content_h_mm + 2 * min_margin_mm

        for name in order:
            w, h = _PAPER_SIZES_MM[name]
            for pw, ph, orient in ((w, h, '竖'), (h, w, '横')):
                if need_w <= pw and need_h <= ph:
                    return pw, ph, f'{name.upper()}{orient}放'

        # 所有标准纸张都放不下: 自定义页面 = 内容 + 留白
        return need_w, need_h, '自定义'

    def _write_pdf(self, path, img_gray, content_w_mm, content_h_mm,
                   page_w_mm, page_h_mm):
        """用标准库手写单页 PDF, 将灰度图按精确物理尺寸居中放置。"""
        mm_to_pt = 72.0 / 25.4
        page_w_pt = page_w_mm * mm_to_pt
        page_h_pt = page_h_mm * mm_to_pt
        place_w_pt = content_w_mm * mm_to_pt
        place_h_pt = content_h_mm * mm_to_pt
        # 居中
        x_pt = (page_w_pt - place_w_pt) / 2.0
        y_pt = (page_h_pt - place_h_pt) / 2.0

        img_h_px, img_w_px = img_gray.shape[:2]
        compressed = zlib.compress(img_gray.tobytes(), 9)

        obj1 = b"<< /Type /Catalog /Pages 2 0 R >>"
        obj2 = b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>"
        obj3 = (
            "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.4f %.4f] "
            "/Resources << /XObject << /Im0 4 0 R >> >> /Contents 5 0 R >>"
            % (page_w_pt, page_h_pt)
        ).encode()
        img_dict = (
            "<< /Type /XObject /Subtype /Image /Width %d /Height %d "
            "/ColorSpace /DeviceGray /BitsPerComponent 8 /Filter /FlateDecode "
            "/Length %d >>" % (img_w_px, img_h_px, len(compressed))
        ).encode()
        obj4 = img_dict + b"\nstream\n" + compressed + b"\nendstream"
        content_stream = (
            "q\n%.4f 0 0 %.4f %.4f %.4f cm\n/Im0 Do\nQ\n"
            % (place_w_pt, place_h_pt, x_pt, y_pt)
        ).encode()
        obj5 = (b"<< /Length %d >>" % len(content_stream)) + b"\nstream\n" + \
            content_stream + b"\nendstream"

        out = bytearray()
        out += b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n"
        offsets = {}
        for num, body in ((1, obj1), (2, obj2), (3, obj3), (4, obj4), (5, obj5)):
            offsets[num] = len(out)
            out += ("%d 0 obj\n" % num).encode() + body + b"\nendobj\n"

        xref_pos = len(out)
        out += b"xref\n0 6\n"
        out += b"0000000000 65535 f \n"
        for num in range(1, 6):
            out += ("%010d 00000 n \n" % offsets[num]).encode()
        out += ("trailer\n<< /Size 6 /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n"
                % xref_pos).encode()

        with open(path, 'wb') as f:
            f.write(bytes(out))

    @staticmethod
    def _set_png_dpi(path, dpi):
        """仅在输出 PNG 时尝试写入 dpi 元信息 (需要 PIL, 无则忽略)。"""
        try:
            from PIL import Image
            im = Image.open(path)
            im.save(path, dpi=(dpi, dpi))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CharucoBoardGenerator()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

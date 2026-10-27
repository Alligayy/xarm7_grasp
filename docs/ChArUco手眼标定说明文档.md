
### xArm7 + D455 眼在手标定说明文档（ChArUco 版）

> 本文档为 ChArUco 标定板版本，替代原 ArUco 单码方案，用于提升单帧位姿解算的精度与鲁棒性。
> 原 ArUco 版说明见 `手眼标定说明文档.md`，两者标定流程一致，仅"标定板 + 识别节点"不同。

#### 1. 软硬件基础环境

* **操作系统与 ROS 版本**：Ubuntu 22.04 + ROS2 Humble
* **硬件配置**：xArm7 机械臂 + RealSense D455 深度相机（眼在手 Eye-in-Hand 安装）
* **核心软件包**：`easy_handeye2`（源码编译）、`xarm7_grasp`（本包，提供 ChArUco 检测节点）
* **相机内参**：使用 RealSense D455 出厂内参，由节点从 `/camera/camera/color/camera_info` 话题动态读取，无需手动填写。
* **标定板要求（ChArUco）**：

  | 参数 | 值 | 说明 |
  |------|------|------|
  | 方格数 (squares_x × squares_y) | 8 × 6 | 列 × 行 |
  | 方格边长 square_length | 30.0 mm | **打印后必须卡尺复测** |
  | ArUco 码边长 marker_length | 22.0 mm | 必须小于方格边长 |
  | 字典 dictionary | DICT_5X5_100 | 生成与检测须一致 |
  | 整板物理尺寸 | 240 mm × 180 mm | 不含四周留白 |

  > **为何用 ChArUco**：它兼具棋盘格的海量亚像素角点（PnP 更鲁棒）与 ArUco 的唯一 ID、抗遮挡、尺度确定特性，单帧位姿质量明显优于单个 ArUco 码。


#### 2. 标定板打印

1. **生成打印文件**（先完成第 3 节的编译）：

   ```bash
   ros2 run xarm7_grasp generate_charuco_board --ros-args \
     -p squares_x:=8 -p squares_y:=6 \
     -p square_length:=0.030 -p marker_length:=0.022 \
     -p dictionary:=DICT_5X5_100 \
     -p output_path:=$HOME/charuco_board.pdf \
     -p page_size:=a4 -p pdf_dpi:=300
   ```

   > **务必用 PDF 而非 PNG 打印**：PNG 没有强制的物理尺寸概念，多数打印软件会忽略其 DPI 元数据或默认“适应页面”，导致缩放走样；**PDF 使用绝对物理单位（毫米）**，标定板被按精确尺寸居中放到页面上，选“实际大小/100%”即可 1:1 打印。本脚本用 Python 标准库直接写出 PDF，**不依赖 PIL/reportlab 等第三方库**。
   > * `page_size` 可选 `a4`/`a3`/`a2`/`auto`，页面放不下时脚本会自动升级纸张并选择横/竖放；
   > * 若仍想输出 PNG，把 `output_path` 改成 `.png` 结尾即可（用 `-p dpi` 设置分辨率），但不推荐用于打印。

2. **打印要求**：
   * 打印对话框务必选择 **“实际大小 / 100% / 无缩放”**，**关闭“适应页面 / 缩放到可打印区域”**，任何自动缩放都会破坏尺度精度。
   * 打印后用**卡尺复测单个方格边长必须正好 30.0 mm**。
   * 若打印机存在固定缩放误差：以实测值为准，把生成脚本、检测节点 launch 中的 `square_length` **同步改成实测值**（尺度准确比一切都重要）。

3. **装裱**：将打印件平整粘贴在**硬质平板**（亚克力/铝板）上，确保无翘曲、无褶皱，平整度直接影响 PnP 精度。


#### 3. 编译

```bash
cd ~/2/Grasp/grasp_ws          # 工作空间根目录
colcon build --packages-select xarm7_grasp
source install/setup.bash
```

> **OpenCV 版本自适应**：`charuco_detector` 节点会自动判别 OpenCV 新旧 ArUco API（4.7+ 与系统自带 4.5.4 接口不同），无需手动改代码。


#### 4. 标定执行步骤

1. **一键启动**：

   ```bash
   ros2 launch xarm7_grasp calibration_handeye2_charuco.launch.xml
   ```

   该 launch 包含：
   * 启动真机机械臂 + MoveIt2 + RViz（`grasp_real.launch.py`）
   * 启动 RealSense D455（标定阶段关闭点云与深度对齐）
   * 启动 ChArUco 识别节点（`charuco_detector`），发布 `camera_color_optical_frame → camera_marker` 的 TF 及可视化图像 `/charuco/result_image`
   * 启动 `easy_handeye2` 手眼标定程序

   **若需手动分步启动**（便于调试）：

   *1*. 先启动真机机械臂与相机（点云、深度对齐设为 false）。

   *2*. 启动 ChArUco 识别节点（*新开终端*）：
   ```bash
   ros2 run xarm7_grasp charuco_detector --ros-args \
     -p squares_x:=8 -p squares_y:=6 \
     -p square_length:=0.030 -p marker_length:=0.022 \
     -p dictionary:=DICT_5X5_100 -p min_corners:=6 \
     -p image_topic:=/camera/camera/color/image_raw \
     -p camera_info_topic:=/camera/camera/color/camera_info \
     -p base_frame:=camera_color_optical_frame \
     -p marker_frame:=camera_marker
   ```

   *3*. 启动 easy_handeye2（*新开终端，与 ArUco 版完全一致*）：
   ```bash
   ros2 launch easy_handeye2 calibrate.launch.py \
     name:=xarm7_d455_calibration \
     calibration_type:=eye_in_hand \
     tracking_base_frame:=camera_color_optical_frame \
     tracking_marker_frame:=camera_marker \
     robot_base_frame:=link_base \
     robot_effector_frame:=link_eef
   ```

2. **视觉监控**：启动 `easy_handeye2` 前，新开终端运行 `ros2 run rqt_image_view rqt_image_view`，订阅 **`/charuco/result_image`**（替代原 `/aruco_single/result`）：
   * 左上角文字 `ChArUco corners: N` **变绿且 N ≥ 6**：识别成功，正在发布 TF；
   * 文字**变红**：角点太少，该帧被丢弃（避免劣质位姿污染采样），需调整相机与标定板相对位姿；
   * 画面上应稳定出现绿色角点与三色坐标轴。

3. **数据采集**：
   * 在 `easy_handeye2` GUI 中点击 `Take Sample` 采点。
   * 用示教器或 RViz2 移动机械臂改变相机姿态，**必须保证相机清晰稳定地看到标定板**。
   * **采样要求**：采集 15~20 个点，姿态变化丰富，在 X、Y、Z 平移的同时务必包含 Roll、Pitch、Yaw 的倾斜与旋转。
   * **每次采点前**：确认坐标轴稳定不抖动、机械臂完全静止、角点数为绿色。

4. **解算与保存**：
   * 在 GUI 切换算法解算（后台自动计算）。
   * 确认后点击 `Save`，文件以 `.calib` 格式保存在隐藏目录 `/home/gao/.ros2/easy_handeye2/calibrations/xarm7_d455_calibration.calib`（在 `gao` 文件夹下按 `Ctrl+H` 显示隐藏文件）。


#### 5. 算法选择与误差验证

* **首选算法**：`OpenCV/Daniilidis`（双四元数模型，同步求解旋转和平移，抗噪性强、精度最高）。备选 `Tsai-Lenz` 或 `Park`。
* **物理校验（盲测）**：观察终端输出的 4×4 齐次变换矩阵第四列（平移向量 X, Y, Z）。用卷尺实测 D455 相机中心到机械臂末端法兰盘 (`link_eef`) 的物理距离，两者基本吻合（毫米级误差）则标定有效。
* **RViz2 最终校验**：标定完成后，在抓取启动文件中开启点云 (`align_depth.enable:=true`)。让机械臂末端触碰桌面特征点，观察 RViz2 中机械臂模型末端与真实点云是否严丝合缝。


#### 6. 关键注意事项 (Troubleshooting)

* **标定板参数三处一致**：`generate_charuco_board`（生成）、`calibration_handeye2_charuco.launch.xml`（检测）、实物尺寸三者必须完全对应，任何一处不一致都会引入系统性误差。
* **深度对齐参数**：标定期间务必将 `align_depth.enable` 保持为 `false`。标定仅依赖 RGB 图像（PnP 算法），关闭深度对齐可节省算力带宽、消除延迟、避免相机发热，确保采样瞬间画面绝对清晰。抓取阶段再开启。
* **TF 未发布 / easy_handeye2 报错**：多为角点数不足（`/charuco/result_image` 显示红色）。检查标定板是否平整、光照是否均匀、相机与板的相对角度是否过大、`square_length` 是否为实测值。
* **精度不升反降**：手眼最终精度仍高度依赖**采样姿态多样性**与**尺度准确性**，ChArUco 只保证识别环节，采样策略仍需严格把控。
* **修改 .calib 后须重启**：手眼标定 TF 由 easy_handeye2 的 publish launch 在启动时一次性读取 `.calib`，改动文件后必须重启该 launch 才生效。

##### 6.1 启动报错：rqt 标定 GUI 加载失败 `numpy has no attribute 'float'`

**报错特征**：一键启动 launch 后，终端出现类似：

```
[rqt_calibrator.py] RosPluginProvider.load(easy_handeye2/RqtHandeyeCalibrator) exception raised ...
  File ".../transforms3d/quaternions.py", line 26, in <module>
    _MAX_FLOAT = np.maximum_sctype(np.float)
AttributeError: module 'numpy' has no attribute 'float'
PluginManager._load_plugin() could not load plugin "easy_handeye2/RqtHandeyeCalibrator"
```

标定 GUI（`Take Sample` 界面）无法弹出。

**根因**：与本项目的标定代码/launch **无关**，是 Python 环境依赖冲突。easy_handeye2 依赖 `transforms3d`；系统 apt 版 `transforms3d 0.3.1` 仍使用已废弃的 `np.float`，而 `~/.local` 中 pip 安装的 `numpy >= 1.24` 已移除 `np.float`，二者冲突（Ubuntu 22.04 系统自带 numpy 1.21 本可兼容，是被 `~/.local` 的新版 numpy 覆盖导致）。

**修复**（升级 transforms3d，**不要动 numpy**，以免影响 cv_bridge/OpenCV）：

```bash
python3 -m pip install --user --upgrade "transforms3d>=0.4.1"
# 验证: 下面命令不再报错即修复成功
python3 -c "import transforms3d.quaternions; print('OK', transforms3d.__version__)"
```

0.4.2 版已重写该段代码、删除 `np.float` 用法。安装到 `~/.local` 覆盖系统旧版，**永久生效，无需每次重做**。修复后重新启动 launch，GUI 插件即可正常加载。


#### 7. 相关文件清单

| 文件 | 作用 |
|------|------|
| `xarm7_grasp/charuco_detector.py` | ChArUco 检测节点，发布 `camera_marker` TF 与可视化图像 |
| `xarm7_grasp/generate_charuco_board.py` | 按精确物理尺寸生成标定板打印图 |
| `launch/calibration_handeye2_charuco.launch.xml` | ChArUco 版一键标定 launch |
| `setup.py` | 注册上述两个可执行文件 |

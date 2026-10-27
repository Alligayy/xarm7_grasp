# xarm7_grasp

基于 **ROS 2 Humble** 的 xArm-7 机械臂视觉抓取系统，集成 RealSense D455 深度相机、手眼标定（ArUco / ChArUco）与 GSNet 抓取位姿检测，并包含一套 **未经验证** 的 Xela 触觉传感器力反馈夹持代码（见下文标注）。

> ⚠️ **重要声明**：本仓库中与触觉力控相关的代码（`grasp_policy_xarm.py` 的触觉模式、`tactile_gripper.py`、`tactile_force.py`、`config/tactile_grasp.yaml` 及 `docs/tactile_grasp.md`）为生成代码，**从未在实际硬件上运行验证过，相关命令能否正常执行未知**，仅供参考，不可直接用于实机。其余视觉抓取、手眼标定与运动控制部分均经过实际验证。

## 功能特性

- **抓取策略执行**：接收 GSNet 检测节点发布的抓取位姿（`/grasp_pose`、`/grasp_width`），通过 `xarm_planner` 服务完成位姿规划、直线规划与轨迹执行，支持仿真与真机两种模式
- **触觉力反馈夹持（⚠️ 未验证）**：基于 Xela 双侧触觉传感器（`/xServTopic`）的位置式力控闭合设计，包含超力/数据超时等故障锁定与停止机制。**仅为生成的代码，未在真实机械臂与传感器上运行过，能否正常工作未知**
- **可达性几何预筛**：利用手眼标定 TF 将相机系抓取位姿变换到基座系，剔除接近方向不合理（易导致奇异点）的候选位姿
- **手眼标定**：支持 easy_handeye2 的经典 ArUco 流程与自研 ChArUco 检测节点（`charuco_detector`），并提供标定板生成工具（`generate_charuco_board`，输出 PDF 保证打印精度）
- **运动控制示例**：关节运动、笛卡尔直线运动、回 home 位、夹爪开合等测试脚本

## 目录结构

```
xarm7_grasp/
├── config/                  # 参数与标定结果
│   ├── tactile_grasp.yaml   # 触觉抓取力控参数（⚠️ 未验证）
│   └── *.calib              # easy_handeye2 手眼标定结果（多种算法）
├── docs/                    # 说明文档（手眼标定、ChArUco、触觉抓取⚠️未验证）
├── launch/                  # 启动文件
│   ├── grasp_simulation.launch.py   # 仿真（fake）模式
│   ├── grasp_real.launch.py         # 真机模式
│   ├── calibration_handeye2*.xml    # 手眼标定
│   └── xarm7_grasp_all.launch.xml   # 一键启动
├── msg/                     # 自定义消息 GraspCfg.msg
├── rviz/                    # RViz 可视化配置
├── test/                    # pytest 离线测试（触觉力控，仅逻辑层面）
├── utils/                   # 工具函数
└── xarm7_grasp/             # 节点源码
    ├── grasp_policy_xarm.py         # 实际使用的抓取策略（xarm_planner 服务版）
    ├── grasp_policy_xarm_joint.py   # 关节空间抓取策略
    ├── grasp_policy.py              # MoveGroup 动作版抓取策略
    ├── tactile_gripper.py           # 触觉力反馈夹持控制（⚠️ 未验证）
    ├── tactile_force.py             # 触觉力处理（⚠️ 未验证）
    ├── charuco_detector.py          # ChArUco 板检测节点
    ├── generate_charuco_board.py    # ChArUco 标定板生成
    ├── go_home*.py / test_*.py      # 运动与夹爪测试脚本
    └── rgbd_display.py              # RGB-D 数据可视化
```

## 依赖环境

- Ubuntu 22.04 + ROS 2 Humble
- [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2)（`xarm_msgs`、`xarm_planner`、`xarm_moveit_config`）
- MoveIt 2
- RealSense D455 + `realsense2` ROS 2 wrapper
- easy_handeye2（手眼标定）
- Xela `xela_server_ros2`（触觉传感器，仅力反馈夹持需要，**该部分未验证**）
- OpenCV / NumPy / transforms3d

## 构建

先 source 已构建的 xarm 与 Xela 工作空间，再构建本包：

```bash
source /opt/ros/humble/setup.bash
source /path/to/xarm_ws/install/setup.bash
source /path/to/xela_ws/install/setup.bash   # 仅触觉抓取需要
cd /path/to/grasp_ws
colcon build --packages-select xarm7_grasp --symlink-install
source install/setup.bash
```

## 使用

### 仿真 / 真机启动

```bash
# 仿真（fake）模式：MoveIt2 + xArm7 + 规划服务 + RViz
ros2 launch xarm7_grasp grasp_simulation.launch.py

# 真机模式：加载相机与真实夹爪
ros2 launch xarm7_grasp grasp_real.launch.py
```

前提：RealSense 相机已启动、手眼标定 TF 已发布、GSNet 检测节点已运行。

### 执行抓取策略

```bash
# 常规视觉抓取（已验证）
ros2 run xarm7_grasp grasp_policy_xarm
```

> ⚠️ 以下触觉力反馈抓取命令 **暂未完成运行验证，仅供参考**，请勿直接在实机上执行：
>
> ```bash
> # 触觉力反馈抓取（未验证；理论上需先在 config/tactile_grasp.yaml 中完成传感器标定配置）
> ros2 run xarm7_grasp grasp_policy_xarm \
>   --ros-args --params-file config/tactile_grasp.yaml
> ```

### 手眼标定

```bash
# ChArUco 方案（推荐，见 docs/ChArUco手眼标定说明文档.md）
ros2 launch xarm7_grasp calibration_handeye2_charuco.launch.xml
ros2 run xarm7_grasp charuco_detector

# 生成标定板（输出 PDF，按实际尺寸打印后用卡尺复测）
ros2 run xarm7_grasp generate_charuco_board
```

### 触觉力控相关离线测试（⚠️ 力控功能未验证，处于开发阶段）

以下测试仅验证力控代码的离线逻辑（未接触过真实传感器与机械臂）：

```bash
python3 -m pytest -q test/test_tactile_force.py test/test_tactile_samples.py
```

## 文档

- [手眼标定说明文档](docs/手眼标定说明文档.md)
- [ChArUco 手眼标定说明文档](docs/ChArUco手眼标定说明文档.md)
- [触觉抓取说明文档](docs/tactile_grasp.md)（⚠️ 对应未验证的力控代码）

## 注意事项

- **触觉力控部分（代码、配置、文档、命令）暂未完成运行验证**；若打算使用，必须自行承担验证与调试工作，严禁未经充分测试直接用于实机
- 触觉力控参数（目标力、力上限、行程等）均为假想的调试起点，实机使用前必须完成传感器归零、受压方向与单位标定，并将 `calibration_confirmed` 置为 `true`
- 若未来开展力控实机调试，应先在支撑面上用可压缩试件验证，确认停止行为后再执行自动搬运
- 不要同时从 RViz、其他脚本或手柄发送夹爪命令

## 许可

本项目基于 [MIT License](LICENSE) 开源。

# xArm7 + Xela 双侧触觉抓取

`grasp_policy_xarm` 现在通过 Xela 反馈驱动官方位置夹爪：低速、小步闭合，两侧达到目标力并持续稳定后抬起，搬运期间继续调节位置。这是外部位置式力反馈控制，响应受 ROS 服务、夹爪速度和传感器刷新率限制。不能保证无瞬时超调，也没有实现剪切滑移检测或自动寻找最小夹持力。

## 首次配置（先在支撑面上用可压缩试件验证）

1. 贴好两侧传感器并固定线缆。更新 MoveIt 中指垫碰撞尺寸、TCP/抓取深度补偿；原策略的 `GRASP_DEPTH_OFFSET=0.04` 没有包含新指垫的实测厚度。GSNet 的宽度还未用于快速预闭合，避免标定误差导致挤压。
2. 启动现有 xarm driver、MoveIt、Xela server 与 `xela_service`。在同一终端 source xarm、Xela 和 grasp 三个工作空间。
3. 查看 `ros2 topic echo /xServTopic --once`。记录两侧 `sensor_pos`，确认每侧 `forces` 与 `taxels` 数量相等。当前 xela_service 的 `forces` 来自 server 的 `calibrated` 字段：必须确认你的 server 标定输出单位确实为 N。只有十六进制原始 `taxels` 时程序拒绝抓取。不能把 ADC 读数换个名字当成牛顿。
4. 在 server 中完成无载归零及已知载荷校验；分别轻压每个指垫，确定法向轴和受压符号。`normal_axis` 对两侧共用，`left_sign/right_sign` 各自为 +1.0 或 -1.0。如果两侧法向轴不同，需先统一传感器输出坐标。错误符号会掩盖压力，必须先验证。
5. 修改 `config/tactile_grasp.yaml` 中的编号、方向，确认上述标定后设置 `calibration_confirmed: true`。确定贴传感器后的最小允许夹爪位置，设置 `close_position`。官方原生位置约为 0 关闭、850 张开，与原 MoveIt 的 drive_joint 弧度方向相反。值不是毫米。确认完全张开不会拉扯线缆。
6. 设定适合试件的目标力、总力上限和单 taxel 力上限。默认每侧 1 N、总力上限 2 N、单 taxel 上限 0.6 N 仅为调试起点，不代表适用于某种材料。若物体仍明显变形，降低目标力、步长和速度。若单侧已经达到目标而另一侧尚未接触，控制器停止继续压紧并等待超时，应检查抓取对中。

## 构建与运行

在 grasp 工作空间构建前先 source 已构建的 xarm 与 Xela 工作空间：

```bash
source /opt/ros/humble/setup.bash
source /media/gao/2/Grasp/xarm_ws/install/setup.bash
source /media/gao/2/Grasp/xela_ws/install/setup.bash
cd /media/gao/2/Grasp/grasp_ws
colcon build --packages-select xarm7_grasp --symlink-install
source install/setup.bash
ros2 run xarm7_grasp grasp_policy_xarm --ros-args --params-file /media/gao/2/Grasp/grasp_ws/src/xarm7_grasp/config/tactile_grasp.yaml
```

运行前用 `ros2 service list` 确认命名空间（默认 `/xarm`）下存在 `get_gripper_position`、`set_gripper_position`、`set_gripper_speed`、`set_gripper_enable`、`set_gripper_mode`、`set_state`。不要同时从 RViz、其他脚本或手柄发送夹爪命令；本策略在触觉模式下直接使用驱动服务。

启动时先校验触觉并张开夹爪，再按原有终端确认流程到观察位姿。每次抓取前重新张开并检查空载；力按每个 taxel 的受压方向投影，负值截断后分别求两侧总和。确认稳定后才抬起。搬运中仍保留原路径人工确认，等待确认期间力环继续运行。

## 异常处理与限制

任一侧超力、单点超力、数据缺失/重复不更新/超过 0.5 s、服务失败、行程耗尽、双侧接触丢失或夹持超时都会锁定故障。循环请求 `set_state=4` 停止机械臂，并尽力把夹爪目标设为当前实测位置；不会自动松爪让物体掉落。停止服务本身也可能失败，错误会记录日志。没有硬件级急停或夹爪力矩断路保证；故障后应人工支撑物体并处理，再恢复驱动和重启策略。

触觉故障发生于已经提交的机械臂轨迹时，靠驱动停止服务中断；后续执行前也检查故障。ROS 服务超时的指令可能仍在驱动端执行，必须人工检查，程序不自动重试或继续抓取。超力后保持当前位置可能仍保留压力，需要人工及时卸载。

这是代码及离线测试完成的版本，尚未在你的机械臂、XR1944 或真实物体上验证。实机首次调试应先确认受压符号、力值单位、允许行程、速度和停止行为，再执行自动搬运。仅修改 `grasp_policy_xarm.py` 所用策略；`grasp_policy_xarm_joint.py` 保持原行为。

测试（已 source ROS 和两个接口工作空间）：

```bash
python3 -m pytest -q test/test_tactile_force.py test/test_tactile_samples.py
```

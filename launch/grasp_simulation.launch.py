import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ============================================================
    # 【新增】运动规划管线选择开关
    # "xarm_ompl" : 自研 XArmBiRRT 管线（多 IK 目标 + 直连快路径 + 路径强简化，
    #               轨迹质量更好；需先编译 xarm_ompl_planner 插件包并 source）
    # ""          : 留空 = 官方默认 ompl 管线（RRTConnect，与原行为完全一致）
    # 改完后需重启本 launch 生效（参数在 xarm_planner_node 启动时读取一次）。
    # ============================================================
    planning_pipeline_id = None # 'xarm_ompl'

    # 自动去底层的 xarm_ws 寻找官方包
    xarm_moveit_config_dir = get_package_share_directory('xarm_moveit_config')
    xarm_planner_dir = get_package_share_directory('xarm_planner')
    
    # 在当前工作空间寻找你的 rviz 文件
    my_package_dir = get_package_share_directory('xarm7_grasp')
    # 加载自己的 rviz 文件                                                                        # xarm7_camera_grasp_vision
    rviz_config_file = os.path.join(my_package_dir, 'rviz', 'xarm7_grasp_vision.rviz')  # xarm7_grasp_vision

    # 调用官方 launch，并加载相机和夹爪
    xarm_fake_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xarm_moveit_config_dir, 'launch', 'xarm7_moveit_fake.launch.py')
        ),
        launch_arguments={
            'add_gripper': 'true',
            'add_realsense_d435i': 'true',
            # 'add_d435i_links':'false',
            'rviz_config': rviz_config_file, # 该参数的定义在_robot_moveit_common2.launch.py文件中
        }.items()
    )

    # 启动 xarm_planner_node（xarm_ros2 官方规划服务节点）
    # 提供 xarm_pose_plan / xarm_joint_plan / xarm_straight_plan / xarm_exec_plan 服务，
    # 供 test_move_xarm.py 等脚本通过简单服务调用移动机械臂
    import json
    # 【新增】把管线选择作为 node_parameters 传给 xarm_planner_node
    # （_robot_planner.launch.py 会把 node_parameters 合并进节点参数，
    #   xarm_planner.cpp 修改5 中读取该参数并调用 setPlanningPipelineId）
    planner_node_parameters = json.dumps(
        {'pipeline_id': planning_pipeline_id}) if planning_pipeline_id else '{}'

    xarm_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xarm_planner_dir, 'launch', '_robot_planner.launch.py')
        ),
        launch_arguments={
            'dof': '7',
            'robot_type': 'xarm',
            'add_gripper': 'true',
            # 【新增】运动规划管线选择（见顶部 planning_pipeline_id 注释）
            'node_parameters': planner_node_parameters,
        }.items()
    )

    return LaunchDescription([
        xarm_fake_launch,
        xarm_planner_launch
    ])

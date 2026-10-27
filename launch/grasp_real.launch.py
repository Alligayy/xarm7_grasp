
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    # 查找 xarm_moveit_config 包的路径
    xarm_moveit_config_dir = get_package_share_directory('xarm_moveit_config')
    xarm_planner_dir = get_package_share_directory('xarm_planner')
    
    # 在xarm7_grasp工作空间寻找rviz 文件
    my_package_dir = get_package_share_directory('xarm7_grasp')
    # 加载rviz 文件
    rviz_config_file = os.path.join(my_package_dir, 'rviz', 'xarm7_grasp_vision.rviz')  # xarm7_grasp_vision

    # 调用官方launch，并加载相机和夹爪
    xarm_fake_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xarm_moveit_config_dir, 'launch', 'xarm7_moveit_realmove.launch.py')
        ),
        launch_arguments={
            'robot_ip': '192.168.1.237',
            'add_gripper': 'true',
            'add_realsense_d435i': 'false',
            'rviz_config': rviz_config_file, # 该参数的定义在_robot_moveit_common2.launch.py文件中
        }.items()
    )

    # 启动 xarm_planner_node（xarm_ros2 官方规划服务节点）
    # 提供 xarm_pose_plan / xarm_joint_plan / xarm_straight_plan / xarm_exec_plan 服务，
    # 供 test_move_xarm.py 等脚本通过简单服务调用移动机械臂
    xarm_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xarm_planner_dir, 'launch', '_robot_planner.launch.py')
        ),
        launch_arguments={
            'dof': '7',
            'robot_type': 'xarm',
            'add_gripper': 'true',
        }.items()
    )

    return LaunchDescription([
        xarm_fake_launch,
        xarm_planner_launch
    ])
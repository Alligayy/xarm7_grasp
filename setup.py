
import os		# 手动新增
from glob import glob	# 手动新增
from setuptools import find_packages, setup

package_name = 'xarm7_grasp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), # 手动新增
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),			# 手动新增
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gao',
    maintainer_email='gao@todo.todo',
    description='xArm-7 vision-tactile grasping system: grasp policy execution, hand-eye calibration (ArUco/ChArUco) and Xela tactile force-feedback grasping on ROS 2.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        # 格式：命令别名 = 包名.文件名:主函数入口
         'test_move = xarm7_grasp.test_move:main',
         'test_move_xarm = xarm7_grasp.test_move_xarm:main',
         'test_move_cartesian = xarm7_grasp.test_move_cartesian:main',
         'test_gripper = xarm7_grasp.test_gripper:main',
         'test_gripper_xarm = xarm7_grasp.test_gripper_xarm:main',
         'grasp_policy = xarm7_grasp.grasp_policy:main',
         'grasp_policy_xarm = xarm7_grasp.grasp_policy_xarm:main',
         'grasp_policy_xarm_joint = xarm7_grasp.grasp_policy_xarm_joint:main',
         'set_env = xarm7_grasp.set_env:main',
         'go_home = xarm7_grasp.go_home:main',
         'go_home_xarm = xarm7_grasp.go_home_xarm:main',
         'rgbd_display = xarm7_grasp.rgbd_display:main',
         'charuco_detector = xarm7_grasp.charuco_detector:main',
         'generate_charuco_board = xarm7_grasp.generate_charuco_board:main',
        ],
    },
)

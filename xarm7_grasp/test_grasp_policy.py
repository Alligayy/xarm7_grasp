#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" xArm Grasp Policy (ROS2 Humble Version).
    Adapted from ROS1 implementation.
"""

import numpy as np
import os
import sys
import time
import threading

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException

from std_msgs.msg import Bool, Int8, String
import geometry_msgs.msg as geometry_msgs

# 假设你的自定义服务已经在 ROS2 环境下编译完成
from xarm_planner.srv import pose_plan, joint_plan, exec_plan, single_straight_plan
from xarm_msgs.srv import GripperMove, GripperConfig
from xarm_grasp.msg import GraspCfg

sys.path.append(os.path.join(os.path.dirname(__file__), 'utils'))
import tool as utool

class PolicyXArm7(Node):
    def __init__(self):
        super().__init__('grasp_agent')

        # Initialize variables
        self.grasp_start = False
        self.grasp_pose = None
        self.pregrasp_pose = None
        self.liftgrasp_pose = None
        self.detection_flag = False
        self.grasp_num = 1
        self.publish_grasp_pose = None

        # Service Clients
        self.line_client = self.create_client(pose_plan, "/xarm_pose_plan")
        self.cartesian_client = self.create_client(single_straight_plan, "/xarm_straight_plan")
        self.exec_client = self.create_client(exec_plan, "/xarm_exec_plan")
        self.gripper_plan_client = self.create_client(joint_plan, "gripper_joint_plan")
        self.gripper_exec_client = self.create_client(exec_plan, "gripper_exec_plan")

        # Subscribers and Publishers
        self.create_subscription(Int8, "/grasp_start", self.graspStart, 1)
        self.create_subscription(GraspCfg, "/grasp_pose", self.updateGraspPose, 10)
        self.start_detection_pub = self.create_publisher(Int8, "/start_detection", 1)

        # Wait for required services
        self.get_logger().info('Waiting for xArm planning services...')
        self.line_client.wait_for_service()
        self.cartesian_client.wait_for_service()
        self.exec_client.wait_for_service()

        # Hardcoded Poses
        self.watch_pose = geometry_msgs.Pose(
            position=geometry_msgs.Vector3(x=0.333, y=0.155, z=0.377), 
            orientation=geometry_msgs.Quaternion(x=0.999, y=0.013, z=-0.031, w=0.003))
        self.place_pose = geometry_msgs.Pose(
            position=geometry_msgs.Vector3(x=0.430, y=-0.219, z=0.275), 
            orientation=geometry_msgs.Quaternion(x=0.997, y=-0.062, z=0.045, w=-0.002))
            
        self.gripper_open_width = [0.1,]
        self.gripper_close_width = [0.8,]

        # Initial move[cite: 3]
        self.get_logger().info('Move to watch pose') 
        self.move_xArm(self.watch_pose)
        self.get_logger().info('Open gripper')
        self.move_gripper(self.gripper_open_width)

        # TF2 Setup[cite: 3]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

        # Timer for publishing grasp_pose TF frame (0.8s interval)[cite: 3]
        self.timer = self.create_timer(0.8, self.publish_grasp_pose_frame)
        self.get_logger().info('Publish grasp_pose frame initialized')

        # 启动独立的后台线程运行抓取流程，避免死锁 ROS2 的 Executor
        self.grasp_thread = threading.Thread(target=self.run_grasp, daemon=True)
        self.grasp_thread.start()
    
    def graspStart(self, data):
        self.get_logger().info('Received grasp_start')
        self.grasp_start = True
        # Start first detection[cite: 3]
        if not self.detection_flag:
            msg = Int8()
            msg.data = 1
            self.start_detection_pub.publish(msg)
            self.detection_flag = True

    def adjust_rotation_matrix(self, R):
        # Original coordinate system base vector (Z-axis)[cite: 3]
        original_z = np.array([0, 0, 1])
        rotated_z = np.dot(R, original_z)

        # Calculate the angle between the rotated z-axis and the original z-axis[cite: 3]
        angle_z = utool.angle_between_vectors(rotated_z, original_z)
        if not (90 < angle_z < 180 or -180 < angle_z < -90):
            self.get_logger().error("Grasp pose is incorrect")

        # Original coordinate system's x-axis[cite: 3]
        original_x = np.array([1, 0, 0])
        rotated_x = np.dot(R, original_x)

        # Calculate the angle between the rotated x-axis and the original x-axis[cite: 3]
        angle_x = utool.angle_between_vectors(rotated_x, original_x)
        if not (-90 <= angle_x <= 90):
            # Rotate 180 degrees around the z-axis (adjustment matrix)[cite: 3]
            R_adjusted = np.dot(R, np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]))
            R = R_adjusted

        return angle_x, R

    def updateGraspPose(self, data):
        """
        Input: data: Grasp pose in the camera coordinate system[cite: 3]
        Return: Grasp pose and pre-grasp pose in the base coordinate system[cite: 3]
        """
        # Get the homogeneous transformation matrix from camera to base coordinate system (T_base_camera)[cite: 3]
        try:
            # 在 ROS2 中使用 rclpy.time.Time() 获取最新变换
            trans_camera = self.tfBuffer.lookup_transform(
                'link_base', 
                'camera_color_optical_frame', 
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform link_base to camera: {ex}')
            return None

        t_base_camera = np.array([
            trans_camera.transform.translation.x, 
            trans_camera.transform.translation.y, 
            trans_camera.transform.translation.z])
        q_base_camera = np.array([
           - trans_camera.transform.rotation.x, 
           - trans_camera.transform.rotation.y, 
           - trans_camera.transform.rotation.z,
           - trans_camera.transform.rotation.w])
        T_base_camera = utool.getTransformMat(t_base_camera, utool.quaternion_to_rotation_matrix(q_base_camera))

        # Grasp pose in the camera coordinate system (T_camera_grasp)[cite: 3]
        t_camera_grasp = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z])
        q_camera_grasp = np.array([
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w])
        T_camera_grasp = utool.getTransformMat(t_camera_grasp, utool.quaternion_to_rotation_matrix(q_camera_grasp))

        # Calculate grasp pose in the base coordinate system[cite: 3]
        T_base_grasp = np.matmul(T_base_camera, T_camera_grasp)

        # Adjust the grasp pose to fit xArm's grasping[cite: 3]
        T_y_90 = np.array([
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1] ])
        T_base_grasp = np.matmul(T_base_grasp, T_y_90)    
        
        angle, adjusted_R = self.adjust_rotation_matrix(T_base_grasp[:3, :3])
        self.get_logger().info(f"x-axis_angle: {angle}°")
        T_base_grasp[:3, :3] = adjusted_R

        q_base_grasp = utool.rotation_matrix_to_quaternion(T_base_grasp[:3, :3])
        q_base_grasp_normalized = q_base_grasp / np.linalg.norm(q_base_grasp)

        # Set grasp pose[cite: 3]
        self.grasp_pose = GraspCfg()
        self.grasp_pose.pose = geometry_msgs.Pose(
            position=geometry_msgs.Vector3(x=T_base_grasp[0, 3], y=T_base_grasp[1, 3], z=max(T_base_grasp[2, 3], 0.02)), 
            orientation=geometry_msgs.Quaternion(
                x=q_base_grasp_normalized[0], y=q_base_grasp_normalized[1],
                z=q_base_grasp_normalized[2], w=q_base_grasp_normalized[3])
        )
        self.grasp_pose.width = data.width
        self.publish_grasp_pose = self.grasp_pose
        
        self.get_logger().info(f"grasp_pose:\ntranslation:{self.grasp_pose.pose.position}\norientation:{self.grasp_pose.pose.orientation}\nwidth:{self.grasp_pose.width}")

        # Calculate pre-grasp pose[cite: 3]
        T_base_pregrasp = T_base_grasp
        self.pregrasp_pose = GraspCfg()
        self.pregrasp_pose.pose = geometry_msgs.Pose(
            position=geometry_msgs.Vector3(x=T_base_pregrasp[0, 3], y=T_base_pregrasp[1, 3], z=max(T_base_pregrasp[2, 3] + 0.05, 0.02)), 
            orientation=geometry_msgs.Quaternion(
                x=q_base_grasp_normalized[0], y=q_base_grasp_normalized[1],
                z=q_base_grasp_normalized[2], w=q_base_grasp_normalized[3])
        )
        self.pregrasp_pose.width = data.width

        # Calculate lift-grasp pose[cite: 3]
        self.liftgrasp_pose = GraspCfg()
        self.liftgrasp_pose.pose = geometry_msgs.Pose(
            position=geometry_msgs.Vector3(x=T_base_pregrasp[0, 3], y=T_base_pregrasp[1, 3], z=max(T_base_pregrasp[2, 3] + 0.1, 0.02)), 
            orientation=geometry_msgs.Quaternion(
                x=q_base_grasp_normalized[0], y=q_base_grasp_normalized[1],
                z=q_base_grasp_normalized[2], w=q_base_grasp_normalized[3])
        )
        self.liftgrasp_pose.width = data.width
        
        self.get_logger().info("Received grasp_pose")
        self.get_logger().info("Published grasp_pose transform from 'link_base' to 'grasp_pose'")

    def publish_grasp_pose_frame(self):
        """
        Publish the grasp pose frame (grasp_pose) to the TF tree with 'link_base' as the parent and 'grasp_pose' as the child.[cite: 3]
        """
        if self.publish_grasp_pose is None:
            return
            
        transform = geometry_msgs.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "link_base"  # Parent frame[cite: 3]
        transform.child_frame_id = "grasp_pose"  # Child frame[cite: 3]

        grasp_pose = self.publish_grasp_pose.pose
        transform.transform.translation.x = grasp_pose.position.x
        transform.transform.translation.y = grasp_pose.position.y
        transform.transform.translation.z = grasp_pose.position.z
        transform.transform.rotation = grasp_pose.orientation

        self.br.sendTransform(transform)
        
    # ROS2 Service Call Helpers
    def gripper_request_plan(self, client, target):
        req = joint_plan.Request()
        req.target = target
        response = client.call(req)
        return response.success

    def request_plan(self, client, target):
        req = pose_plan.Request()
        req.target = target
        response = client.call(req)
        return response.success

    def request_exec(self, client):
        req = exec_plan.Request()
        req.execute = True
        response = client.call(req)
        return response.success

    def move_gripper(self, target):
        if self.gripper_request_plan(self.gripper_plan_client, target):
            self.get_logger().info("Gripper Plan SUCCESS! Executing...")
            self.request_exec(self.gripper_exec_client)
            return 0
        return -1

    def move_xArm(self, target):
        if self.request_pose_plan(target):
            self.get_logger().info("Plan SUCCESS! Executing...")
            self.request_exec(self.exec_client)
            return 0
        return -1

    def move_xArm_cartesian(self, target):
        if self.request_cartesian_plan(target):
            self.get_logger().info("Cartesian Plan SUCCESS! Executing...")
            self.request_exec(self.exec_client)
            return 0
        return -1

    def confirm_execution(self, prompt):
        try:
            answer = input(prompt).strip().lower()
        except (EOFError, KeyboardInterrupt):
            self.get_logger().warn("Execution canceled by user.")
            return False

        if answer in ["y", "yes"]:
            return True

        self.get_logger().warn("Execution canceled by user.")
        return False
    
    def move_xArm_interaction(self, target):
        if self.request_pose_plan(target):
            self.get_logger().info("Plan SUCCESS! Please check the trajectory in RViz.")
            if not self.confirm_execution("Execute this plan? [y/N]: "):
                return -1
            self.request_exec(self.exec_client)
            return 0
        return -1

    def move_xArm_cartesian_interaction(self, target):
        if self.request_cartesian_plan(target):
            self.get_logger().info("Cartesian Plan SUCCESS! Please check the trajectory in RViz.")
            if not self.confirm_execution("Execute this Cartesian plan? [y/N]: "):
                return -1
            self.request_exec(self.exec_client)
            return 0
        return -1
    
    def normalize_quaternion(self, q):
        norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if norm == 0:
            self.get_logger().warn("Quaternion norm is zero. Resetting to identity.")
            q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
        else:
            q.x /= norm
            q.y /= norm
            q.z /= norm
            q.w /= norm
        return q

    def request_pose_plan(self, pose):
        pose.orientation = self.normalize_quaternion(pose.orientation)
        req = pose_plan.Request()
        req.target = pose
        
        try:
            response = self.line_client.call(req)
            if not response.success:
                self.get_logger().warn("Pose plan service returned: FAILED")
            return response.success
        except Exception as e:
            self.get_logger().error(f"Failed to call /xarm_pose_plan: {str(e)}")
            return False

    def request_cartesian_plan(self, pose):
        pose.orientation = self.normalize_quaternion(pose.orientation)
        req = single_straight_plan.Request()
        req.target = pose
        
        try:
            response = self.cartesian_client.call(req)
            if not response.success:
                self.get_logger().warn("Cartesian plan service returned: FAILED")
            return response.success
        except Exception as e:
            self.get_logger().error(f"Failed to call /xarm_straight_plan: {str(e)}")
            return False

    def run_grasp(self):
        """Execute grasping logic in a background thread"""
        self.get_logger().info('Waiting for /grasp_start')
        
        while rclpy.ok():
            if (not self.grasp_start) or (self.grasp_pose is None):
                time.sleep(0.1) # 替代 rospy.Rate(10.0).sleep()[cite: 3]
                continue
                
            self.get_logger().info(f'================= Start No.{self.grasp_num} grasp =================')
            
            # Move to pre-grasp pose[cite: 3]
            self.get_logger().info('step 1: move to pre-grasp pose')
            self.move_xArm_cartesian_interaction(self.pregrasp_pose.pose)
            
            # Move to grasp pose[cite: 3]
            self.get_logger().info('step 2: move to grasp pose')
            self.move_xArm_cartesian_interaction(self.grasp_pose.pose)
            
            # Close the gripper[cite: 3]
            self.get_logger().info('step 3: close gripper')
            self.move_gripper(self.gripper_close_width)
            
            # Lift the arm[cite: 3]
            self.get_logger().info('step 4: lift arm')
            self.move_xArm_cartesian_interaction(self.liftgrasp_pose.pose)
            
            # Move to above the storage box[cite: 3]
            self.get_logger().info('step 5: move to place pose')
            self.move_xArm_cartesian_interaction(self.place_pose)
            
            # Open the gripper[cite: 3]
            self.get_logger().info('step 6: open gripper')
            self.move_gripper(self.gripper_open_width)
            
            # Move to watch position[cite: 3]
            self.get_logger().info('step 7: move to watch pose')
            self.move_xArm_cartesian_interaction(self.watch_pose)
            
            self.get_logger().info(f'================= End No.{self.grasp_num} grasp =================')
            self.grasp_num += 1
            self.grasp_pose = None
            
            # Trigger next detection
            msg = Int8()
            msg.data = 1
            self.start_detection_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        policy = PolicyXArm7()
        # spin() 会接管主线程，用于处理 TF 回调、话题接收等
        # 而我们的抓取控制流和 input() 交互运行在刚才创建的后台线程中
        rclpy.spin(policy)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
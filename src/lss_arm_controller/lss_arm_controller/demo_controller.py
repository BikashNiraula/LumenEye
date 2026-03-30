#!/usr/bin/env python3
"""
Demo Controller for LSS 4DOF Arm
Runs a simple demonstration sequence.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time


class DemoController(Node):
    def __init__(self):
        super().__init__('demo_controller')
        
        # Joint names
        self.arm_joints = [
            'lss_arm_joint_1',
            'lss_arm_joint_2',
            'lss_arm_joint_3',
            'lss_arm_joint_4'
        ]
        
        self.gripper_joints = [
            'lss_arm_joint_5',
            'lss_arm_joint_6'
        ]
        
        # Action clients
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        
        # Subscribe to joint states
        self.joint_states = {}
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('Demo Controller initialized!')
        self.get_logger().info('Waiting for action servers...')
        
    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = msg.position[i]
    
    def send_arm_goal(self, positions, duration_sec=2.0):
        """Send arm to target positions."""
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Arm action server not available!')
            return False
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 4
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        trajectory.points.append(point)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f'Moving arm to: {positions}')
        
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected!')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info('Arm movement complete!')
        return True
    
    def send_gripper_goal(self, position, duration_sec=1.0):
        """Send gripper to target position. 0=closed, 0.6=open"""
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper action server not available!')
            return False
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [position, -position]  # Mimic joint
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        trajectory.points.append(point)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f'Moving gripper to: {position}')
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info('Gripper movement complete!')
        return True
    
    def run_demo(self):
        """Run demonstration sequence."""
        self.get_logger().info('='*50)
        self.get_logger().info('Starting Demo Sequence')
        self.get_logger().info('='*50)
        
        # Wait for controllers to be ready
        time.sleep(2.0)
        
        # Position 1: Home position
        self.get_logger().info('\n[1/6] Moving to HOME position...')
        self.send_arm_goal([0.0, 0.0, 0.0, 0.0], duration_sec=3.0)
        time.sleep(0.5)
        
        # Position 2: Open gripper
        self.get_logger().info('\n[2/6] Opening gripper...')
        self.send_gripper_goal(0.5, duration_sec=1.0)
        time.sleep(0.5)
        
        # Position 3: Move to pick position
        self.get_logger().info('\n[3/6] Moving to PICK position...')
        self.send_arm_goal([0.5, 0.8, -0.5, -0.3], duration_sec=2.0)
        time.sleep(0.5)
        
        # Position 4: Close gripper
        self.get_logger().info('\n[4/6] Closing gripper...')
        self.send_gripper_goal(0.1, duration_sec=1.0)
        time.sleep(0.5)
        
        # Position 5: Move to place position
        self.get_logger().info('\n[5/6] Moving to PLACE position...')
        self.send_arm_goal([-0.5, 0.6, -0.4, 0.2], duration_sec=2.0)
        time.sleep(0.5)
        
        # Position 6: Open gripper
        self.get_logger().info('\n[6/6] Opening gripper...')
        self.send_gripper_goal(0.5, duration_sec=1.0)
        time.sleep(0.5)
        
        # Return home
        self.get_logger().info('\n[DONE] Returning to HOME...')
        self.send_arm_goal([0.0, 0.0, 0.0, 0.0], duration_sec=2.0)
        
        self.get_logger().info('='*50)
        self.get_logger().info('Demo Sequence Complete!')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    
    controller = DemoController()
    
    try:
        controller.run_demo()
        
        # Keep node alive
        controller.get_logger().info('\nNode running. Press Ctrl+C to exit.')
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

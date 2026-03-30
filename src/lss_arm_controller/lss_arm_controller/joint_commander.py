#!/usr/bin/env python3
"""
Interactive Joint Commander for LSS 4DOF Arm
Send joint commands from terminal.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        
        self.arm_joints = [
            'lss_arm_joint_1',
            'lss_arm_joint_2',
            'lss_arm_joint_3',
            'lss_arm_joint_4'
        ]
        
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Joint Commander Ready!')
        self.print_help()
    
    def print_help(self):
        print('\n' + '='*60)
        print('LSS 4DOF Arm - Interactive Joint Commander')
        print('='*60)
        print('Commands:')
        print('  move j1 j2 j3 j4  - Move to positions (radians)')
        print('  home              - Go to home position [0,0,0,0]')
        print('  up                - Arm pointing up')
        print('  forward           - Arm pointing forward')
        print('  help              - Show this help')
        print('  quit              - Exit')
        print('')
        print('Joint Limits (radians):')
        print('  J1 (base):     [-3.14, 3.14]')
        print('  J2 (shoulder): [-2.18, 2.18]')
        print('  J3 (elbow):    [-1.92, 1.57]')
        print('  J4 (wrist):    [-2.00, 2.00]')
        print('='*60 + '\n')
    
    def send_positions(self, positions, duration=2.0):
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 4
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )
        trajectory.points.append(point)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        print(f'Sending: {positions}')
        
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().accepted:
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            print('Movement complete!')
            return True
        else:
            print('Goal rejected!')
            return False
    
    def run(self):
        while rclpy.ok():
            try:
                cmd = input('>>> ').strip().lower()
                
                if not cmd:
                    continue
                
                parts = cmd.split()
                
                if parts[0] == 'quit' or parts[0] == 'exit':
                    break
                elif parts[0] == 'help':
                    self.print_help()
                elif parts[0] == 'home':
                    self.send_positions([0.0, 0.0, 0.0, 0.0])
                elif parts[0] == 'up':
                    self.send_positions([0.0, 1.57, 0.0, 0.0])
                elif parts[0] == 'forward':
                    self.send_positions([0.0, 0.0, -1.57, 0.0])
                elif parts[0] == 'move':
                    if len(parts) != 5:
                        print('Usage: move j1 j2 j3 j4')
                        continue
                    try:
                        positions = [float(p) for p in parts[1:5]]
                        self.send_positions(positions)
                    except ValueError:
                        print('Invalid position values!')
                else:
                    print(f'Unknown command: {parts[0]}')
                    print('Type "help" for available commands.')
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        print('Goodbye!')


def main(args=None):
    rclpy.init(args=args)
    commander = JointCommander()
    
    try:
        commander.run()
    finally:
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

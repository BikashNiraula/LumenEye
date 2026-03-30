#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('lss_arm_description')
    urdf_file  = os.path.join(pkg_description, 'urdf', 'lss_arm_4dof.urdf')
    rviz_config = os.path.join(pkg_description, 'rviz', 'view_robot.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # 1. Publishes TF transforms from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }]
        ),

        # 2. Executes joint trajectories and publishes /joint_states
        Node(
            package='lss_arm_controller',
            executable='fake_joint_executor',
            name='fake_joint_executor',
            output='screen',
        ),

        # 3. Moves the blocking box, publishes /lumeneye/box_pose
        Node(
            package='lss_arm_controller',
            executable='box_mover',
            name='box_mover',
            output='screen',
        ),

        # 4. Perception: reads joint_states + box_pose, publishes markers
        Node(
            package='lss_arm_controller',
            executable='perception_node',
            name='perception_node',
            output='screen',
        ),

        # 5. Lamp adjuster: moves arm to keep beam clear
        Node(
            package='lss_arm_controller',
            executable='lamp_adjuster',
            name='lamp_adjuster',
            output='screen',
        ),

        # 6. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])

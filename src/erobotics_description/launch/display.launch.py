import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    # 1. Path to your package and URDF
    pkg_share = get_package_share_directory('erobotics_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mycobot_280_arduino_adaptive_gripper.urdf')

    # 2. Read the URDF file content
    # Using ParameterValue helps Jazzy handle the string correctly
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': False # Set to True if using Gazebo/Isaac Sim
            }]
        ),

        # Joint State Publisher GUI (to move the MyCobot joints)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # This env var fix handles the Snap error we discussed earlier
            env={
                'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:' + os.environ.get('LD_LIBRARY_PATH', '')
            } 
        )
    ])
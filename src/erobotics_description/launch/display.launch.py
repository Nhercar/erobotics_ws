import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 1. Path to your package and URDF
    pkg_share = get_package_share_directory('erobotics_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mycobot_280_arduino.urdf.xacro')

    use_gripper_arg = DeclareLaunchArgument(
        'use_gripper',
        default_value='true',
        description='Indica si se debe cargar la pinza (true/false)'
    )

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value= urdf_file,
        description="Absolute path to the robot urdf file"
    )

    robot_description_content = ParameterValue(Command([
        "xacro ", LaunchConfiguration("model"),
        ' use_gripper:=', LaunchConfiguration("use_gripper")])
        , value_type=str)

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
            }]
        )
    
    joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        )
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", os.path.join(get_package_share_directory("erobotics_description"), "rviz", "display.rviz")]
        )
    
    
    
    
    
    
    return LaunchDescription([
        use_gripper_arg,
        model_arg,
        # Robot State Publisher
        robot_state_publisher,
        # Joint State Publisher GUI (to move the MyCobot joints)
        joint_state_publisher,      
        # RViz2
        rviz_node       
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from  launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")


    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(
                get_package_share_directory("erobotics_description"),
                "urdf", 
                "mycobot_280_arduino.urdf.xacro"
            )
        ]),
        value_type=str,
    )

        # 2. Argumentos de lanzamiento
    use_gripper_arg = DeclareLaunchArgument(
        'use_gripper',
        default_value='true',
        description='Indica si se debe cargar la pinza (true/false)'
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        condition=UnlessCondition(is_sim),
        parameters=[{"robot_description": robot_description}],
        output="screen",            
    )

        # 8. Controladores (Spawners de ros2_control)
    # Primero el broadcaster de estados de articulaciones
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Controlador del brazo
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    # Controlador del gripper (se lanza solo si use_gripper es true)
    # Usamos una técnica de "delay" para esperar a que el robot aparezca
    gripper_controller_spawner = Node(
        condition=IfCondition(LaunchConfiguration('use_gripper')),
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )  


    return LaunchDescription([
        is_sim_arg,
        use_gripper_arg,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner    
    ])
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Rutas y directorios
    pkg_description = get_package_share_directory('erobotics_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(pkg_description, 'worlds', 'my_world.sdf')


    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(pkg_description, "urdf", "mycobot_280_arduino.urdf.xacro"),
        description="Absolute path to the robot xacro file"
    )

    # set GZ_SIM_RESOURCE_PATH to the workspace parent (string, not list)
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(Path(pkg_description).parent.resolve())
    )

    # 2. Argumentos de lanzamiento
    use_gripper_arg = DeclareLaunchArgument(
        'use_gripper',
        default_value='true',
        description='Indica si se debe cargar la pinza (true/false)'
    )

    declare_use_camera = DeclareLaunchArgument(
    'use_camera',
    default_value='false',
    description='Añadir cámara RGB al robot'
    )

    # 3. Procesar Xacro (Robot Description)
    robot_description = Command([
        'xacro ', LaunchConfiguration("model"), 
        ' use_gripper:=', LaunchConfiguration('use_gripper'), 
        ' use_camera:=', LaunchConfiguration('use_camera')
    ])

    # 4. Nodo Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 5. Iniciar Gazebo Sim (Mundo vacío por defecto)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
       launch_arguments={
            'gz_args': f'-r {world_file}' # '-r' sirve para que la simulación empiece a correr al abrirse
        }.items()
    )

    # 6. Aparecer (Spawn) el robot en Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'erobotics',
            '-z', '0.03'
        ]
    )

    # 7. Bridge (Puente de comunicación entre ROS 2 y Gazebo)
    # Necesario para el reloj (/clock) y estados
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            '/robot/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/robot/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ]
    )

    # 8. Controladores (Spawners de ros2_control)
    # Primero el broadcaster de estados de articulaciones
    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    # )

    # Controlador del brazo (debe coincidir con el nombre en tu archivo YAML/Xacro)
    # arm_controller_spawner = Node(robot_state_publisher
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['arm_controller'],
    # )

    # Controlador del gripper (se lanza solo si use_gripper es true)
    # Usamos una técnica de "delay" para esperar a que el robot aparezca
    # gripper_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['gripper_controller'],
    #     condition=None # Podrías añadir lógica aquí si fuera necesario
    # )

    return LaunchDescription([
        model_arg,
        use_gripper_arg,
        declare_use_camera,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge
    ])
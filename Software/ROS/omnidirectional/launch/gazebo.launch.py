from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'omnidirectional'

    # Rutas de archivos
    config_file = '/home/vboxuser/Escritorio/omnidirectional/config/controller_manager.yaml'
    config_file_ekf = '/home/vboxuser/Escritorio/omnidirectional/config/ekf.yaml'
    xacro_path = ' /home/vboxuser/Escritorio/omnidirectional/urdf/main.xacro'
    
    # Publicador de estado del robot
    state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(['xacro', xacro_path])
        }]
    )

    # Lanzar Gazebo manualmente
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # Spawnear robot desde el topic /robot_description
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'omni_robot', '-topic', 'robot_description', '-z', '0.03'],
        output='screen'
    )

    # Nodo ros2_control con robot_description y configuración
    controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            "robot_description": Command(['xacro', xacro_path])
        }, config_file],
        output='screen'
    )

    # Spawner de joint_state_broadcaster
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
        )
        
    # Spawner de velocity_controller
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller'],
        output='screen'
    )
    
    # Nodo de cinemática inversa
    cin_inv = Node(
        package='omnidirectional',
        executable='inversa',
        name='cinematica_inversa',
        output='screen'
    )

    odometria_gt = Node(
        package = "omnidirectional",
		executable = "odom_gt",
		name = "odometria",
        parameters=[{'use_sim_time': True}],
		output='screen'
	)
    
    odometria_gz = Node(
        package = "omnidirectional",
		executable = "odom_encoder_gz",
		name = "odometria",
        parameters=[{'use_sim_time': True}],
		output='screen'
	)

    teleop_key = Node(
        package='teleop_twist_keyboard',
		executable='teleop_twist_keyboard',
		name='teleop_keyboard',
		output='screen',
		prefix='xterm -e'
	)

    rviz = ExecuteProcess(
		cmd=['rviz2', '-d', '/home/vboxuser/Escritorio/omnidirectional/rviz/urdf.rviz'],
		output = 'screen'
	)

    sensor_fusion = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file_ekf]
    )

    return LaunchDescription([
        gazebo,
        state_pub,
        spawn_entity,
        controller_node,
        joint_state_spawner,
        velocity_controller_spawner,
        cin_inv,
        odometria_gz,
        odometria_gt,
        sensor_fusion,
        teleop_key,
        rviz
    ])

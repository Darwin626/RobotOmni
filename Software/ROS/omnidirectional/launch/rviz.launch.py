from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
	
	
    xacro_path = ' /home/vboxuser/Escritorio/omnidirectional/urdf/main.xacro'
    config_file_ekf = '/home/vboxuser/Escritorio/omnidirectional/config/ekf.yaml'
    
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
    
    rviz = ExecuteProcess(
		cmd=['rviz2', '-d', '/home/vboxuser/Escritorio/omnidirectional/rviz/urdf.rviz'],
		output = 'screen'
	)
    
    cin_inv = Node(
		package = "omnidirectional",
		executable = "inversa",
		name = "cinematica_inversa",
		output='screen'
	)

    odometria_sim = Node(
		package = "omnidirectional",
		executable = "odome",
		name = "odometria",
		output='screen'
	)

    teleop_key = Node(
		package='teleop_twist_keyboard',
		executable='teleop_twist_keyboard',
		name='teleop_keyboard',
		output='screen',
		prefix='xterm -e'
	)
    
    sensor_fusion = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file_ekf]
	)
    
    return LaunchDescription([
		state_pub,
		rviz,
		cin_inv,
		odometria_sim,
		teleop_key,
        sensor_fusion
		])

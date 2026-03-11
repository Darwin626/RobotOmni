from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
	
    sensor_read = Node(
		package = "omnidirectional",
		executable = "sensor",
		name = "Sensor_ser_read",
		output='screen'
	)
    
    lidar = Node(
        name="replidar_composition",
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_link',
            'inverted': False,
            'angle_compensate': True
		}]
	)
        	
    return LaunchDescription([
        lidar,
        sensor_read
		])

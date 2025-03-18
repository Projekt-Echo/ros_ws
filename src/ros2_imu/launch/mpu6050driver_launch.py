import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	ld = LaunchDescription()
	share_dir = get_package_share_directory('mpu6050driver')
	parameter_file = LaunchConfiguration('params_file')

	params_declare = DeclareLaunchArgument('params_file',
	                                       default_value=os.path.join(
		                                       share_dir, 'params', 'mpu6050.yaml'),
	                                       description='Path to the ROS2 parameters file to use.')

	mpu6050driver_node = Node(
		package='mpu6050driver',
		executable='mpu6050driver',
		name='mpu6050driver_node',
		output="screen",
		emulate_tty=True,
		parameters=[parameter_file]
	)

	filter_node = Node(
		package='imu_complementary_filter',
		node_executable='complementary_filter_node',
		name='complementary_filter_gain_node',
		output='screen',
		parameters=[
			{'do_bias_estimation': True},
			{'do_adaptive_gain': True},
			{'use_mag': False},
			{'gain_acc': 0.01},
			{'gain_mag': 0.01},
		],
	)

	ld.add_action(params_declare)
	ld.add_action(mpu6050driver_node)
	ld.add_action(filter_node)
	return ld

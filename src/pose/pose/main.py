import math

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from serial import Serial


class PoseNode(Node):
	def __init__(self):
		# Initialize the ROS2 node
		super().__init__('pose_node')

		# Initialize the serial port
		self.serial = Serial(
			port='/dev/ttyAMA2',
			baudrate=38400,
			bytesize=8,
			parity='N',
			stopbits=1,
			timeout=1
		)

		# Subscribe to the Lidar topics
		self.laser_subscriber = self.create_subscription(
			LaserScan,
			'/scan',
			self.ladar_callback,
			10
		)

		# Subscribe to the IMU topic
		self.imu_angle_subscriber = self.create_subscription(
			Vector3,
			'/imu/angles',
			self.angles_callback,
			10
		)

		# Add class variables to store distances
		self.distance_0 = 0.0  # 0 degrees
		self.distance_90 = 0.0  # 90 degrees
		self.distance_180 = 0.0  # 180 degrees (left side)
		self.distance_270 = 0.0  # 270 degrees (back side)

		self.last_valid_x = 0.0
		self.last_valid_y = 0.0

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		# Publisher
		# Create a publisher
		self.timer = self.create_timer(0.1, self.timer_callback)  # Message Frequency: 10Hz

	def timer_callback(self):
		pose = self.pose_msg_builder(self.pitch, self.roll, self.yaw, self.last_valid_x, self.last_valid_y)
		self.pose_sender(pose)

	def angles_callback(self, msg):
		"""
		Callback function for IMU angles
		:param msg: IMU angles message
		:return:
		"""
		# Extract angles from the message
		self.roll = msg.x
		self.pitch = msg.y
		self.yaw = msg.z

	# # Most important: Yaw Axis
	# angles_data = f'Angles: {self.roll:.2f},{self.pitch:.2f},{self.yaw:.2f}\n'
	# self.serial.write(angles_data.encode('utf-8'))  # Send angles to STM32
	# self.get_logger().info(angles_data)

	def ladar_callback(self, msg):
		"""
		Callback function for Lidar messages
		:param msg: Lidar message
		:return:
		"""
		# Calculate how many points are in the scan
		num_points = len(msg.ranges)

		# Define angles and corresponding attribute names
		directions = {
			"distance_0": self.distance_0,
			"distance_90": self.distance_90,
			"distance_180": self.distance_180,
			"distance_270": self.distance_270,
		}

		# Calculate the index and obtain the safe distance
		for attr, angle in directions.items():
			idx = int((angle - msg.angle_min) / msg.angle_increment) % num_points
			distance = msg.ranges[idx]
			setattr(self, attr, distance if not (math.isnan(distance) or distance == float('inf')) else 0)

	# self.get_logger().info(
	# 	f'Distances - 0°: {self.distance_0:.2f}, 90°: {self.distance_90:.2f}, 180°: {self.distance_180:.2f}, 270°: {self.distance_270:.2f}')

	def pose_msg_builder(self, pitch, roll, yaw, x_coord, y_coord):
		"""
		Build the message to be sent
		:param pitch: Pitch angle
		:param roll: Roll angle
		:param yaw: Yaw angle
		:param x_coord: x_coord
		:param y_coord: y_coord
		:return: Formatted message string
		"""
		# Initialize last valid coordinates if not already set
		if not hasattr(self, 'last_valid_x'):
			self.last_valid_x = 0
		if not hasattr(self, 'last_valid_y'):
			self.last_valid_y = 0

		# Safety check for coordinates
		if math.isnan(x_coord) or math.isnan(y_coord):
			self.get_logger().warn('NaN values detected in coordinate data, using previous values')
			x_coord = self.last_valid_x
			y_coord = self.last_valid_y
		else:
			# Transform coordinates to centimeters and limit to 4 digits
			x_coord = min(int(x_coord * 100), 9999)  # Transform to centimeters and limit to 4 digits
			y_coord = min(int(y_coord * 100), 9999)  # Transform to centimeters and limit to 4 digits
			# Store the last valid coordinates
			self.last_valid_x = x_coord
			self.last_valid_y = y_coord

		# Safety check for angles
		if math.isnan(pitch):
			pitch = 0.0
		if math.isnan(roll):
			roll = 0.0
		if math.isnan(yaw):
			yaw = 0.0

		# Format angles as: sign + 3 digit integer + decimal point + 2 decimals
		pitch_str = f"{'+' if pitch >= 0 else '-'}{int(abs(pitch)):03d}.{int(abs(pitch * 100) % 100):02d}"
		roll_str = f"{'+' if roll >= 0 else '-'}{int(abs(roll)):03d}.{int(abs(roll * 100) % 100):02d}"
		yaw_str = f"{'+' if yaw >= 0 else '-'}{int(abs(yaw)):03d}.{int(abs(yaw * 100) % 100):02d}"

		# Construct the formatted message string
		data = f"@:{pitch_str},{roll_str},{yaw_str} #:{x_coord:04d},{y_coord:04d}\n"

		return data

	def pose_sender(self, data):
		"""
		Send data to the serial port
		:param data: Data to be sent
		"""
		# Send data to the serial port
		self.serial.write(data.encode('utf-8'))
		# Log the sent data
		self.get_logger().info(f'Sent data: {data.strip()}')



def main():
	rclpy.init()
	node = PoseNode()
	rclpy.spin(node)
	node.serial.close()
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

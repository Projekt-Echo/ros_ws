import math

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from serial import Serial


class PositionNode(Node):
	def __init__(self):
		# Initialize the ROS2 node
		super().__init__('position_node')

		# Initialize the serial port
		self.serial = Serial(
			port='/dev/ttyAMA2',
			baudrate=115200,
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

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		# Publisher
		# Create a publisher
		self.timer = self.create_timer(1.0, self.send_data)  # 每秒发送一次数据

	def angles_callback(self, msg):
		# Extract angles from the message
		self.roll = msg.x
		self.pitch = msg.y
		self.yaw = msg.z

		# Most important: Yaw Axis
		angles_data = f'Angles - Roll: {self.roll:.2f}, Pitch: {self.pitch:.2f}, Yaw: {self.yaw:.2f}\n'
		self.get_logger().info(angles_data)

	def ladar_callback(self, msg):
		self.get_logger().info('Receiver Lidar Message')

		# Calculate how many points are in the scan
		num_points = len(msg.ranges)

		# Calculate indices for the specific angles
		idx_0 = int((0 - msg.angle_min) / msg.angle_increment) % num_points
		idx_90 = int((1.57 - msg.angle_min) / msg.angle_increment) % num_points  # 90° in radians
		idx_180 = int((3.14 - msg.angle_min) / msg.angle_increment) % num_points  # 180° in radians
		idx_270 = int((4.71 - msg.angle_min) / msg.angle_increment) % num_points  # 270° in radians

		# Get distances (with improved safety checks for both inf and NaN)
		self.distance_0 = msg.ranges[idx_0] if not (
				math.isnan(msg.ranges[idx_0]) or msg.ranges[idx_0] == float('inf')) else 0
		self.distance_90 = msg.ranges[idx_90] if not (
				math.isnan(msg.ranges[idx_90]) or msg.ranges[idx_90] == float('inf')) else 0
		self.distance_180 = msg.ranges[idx_180] if not (
				math.isnan(msg.ranges[idx_180]) or msg.ranges[idx_180] == float('inf')) else 0
		self.distance_270 = msg.ranges[idx_270] if not (
				math.isnan(msg.ranges[idx_270]) or msg.ranges[idx_270] == float('inf')) else 0

		self.get_logger().info(
			f'Distances - 0°: {self.distance_0:.2f}, 90°: {self.distance_90:.2f}, 180°: {self.distance_180:.2f}, 270°: {self.distance_270:.2f}')

	def send_data(self):
		# Store previous valid coordinates as class variables if they don't exist
		if not hasattr(self, 'last_valid_x'):
			self.last_valid_x = 0
		if not hasattr(self, 'last_valid_y'):
			self.last_valid_y = 0

		# Add safety checks before converting to int
		if math.isnan(self.distance_180) or math.isnan(self.distance_270):
			self.get_logger().warn('NaN values detected in distance data, using previous values')
			x_coord = self.last_valid_x
			y_coord = self.last_valid_y
		else:
			x_coord = min(int(self.distance_180 * 100), 65535)  # 180° distance as X coordinate
			y_coord = min(int(self.distance_270 * 100), 65535)  # 270° distance as Y coordinate
			# Save these as the last valid coordinates
			self.last_valid_x = x_coord
			self.last_valid_y = y_coord

		# Send position data to stm32
		data = [
			(x_coord >> 8) & 0xFF,  # High 8 bits of X coordinate
			y_coord & 0xFF,  # 8 bits of Y coordinate
		]
# self.serial.write(data)


def main():
	rclpy.init()
	node = PositionNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

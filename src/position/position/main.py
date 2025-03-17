import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
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
		self.imu_subscriber = self.create_subscription(
			Imu,
			'/imu/mpu6050',
			self.imu_callback,
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

	def ladar_callback(self, msg):
		self.get_logger().info('Receiver Lidar Message')

		# Calculate how many points are in the scan
		num_points = len(msg.ranges)

		# Calculate indices for the specific angles
		# LaserScan ranges typically start at angle_min and increment by angle_increment
		idx_0 = int((0 - msg.angle_min) / msg.angle_increment) % num_points
		idx_90 = int((1.57 - msg.angle_min) / msg.angle_increment) % num_points  # 90° in radians
		idx_180 = int((3.14 - msg.angle_min) / msg.angle_increment) % num_points  # 180° in radians
		idx_270 = int((4.71 - msg.angle_min) / msg.angle_increment) % num_points  # 270° in radians

		# Get distances (with safety checks)
		self.distance_0 = msg.ranges[idx_0] if not float('inf') in [msg.ranges[idx_0]] else 0
		self.distance_90 = msg.ranges[idx_90] if not float('inf') in [msg.ranges[idx_90]] else 0
		self.distance_180 = msg.ranges[idx_180] if not float('inf') in [msg.ranges[idx_180]] else 0
		self.distance_270 = msg.ranges[idx_270] if not float('inf') in [msg.ranges[idx_270]] else 0

		self.get_logger().info(
			f'Distances - 0°: {self.distance_0:.2f}, 90°: {self.distance_90:.2f}, 180°: {self.distance_180:.2f}, 270°: {self.distance_270:.2f}')


	def imu_callback(self, msg):
		self.get_logger().info('Receiver Imu Message')


	def send_data(self):
		# Convert distances to cm and limit them to 65535
		x_coord = min(int(self.distance_180 * 100), 65535)  # 180° distance as X coordinate
		y_coord = min(int(self.distance_270 * 100), 65535)  # 270° distance as Y coordinate

		# Send position data to stm32
		data = [
			0x55, 0x00,  # Feature header (2 bytes)
			0x00, 0x00,  # Status code (2 bytes)
			x_coord & 0xFF, (x_coord >> 8) & 0xFF,  # X coordinate (2 bytes) - 180 degrees
			y_coord & 0xFF, (y_coord >> 8) & 0xFF,  # Y coordinate (2 bytes) - 270 degrees
			0x00, 0x00,  # Target X coordinate (2 bytes)
			0x00, 0x00,  # Target Y coordinate (2 bytes)
			0x00, 0x00,  # Reserved (2 bytes)
			0x00, 0x00   # Reserved (2 bytes)
		]

		bytes_data = bytearray(data)

		self.serial.write(bytes_data)



def main():
	rclpy.init()
	node = PositionNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

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

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		# Publisher
		# Create a publisher
		self.timer = self.create_timer(1.0, self.send_data)  # 每秒发送一次数据

	def ladar_callback(self, msg):
		self.get_logger().info('Receiver Lidar Message')
		self.serial.write(str("Test").encode('utf-8'))
		pass

	def imu_callback(self, msg):
		self.get_logger().info('Receiver Imu Message')
		pass

	def send_data(self):
		# Send data to the serial port
		self.serial.write(str("Test").encode('utf-8'))
		self.get_logger().info('Send a message to the serial port')

def main():
	rclpy.init()
	node = PositionNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

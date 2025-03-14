import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from serial import Serial


class PositionNode(Node):
	def __init__(self):
		# 初始化ROS2 Node
		super().__init__('position_node')

		# 初始化下位机通讯UART
		self.serial = Serial(
			port='/dev/ttyAMA0',
			baudrate=115200,
			bytesize=8,
			parity='N',
			stopbits=1,
			timeout=1
		)

		# 激光雷达订阅
		self.laser_subscriber = self.create_subscription(
			LaserScan,
			'/scan',
			self.ladar_callback,
			10
		)

		# IMU订阅
		self.imu_subscriber = self.create_subscription(
			Imu,
			'/imu/mpu6050',
			self.imu_callback,
			10
		)

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

	# 发布器

	# 创建定时器用于发送数据
	# self.timer = self.create_timer()

	def ladar_callback(self, msg):
		self.get_logger().info('Receiver Lidar Message')
		self.serial.write(str("Test").encode('utf-8'))
		pass

	def imu_callback(self, msg):
		self.get_logger().info('Receiver Imu Message')
		pass


def main():
	rclpy.init()
	node = PositionNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

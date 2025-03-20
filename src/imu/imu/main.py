#!/usr/bin/env python3
import rclpy
import smbus
from rclpy.node import Node
from sensor_msgs.msg import Imu


class JY61PDriver(Node):
	def __init__(self):
		super().__init__('imu')
		self.bus = smbus.SMBus(1)  # 使用I2C总线1
		self.address = 0x50  # JY61P的I2C地址
		self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
		self.timer = self.create_timer(0.1, self.timer_callback)  # 每0.1秒读取一次数据
		self.declare_parameter('frame_id', 'jy61p_link')  # 声明参数，用于设置IMU数据的参考系
		self.frame_id = self.get_parameter('frame_id').value

	def timer_callback(self):
		try:
			# 读取加速度数据
			ax = self.read_word_2c(0x34)
			ay = self.read_word_2c(0x35)
			az = self.read_word_2c(0x36)

			# 读取角速度数据
			gx = self.read_word_2c(0x37)
			gy = self.read_word_2c(0x38)
			gz = self.read_word_2c(0x39)

			# 读取四元数数据
			quaternion = self.read_quaternion()

			# 读取温度数据
			temperature = self.read_word_2c(0x40)

			# 转换为物理量
			ax = ax / 32768.0 * 16 * 9.8  # 转换为m/s²
			ay = ay / 32768.0 * 16 * 9.8  # 转换为m/s²
			az = az / 32768.0 * 16 * 9.8  # 转换为m/s²
			gx = gx / 32768.0 * 2000  # 转换为°/s
			gy = gy / 32768.0 * 2000  # 转换为°/s
			gz = gz / 32768.0 * 2000  # 转换为°/s
			temperature = temperature / 100.0  # 转换为摄氏度

			# 创建并发布IMU消息
			imu_msg = Imu()
			imu_msg.header.stamp = self.get_clock().now().to_msg()
			imu_msg.header.frame_id = self.frame_id
			imu_msg.linear_acceleration.x = ax
			imu_msg.linear_acceleration.y = ay
			imu_msg.linear_acceleration.z = az
			imu_msg.angular_velocity.x = gx
			imu_msg.angular_velocity.y = gy
			imu_msg.angular_velocity.z = gz
			imu_msg.orientation.x = quaternion[0]
			imu_msg.orientation.y = quaternion[1]
			imu_msg.orientation.z = quaternion[2]
			imu_msg.orientation.w = quaternion[3]

			self.imu_pub.publish(imu_msg)

		except Exception as e:
			self.get_logger().error('Error reading sensor data: %s' % str(e))

	def read_word_2c(self, addr):
		# 读取两个字节并转换为有符号的短整型
		high = self.bus.read_byte_data(self.address, addr)
		low = self.bus.read_byte_data(self.address, addr + 1)
		val = (high << 8) + low
		if val >= 0x8000:
			return -((65535 - val) + 1)
		else:
			return val

	def read_quaternion(self):
		"""
		从JY61P传感器读取四元数
		:return: 四元数 (x, y, z, w)
		"""
		q0 = self.read_word_2c(0x51)  # 读取四元数 q0
		q1 = self.read_word_2c(0x52)  # 读取四元数 q1
		q2 = self.read_word_2c(0x53)  # 读取四元数 q2
		q3 = self.read_word_2c(0x54)  # 读取四元数 q3

		# 归一化处理
		q0 = q0 / 32768.0
		q1 = q1 / 32768.0
		q2 = q2 / 32768.0
		q3 = q3 / 32768.0

		return [q1, q2, q3, q0]


def main(args=None):
	rclpy.init(args=args)
	imu = JY61PDriver()
	rclpy.spin(imu)
	imu.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

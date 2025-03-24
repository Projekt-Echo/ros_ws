import struct
import time

import rclpy
import smbus2
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from .constant import *


class JY61PDriver(Node):
	def __init__(self):
		super().__init__('imu')
		self.bus = smbus2.SMBus(1)  # 使用I2C总线1
		self.address = 0x50  # JY61P的I2C地址
		self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
		self.angles_pub = self.create_publisher(Vector3, 'imu/angles', 10)
		self.timer = self.create_timer(0.1, self.timer_callback)  # 每0.1秒读取一次数据
		self.declare_parameter('frame_id', 'imu_link')  # 声明参数，用于设置IMU数据的参考系
		self.init_service = self.create_service(Trigger, 'imu/init', self.init_callback)  # 创建服务，用于初始化IMU
		self.frame_id = self.get_parameter('frame_id').value
		self.init_imu(self.bus)  # 初始化IMU传感器

	def init_callback(self, request, response):
		try:
			self.init_imu(self.bus)
			response.success = True
			response.message = 'IMU initialized successfully'
			self.get_logger().info('IMU initialized successfully')
		except:
			response.success = False
			response.message = 'Failed to initialize IMU'
			self.get_logger().error('Failed to initialize IMU')
		return response


	def timer_callback(self):
		try:
			# 读取传感器数据
			roll, pitch, yaw = self.read_angles()
			ax, ay, az = self.read_accel()
			gx, gy, gz = self.read_velocity()
			quaternion = self.read_quaternion()


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

			angles_msg = Vector3()
			angles_msg.x = roll
			angles_msg.y = pitch
			angles_msg.z = yaw

			self.imu_pub.publish(imu_msg)
			self.angles_pub.publish(angles_msg)

		except Exception as e:
			self.get_logger().error('Error reading sensor data: %s' % str(e))

	def read_angles(self):
		"""
		从JY61P传感器读取角度
		:return: 角度 (roll, pitch, yaw)
		"""
		angles_data = self.bus.read_i2c_block_data(self.address, Roll, 6)
		val = struct.unpack("hhh", bytearray(angles_data))

		roll = val[0] / 32768.0 * 180
		pitch = val[1] / 32768.0 * 180
		yaw = val[2] / 32768.0 * 180
		return [roll, pitch, yaw]

	def read_accel(self):
		"""
		从JY61P传感器读取加速度
		:return: 加速度 (ax, ay, az)
		"""
		accel_data = self.bus.read_i2c_block_data(self.address, AX, 6)
		val = struct.unpack("hhh", bytearray(accel_data))

		# 归一化处理
		ax = val[0] / 32768.0 * 16 * 9.8
		ay = val[1] / 32768.0 * 16 * 9.8
		az = val[2] / 32768.0 * 16 * 9.8

		return [ax, ay, az]

	def read_velocity(self):
		"""
		从JY61P传感器读取角速度
		:return: 角速度 (gx, gy, gz)
		"""
		velocity_data = self.bus.read_i2c_block_data(self.address, GX, 6)
		val = struct.unpack("hhh", bytearray(velocity_data))

		# 归一化处理
		gx = val[0] / 32768.0 * 2000
		gy = val[1] / 32768.0 * 2000
		gz = val[2] / 32768.0 * 2000

		return [gx, gy, gz]

	def read_quaternion(self):
		"""
		从JY61P传感器读取四元数
		:return: 四元数 (x, y, z, w)
		"""
		quaternion_data = self.bus.read_i2c_block_data(self.address, q0, 8)
		val = struct.unpack("hhhh", bytearray(quaternion_data))

		# 归一化处理
		q0_value = val[0] / 32768.0
		q1_value = val[1] / 32768.0
		q2_value = val[2] / 32768.0
		q3_value = val[3] / 32768.0

		return [q0_value, q1_value, q2_value, q3_value]

	def init_imu(self, bus):
		"""
		初始化IMU传感器
		"""
		try:
			bus.write_i2c_block_data(self.address, KEY, [0X88, 0XB5])
			time.sleep(0.2)
			bus.write_i2c_block_data(self.address, CALSW, [0X04, 0X00])
			time.sleep(0.2)
			bus.write_i2c_block_data(self.address, SAVE, [0X00, 0X00])
			time.sleep(0.2)
		except Exception as e:
			self.get_logger().error('Error initializing IMU: %s' % str(e))


def main(args=None):
	rclpy.init(args=args)
	imu = JY61PDriver()
	rclpy.spin(imu)
	imu.bus.close()
	imu.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

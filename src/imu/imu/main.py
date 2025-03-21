#!/usr/bin/env python3
import struct

import rclpy
import smbus
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu


class JY61PDriver(Node):
	def __init__(self):
		super().__init__('imu')
		self.bus = smbus.SMBus(1)  # 使用I2C总线1
		self.address = 0x50  # JY61P的I2C地址
		self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
		self.angles_pub = self.create_publisher(Vector3, 'imu/angles', 10)
		self.timer = self.create_timer(0.1, self.timer_callback)  # 每0.1秒读取一次数据
		self.declare_parameter('frame_id', 'imu_link')  # 声明参数，用于设置IMU数据的参考系
		self.frame_id = self.get_parameter('frame_id').value

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

		except Exception as e:
			self.get_logger().error('Error reading sensor data: %s' % str(e))

	def read_angles(self):
		"""
		从JY61P传感器读取角度
		:return: 角度 (roll, pitch, yaw)
		"""
		angles_data = self.bus.read_i2c_block_data(self.address, 0x3D, 6)
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
		accel_data = self.bus.read_i2c_block_data(self.address, 0x34, 6)
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
		velocity_data = self.bus.read_i2c_block_data(self.address, 0x37, 6)
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
		quaternion_data = self.bus.read_i2c_block_data(self.address, 0x51, 8)
		val = struct.unpack("hhhh", bytearray(quaternion_data))

		# 归一化处理
		q0 = val[0] / 32768.0
		q1 = val[1] / 32768.0
		q2 = val[2] / 32768.0
		q3 = val[3] / 32768.0

		return [q1, q2, q3, q0]


def main(args=None):
	rclpy.init(args=args)
	imu = JY61PDriver()
	rclpy.spin(imu)
	imu.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

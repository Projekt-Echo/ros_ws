import math

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.optimize import curve_fit
from sensor_msgs.msg import LaserScan, Imu
from serial import Serial


class PositionDirectNode(Node):
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

		# Add class variables to store fitting results
		self.offset_angles = [0.0, 0.0, 0.0, 0.0]  # 偏移角度
		self.avg_distances = [0.0, 0.0, 0.0, 0.0]  # 平均距离

	def ladar_callback(self, msg):
		self.get_logger().info('Receiver Lidar Message')

		# Calculate how many points are in the scan
		num_points = len(msg.ranges)
		angle_min = msg.angle_min
		angle_increment = msg.angle_increment

		# 定义目标角度（度）和对应的角度范围（前后5度）
		target_angles = [0, 90, 180, 270]
		angle_threshold = np.deg2rad(5)  # 5度范围转换为弧度

		# 存储各目标角度范围内的点
		points = []
		angles_list = []

		for angle in target_angles:
			angle_rad = np.deg2rad(angle)
			# 计算角度范围
			min_angle = angle_rad - angle_threshold
			max_angle = angle_rad + angle_threshold

			# 找到在该角度范围内的索引
			indices = []
			for i in range(num_points):
				current_angle = angle_min + i * angle_increment
				if min_angle <= current_angle <= max_angle:
					indices.append(i)

			# 提取对应的距离和角度数据
			ranges = [msg.ranges[i] for i in indices]
			angles = [angle_min + i * angle_increment for i in indices]

			# 过滤掉无效值
			valid_ranges = []
			valid_angles = []
			for r, a in zip(ranges, angles):
				if not (math.isnan(r) or r == float('inf')):
					valid_ranges.append(r)
					valid_angles.append(a)

			points.append(valid_ranges)
			angles_list.append(valid_angles)

		# 对每个目标角度范围内的点进行拟合
		fit_params_list = []
		offset_angles = []
		avg_distances_to_curve = []

		for i in range(len(target_angles)):
			if len(points[i]) > 0:
				# 使用角度作为x数据，距离作为y数据
				x_data = angles_list[i]
				y_data = points[i]

				# 定义拟合函数（例如直线）
				def linear_func(x, a, b):
					return a * x + b

				# 初始参数猜测
				init_params = [0.0, np.mean(y_data)]

				try:
					# 使用非线性最小二乘法拟合曲线
					params, _ = curve_fit(linear_func, x_data, y_data, p0=init_params)
				except:
					# 如果拟合失败，使用线性回归
					A = np.vstack([x_data, np.ones(len(x_data))]).T
					# 解包所有返回值，只使用解
					params, residuals, rank, singular_values = np.linalg.lstsq(A, y_data, rcond=None)
					params = [params[0], params[1]]

				fit_params_list.append(params)

				# 计算偏移角度（这里以拟合直线的斜率作为偏移角度）
				offset_angle = np.arctan(params[0])
				offset_angles.append(offset_angle)

				# 计算各点到拟合直线的距离
				distances = []
				for x, y in zip(x_data, y_data):
					# 直线方程：y = a*x + b
					a = params[0]
					b = params[1]
					distance = abs(a * x - y + b) / np.sqrt(a ** 2 + 1)
					distances.append(distance)

				# 计算平均距离
				avg_distance = np.mean(distances)
				avg_distances_to_curve.append(avg_distance)

			else:
				fit_params_list.append([0.0, 0.0])
				offset_angles.append(0.0)
				avg_distances_to_curve.append(0.0)

		# 更新类变量存储拟合结果
		self.offset_angles = offset_angles
		self.avg_distances = avg_distances_to_curve

		# 打印结果
		for i, angle in enumerate(target_angles):
			self.get_logger().info(f"目标角度: {angle}度")
			self.get_logger().info(f"拟合参数: 斜率={fit_params_list[i][0]:.4f}, 截距={fit_params_list[i][1]:.4f}")
			self.get_logger().info(f"偏移角度: {np.rad2deg(offset_angles[i]):.4f}度")
			self.get_logger().info(f"点到拟合直线的平均距离: {avg_distances_to_curve[i]:.4f}米\n")

	def imu_callback(self, msg):
		self.get_logger().info('Receiver Imu Message')

	def send_data(self):
		# 构造要发送的数据
		# 这里以偏移角度和平均距离为例，可以根据实际需求选择发送哪些数据
		data_to_send = []
		for i in range(4):
			# 将偏移角度和平均距离转换为字符串，并限制小数位数
			offset_str = f"{np.rad2deg(self.offset_angles[i]):.4f}"
			distance_str = f"{self.avg_distances[i]:.4f}"
			# 将字符串转换为字节
			data_to_send.append(offset_str.encode('utf-8'))
			data_to_send.append(distance_str.encode('utf-8'))

		# 将所有数据连接成一个字节流
		data_stream = b''.join(data_to_send)

		# 通过串口发送数据
		self.serial.write(data_stream)

		self.get_logger().info(f"Sent data: {data_stream}")


def main():
	rclpy.init()
	node = PositionDirectNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
import socket
import numpy as np
import cv2
import pyrealsense2 as rs
from cv_bridge import CvBridge
import signal
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class ArucoPublisher(Node):
	def __init__(self):
		super().__init__('aruco_publisher')
		self.publisher = self.create_publisher(Int64, '/movement/aruco', 10)

		timer_period = 1/30

		signal.signal(signal.SIGALRM, self.timeout_handler)

		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

		self.server_address = ('169.254.167.120', 12345)

		print("Starting UDP Server")

		self.server_socket.bind(self.server_address)

		self.bridge = CvBridge()

		_, self.camera_pipeline = self.initialize_camera()

		self.timer = self.create_timer(timer_period, self.timer_callback)

	def initialize_camera(self):
		# Initializes the camera for all the frames being analyzed
		imu_pipeline = rs.pipeline()
		imu_config = rs.config()
		imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
		imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
		imu_pipeline.start(imu_config)

		camera_pipeline = rs.pipeline()
		camera_config = rs.config()

		camera_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
		camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

		camera_pipeline.start(camera_config)

		return imu_pipeline, camera_pipeline
	
	def detect_aruco_corners(self, image_data):
		image = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)

		aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
		parameters = cv2.aruco.DetectorParameters_create()

		corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

		if ids is not None:
			for i in range(len(ids)):
				cv2.aruco.drawDetectedMarkers(image, corners)

				center = np.mean(corners[i][0], axis=0).astype(int)
				print(center)
				cv2.circle(image, tuple(center), 5, (0, 255, 0), -1)
		else:
			center = [1000, 0]
			print("None")

		image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

		return image, center
	

	def timeout_handler(self, signum, frame):
		raise TimeoutError()

	def timer_callback(self):
		msg = Int64()

		signal.alarm(1)

		frames = self.camera_pipeline.wait_for_frames()

		color_frame = frames.get_color_frame()

		image_data = np.asarray(color_frame.get_data(), dtype=np.uint8)

		result_image, center = self.detect_aruco_corners(image_data)

		msg.data = int(center[0])

		self.publisher.publish(msg)

		compressed_data = self.bridge.cv2_to_compressed_imgmsg(result_image, 'jpg').data

		image_data = np.asarray(compressed_data, dtype=np.uint8)

		split_data = np.array_split(image_data, 3)
		try:
			data, address = self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data[0].tobytes(), address)
			self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data[1].tobytes(), address)
			self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data[2].tobytes(), address)
			self.server_socket.recvfrom(4096)
			#print("Sent")

		except TimeoutError:
			print("Timeout")
			signal.alarm(0)
			return


def main(args=None):
	rclpy.init(args=args)

	aruco_publisher = ArucoPublisher()

	try:
		rclpy.spin(aruco_publisher)
	except KeyboardInterrupt:
		aruco_publisher.server_socket.close()

	aruco_publisher.destroy_node()


if __name__ == '__main__':
	main()
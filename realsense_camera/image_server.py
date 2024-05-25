import rclpy
from rclpy.node import Node

import socket
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
import signal
import time

class ImageServer(Node):

	def __init__(self, pipeline):
		super().__init__('image_publisher')

		timer_period = 1/30
		self.camera_pipeline = pipeline
		signal.signal(signal.SIGALRM, self.timeout_handler)

		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_address = ('192.168.1.11', 12345)
		print("Starting UDP Server")
		self.server_socket.bind(self.server_address)

		self.timer = self.create_timer(timer_period, self.timer_callback)


	def timeout_handler(self, signum, frame):
		raise TimeoutError()

	def timer_callback(self):

		bridge = CvBridge()

		
		signal.alarm(0)

		frames = self.camera_pipeline.wait_for_frames()

		try:
			signal.alarm(1)

			color_frame = frames.get_color_frame()

			image_data = np.asarray(color_frame.get_data(), dtype=np.int32)

			compressed_data = bridge.cv2_to_compressed_imgmsg(image_data, 'jpg').data

			image_data = np.asarray(compressed_data, dtype=np.uint8)

			split_data = np.array_split(image_data, 3)

			data, address = self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data[0].tobytes(), address)
			self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data[1].tobytes(), address)
			self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data[2].tobytes(), address)
			self.server_socket.recvfrom(4096)
			print("Image: Sent")

		except (TimeoutError, OSError):
			print("Image: Timeout")
			self.server_socket.close()
			self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.server_socket.bind(self.server_address)

			signal.alarm(0)
			return

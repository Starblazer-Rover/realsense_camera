import socket
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
import signal
import zlib

import rclpy
from rclpy.node import Node

class DepthServer(Node):

	def __init__(self, pipeline):
		super().__init__('depth_server')
		
		timer_period = 1/30

		self.camera_pipeline = pipeline
		signal.signal(signal.SIGALRM, self.timeout_handler)
		signal.alarm(0)

		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_address = ('192.168.1.11', 12350)

		self.image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.image_address = ('192.168.1.11', 12345)

		print("Starting UDP Server")
		self.server_socket.bind(self.server_address)
		self.image_socket.bind(self.image_address)

		self.timer = self.create_timer(timer_period, self.timer_callback)

	def timeout_handler(self, signum, frame):
		raise TimeoutError()

	def timer_callback(self):

		bridge = CvBridge()

		frames = self.camera_pipeline.wait_for_frames()

		try:
			signal.alarm(1)

			depth_frame = frames.get_depth_frame()
			color_frame = frames.get_color_frame()

			depth_data = np.array(depth_frame.get_data(), dtype=np.uint16)

			split_data = np.array_split(depth_data, 4)
			split_data_0 = zlib.compress(split_data[0].tobytes())
			split_data_1 = zlib.compress(split_data[1].tobytes())
			split_data_2 = zlib.compress(split_data[2].tobytes())
			split_data_3 = zlib.compress(split_data[3].tobytes())
				

			data, address = self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data_0, address)
			self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data_1, address)
			self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data_2, address)
			self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data_3, address)
			self.server_socket.recvfrom(4096)
			print('Depth: Sent')

			image_data = np.asarray(color_frame.get_data(), dtype=np.int32)

			compressed_data = bridge.cv2_to_compressed_imgmsg(image_data, 'jpg').data

			image_data = np.asarray(compressed_data, dtype=np.uint8)

			split_data = np.array_split(image_data, 3)

			data, address = self.image_socket.recvfrom(4096)

			self.image_socket.sendto(split_data[0].tobytes(), address)
			self.image_socket.recvfrom(4096)

			self.image_socket.sendto(split_data[1].tobytes(), address)
			self.image_socket.recvfrom(4096)

			self.image_socket.sendto(split_data[2].tobytes(), address)
			self.image_socket.recvfrom(4096)
			print("Image: Sent")

			signal.alarm(0)

	
		except (TimeoutError, OSError):
			signal.alarm(0)
			print("Timeout")
			self.server_socket.close()
			self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.server_socket.bind(self.server_address)

			self.image_socket.close()
			self.image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.image_socket.bind(self.image_address)
			return


def __initialize_camera():
    # Initializes the camera for all the frames being analyzed
        camera_pipeline = rs.pipeline()
        camera_config = rs.config()

        camera_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        camera_pipeline.start(camera_config)

        return camera_pipeline


def main(args=None):

	rclpy.init(args=args)

	pipeline = __initialize_camera()

	publisher = DepthServer(pipeline)

	try:
		rclpy.spin(publisher)
	except KeyboardInterrupt:
		publisher.destroy_node()

if __name__ == '__main__':
	main()

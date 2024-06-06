import socket
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
import signal
import zlib
from sensor_msgs_py import point_cloud2 as pc2

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16MultiArray


class DepthServer(Node):

	def __init__(self):
		super().__init__('depth_server')

		signal.signal(signal.SIGALRM, self.timeout_handler)
		signal.alarm(0)

		self.subscription = self.create_subscription(UInt16MultiArray, '/depth/depth_raw', self.timer_callback, 1)

		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_address = ('192.168.1.11', 12350)

		print("Starting UDP Server")
		self.server_socket.bind(self.server_address)

	def timeout_handler(self, signum, frame):
		raise TimeoutError()

	def timer_callback(self, msg):

		data = np.array(msg.data, dtype=np.uint16)

		try:
			signal.alarm(1)

			split_data = np.array_split(data, 4)
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

			signal.alarm(0)

	
		except (TimeoutError, OSError):
			signal.alarm(0)
			print("Timeout")
			self.server_socket.close()
			self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.server_socket.bind(self.server_address)

			return


def main(args=None):

	rclpy.init(args=args)

	publisher = DepthServer()

	try:
		rclpy.spin(publisher)
	except KeyboardInterrupt:
		publisher.destroy_node()

if __name__ == '__main__':
	main()

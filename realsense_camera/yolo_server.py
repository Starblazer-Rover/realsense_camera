import socket
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
import signal
import zlib

import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class ImageServer(Node):

	def __init__(self):
		super().__init__('image_server')

		signal.signal(signal.SIGALRM, self.timeout_handler)
		signal.alarm(0)

		self.image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.image_address = ('192.168.1.60', 12345)

		self.bridge = CvBridge()

		weights_path = '/home/billee/yolov9/weights/yolov9-e.pt'

		self.model = YOLO(weights_path)

		self.image_subscriber = self.create_subscription(Image, '/camera/Image_raw', self.timer_callback, 10)

		print("Starting UDP Server")
		self.image_socket.bind(self.image_address)

	def timeout_handler(self, signum, frame):
		raise TimeoutError()
	
	def predict_and_detect(self, img, rectangle_thickness=2, text_thickness=1):
		results = self.model.predict(img)

		print('got_results')

		for result in results:
			for box in result.boxes:
				cv2.rectangle(img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
						(int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0), rectangle_thickness)
				
				cv2.putText(img, f"{result.names[int(box.cls[0])]}",
                        (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), text_thickness)
				
		return img, results
	
	def timer_callback(self, msg):

		data = self.bridge.imgmsg_to_cv2(msg)

		data = self.predict_and_detect(data)

		try:
			signal.alarm(0)

			image_data = np.asarray(data, dtype=np.int32)

			compressed_data = self.bridge.cv2_to_compressed_imgmsg(image_data, 'jpg').data

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

			self.image_socket.close()
			self.image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.image_socket.bind(self.image_address)
			return


def main(args=None):

	rclpy.init(args=args)

	publisher = ImageServer()

	try:
		rclpy.spin(publisher)
	except KeyboardInterrupt:
		publisher.destroy_node()

if __name__ == '__main__':
	main()

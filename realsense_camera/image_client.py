import socket
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class ImagePublisher(Node):

        def __init__(self):
                super().__init__('image_publisher')
                self.publisher = self.create_publisher(Image, '/camera/raw_image', 1)

                self.bridge = CvBridge()

                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

                self.server_address = ('localhost', 12345)

                timer_period = 1/60
                self.timer = self.create_timer(timer_period, self.timer_callback)

                message = np.array([1, 2, 3, 4], dtype=np.int32).tobytes()

                self.client_socket.sendto(message, self.server_address)

        def timer_callback(self):
                section_0, _ = self.client_socket.recvfrom(4096)
                section_1, _ = self.client_socket.recvfrom(4096)
                section_2, _ = self.client_socket.recvfrom(4096)

                section_0 = np.frombuffer(section_0, dtype=np.uint8)
                section_1 = np.frombuffer(section_1, dtype=np.uint8)
                section_2 = np.frombuffer(section_2, dtype=np.uint8)

                image_data = [section_0, section_1, section_2]

                image_data = np.concatenate(image_data)

                compressed_image = CompressedImage()

                compressed_image.data = image_data.tolist()
                compressed_image.format = 'jpg'

                cv_image = self.bridge.compressed_imgmsg_to_cv2(compressed_image, desired_encoding='bgr8')
      
                cv_image = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')

                self.publisher.publish(cv_image)

                message = np.array([1, 2, 3, 4], dtype=np.int32).tobytes()
                self.client_socket.sendto(message, self.server_address)


def main(args=None):
        rclpy.init(args=args)

        image_publisher = ImagePublisher()

        rclpy.spin(image_publisher)

        image_publisher.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
        main()

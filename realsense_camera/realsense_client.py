import socket
import numpy as np
import signal
import time
import sys

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter('topic', '/camera1')
        self.declare_parameter('port', 12345)

        topic = self.get_parameter('topic').value
        port = self.get_parameter('port').value

        self.publisher = self.create_publisher(Image, topic, 10)

        self.bridge = CvBridge()
        signal.signal(signal.SIGALRM, self.timeout_handler)

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.server_address = ('192.168.1.11', port)
        print("connected")

        self.timee = Clock().now()
        self.counter = 0

        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_header(self, frame_id):
        """Creates a header object for the message

        Header:
            stamp: Time message which has the seconds and nanoseconds since the epoch
            frame_id: TF which the header is relevant to

        Args:
            frame_id (String): This is the transform which the message applies to

        Returns:
            Header: Header containing the timestamp and given frame_id
        """

        # Creates a timer for timestamps
        timer = Clock().now()
        header = Header()

        header.stamp = timer.to_msg()
        header.frame_id = frame_id

        return header

    def timeout_handler(self, signum, frame):
       raise TimeoutError()

    def timer_callback(self):
        signal.alarm(1)

        try:
            self.client_socket.sendto("Message".encode(), self.server_address)

            section_0, _ = self.client_socket.recvfrom(131072)
            self.client_socket.sendto("Message".encode(), self.server_address)

            section_1, _ = self.client_socket.recvfrom(131072)
            self.client_socket.sendto("Message".encode(), self.server_address)

            section_2, _ = self.client_socket.recvfrom(131072)
            self.client_socket.sendto("Message".encode(), self.server_address)

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

        except TimeoutError:
            print("Timeout")
            time.sleep(0.75)
            self.client_socket.close()

            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            signal.alarm(0)
            self.timee = Clock().now()
            self.counter = 0
            return
        except (CvBridgeError, TypeError):
            print("Failed")
            self.client_socket.close()

            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.timee = Clock().now()
            self.counter = 0
            try:
                time.sleep(1)
            except TimeoutError:
                signal.alarm(0)
                return


def main(args=None):
    rclpy.init(args=args)

    topic = sys.argv[1]
    port = int(sys.argv[2])

    try:
        assert topic != None and port != None
    except AssertionError:
        print("Format: python3 test_client.py <topic> <port>")
        sys.exit()

    image_publisher = ImagePublisher(topic, port)

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        image_publisher.client_socket.close()

    image_publisher.destroy_node()


if __name__ == '__main__':
    main()

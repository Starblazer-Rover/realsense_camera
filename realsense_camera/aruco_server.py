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
from sensor_msgs.msg import Image

class ArucoPublisher(Node):
    def __init__(self):
        super().__init__('aruco_publisher')
        self.publisher = self.create_publisher(Int64, '/movement/aruco', 10)

        self.subscriber = self.create_subscription(Image, '/camera/Image_raw', self.timer_callback, 1)

        signal.signal(signal.SIGALRM, self.timeout_handler)

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.server_address = ('192.168.1.60', 12345)

        print("Starting UDP Server")

        self.server_socket.bind(self.server_address)

        self.bridge = CvBridge()
    
    def detect_aruco_corners(self, image_data):
        image = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()

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

    def timer_callback(self, msg):
        int_msg = Int64()

        signal.alarm(1)

        data = self.bridge.imgmsg_to_cv2(msg)

        image_data = np.asarray(data, dtype=np.uint8)

        result_image, center = self.detect_aruco_corners(image_data)

        int_msg.data = int(center[0])

        self.publisher.publish(int_msg)

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

import socket
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
import signal
import zlib
import cv2
import sys


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class Picture(Node):

    def __init__(self, number):
        super().__init__('picture')

        self.number = number

        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(Image, '/camera/Image_raw', self.timer_callback, 10)

    def timer_callback(self, msg):

        data = self.bridge.imgmsg_to_cv2(msg)
        cv2.imwrite(f'/home/billee/billee_ws/src/sensors/resource/picture_{self.number}.png', data)

        print('taken')
        

def main(args=None):
    
    number = int(sys.argv[1])

    rclpy.init(args=args)

    publisher = Picture(number)

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.destroy_node()

if __name__ == '__main__':
    main()

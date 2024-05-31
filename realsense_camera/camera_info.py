import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

import numpy as np


class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__('camera_info_publisher')

        self.publisher = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)

        timer_period = 1/5

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

    def timer_callback(self):
        msg = CameraInfo()

        msg.header = self.create_header('camera_link')

        msg.height = 480
        msg.width = 640

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    publisher = CameraInfoPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.destroy_node()


if __name__ == '__main__':
    main()


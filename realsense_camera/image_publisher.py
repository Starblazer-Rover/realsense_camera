from rclpy.clock import Clock
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node

import pyrealsense2 as rs

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

class ImagePublisher(Node):
    def __init__(self, pipeline):
        super().__init__('image_publisher')

        self.__image_publisher = self.create_publisher(CompressedImage, '/camera/CompressedImage', 1)

        timer_period = 1/500

        self.timer = self.create_timer(timer_period, self.timer_callback)

        timer = Clock().now().to_msg()
        self.start_time = timer.sec + (timer.nanosec / 1000000000)

        self.bridge = CvBridge()
        self.pipeline = pipeline
        
        self.counter = 0
        
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
    
    def create_image(self, color_frame):

        image_data = np.asarray(color_frame.get_data())

        msg = self.bridge.cv2_to_compressed_imgmsg(image_data, 'jpg')
        msg.header = self.create_header('camera_link')

        return msg
    
    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()

        timer = Clock().now().to_msg()

        time = timer.sec + (timer.nanosec / 1000000000)

        time = time - self.start_time

        if color_frame.is_video_frame():
            image_msg = self.create_image(color_frame)
            self.__image_publisher.publish(image_msg)

            self.counter += 1

            #self.get_logger().info(f'{self.counter / time}')

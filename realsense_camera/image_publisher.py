from rclpy.clock import Clock
from cv_bridge import CvBridge
import cv2
import numpy as np

import pyrealsense2 as rs

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

class ImagePublisher():
    def __init__(self):
        self.bridge = CvBridge()
        
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

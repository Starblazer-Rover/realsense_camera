import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from cv_bridge import CvBridge
import numpy as np

import pyrealsense2 as rs

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(CompressedImage, '/camera/compressed_image', 10)
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline.start()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        timer_period = 1 / 30
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

        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if color_frame:
                image_data = np.asarray(color_frame.get_data())
                msg = self.bridge.cv2_to_compressed_imgmsg(image_data, 'jpg')
                msg.header = self.create_header('camera_link')
                self.publisher.publish(msg)
                self.get_logger().info(f'camera working')

        except Exception as e:
            self.get_logger().error(f"Error capturing and publishing image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
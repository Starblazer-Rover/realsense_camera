import numpy as np
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import UInt16MultiArray
import pyrealsense2 as rs

class DepthPublisher(Node):

    def __init__(self, pipeline):
        super().__init__('depth_publisher')

        self.pipeline = pipeline
        self.pointcloud_publisher = self.create_publisher(UInt16MultiArray, '/depth/depth_raw', 1)

        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
            
    def update_depth(self, data):
        """Grabs the depth of all the points in the frame

        Returns:
            List: List containing x, y, z coordinates of each point
        """

        data = np.array(data).reshape((480, 640))

        data = data.astype(float)

        data *= 0.0394
    
        return data

    def timer_callback(self):
        msg = UInt16MultiArray()

        print('here')
        frames = self.pipeline.wait_for_frames()

        data = frames.get_depth_frame().get_data()

        data = np.array(data, dtype=np.uint16).flatten().tolist()

        msg.data = data

        self.pointcloud_publisher.publish(msg)


def __initialize_camera():
    # Initializes the camera for all the frames being analyzed
        camera_pipeline = rs.pipeline()
        camera_config = rs.config()

        camera_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        camera_pipeline.start(camera_config)

        return camera_pipeline


def main(args=None):
    rclpy.init(args=args)

    pipeline = __initialize_camera()

    pointcloud_publisher = DepthPublisher(pipeline)

    try:
        rclpy.spin(pointcloud_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        #pipeline.stop()
        pointcloud_publisher.destroy_node()


if __name__ == '__main__':
    main()

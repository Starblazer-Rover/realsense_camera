import numpy as np
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
import pyrealsense2 as rs

class PointCloudPublisher(Node):

    def __init__(self, pipeline):
        super().__init__('pointcloud_publisher')

        self.pipeline = pipeline
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/depth/PointCloud2_raw', 1)

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
            
    def update_depth(self, data, f_mm):
        """Grabs the depth of all the points in the frame

        Returns:
            List: List containing x, y, z coordinates of each point
        """

        data = np.array(data).reshape((480, 640))


        aspect_ratio = (4, 3)
        aspect_factor = np.sqrt(aspect_ratio[0]**2 + aspect_ratio[1]**2)

        height, width = data.shape

        sensor_diagonal_rad = np.deg2rad(100.6)
        sensor_diagonal_mm = 2 * f_mm * np.tan(sensor_diagonal_rad / 2)

        sensor_width_mm = sensor_diagonal_mm * (aspect_ratio[0] / aspect_factor)
        sensor_height_mm = sensor_diagonal_mm * (aspect_ratio[1] / aspect_factor)

        f_pixels = f_mm * (width / sensor_width_mm)

        data = data.astype(float)

        data *= 0.0394

        data[data == 0] = np.nan

        y_coords, x_coords = np.indices((height, width))

        cx = width / 2
        cy = height / 2

        x_coords = (width - 1- x_coords - cx) * data / f_pixels
        y_coords = (height - 1 - y_coords - cy) * data / f_pixels

        points = np.dstack((data, x_coords, y_coords))
    
        return points

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()

        data = frames.get_depth_frame().get_data()
        
        points = self.update_depth(data, 1.93)

        header = self.create_header('camera_link')

        msg = pc2.create_cloud_xyz32(header, points)

        print('working')

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

    pointcloud_publisher = PointCloudPublisher(pipeline)

    try:
        rclpy.spin(pointcloud_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        pointcloud_publisher.destroy_node()


if __name__ == '__main__':
    main()

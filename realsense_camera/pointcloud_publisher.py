import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import pyrealsense2 as rs
import numpy as np

from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/camera/PointCloud2', 10)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        timer_period = 0.5
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
    
    def update_depth(self):
        """Grabs the depth of all the points in the frame

        Returns:
            List: List containing x, y, z coordinates of each point
        """
        frames = self.pipeline.wait_for_frames()

        for frame in frames:
            if not frame.is_depth_frame():
                continue

            height = frame.as_depth_frame().get_height()
            width = frame.as_depth_frame().get_width()

            points = np.zeros((height, width, 3), dtype=np.float32)

            for y in range(height):
                for x in range(width):
                    depth = frame.as_depth_frame().get_distance(x, y)
                    
                    points[y, x, 0] = x
                    points[y, x, 1] = y
                    points[y, x, 2] = depth

        return points
    
    def create_pointcloud(self):
        header = self.create_header('camera_link')

        """
        Use this format if you want to add anything else like rgb, make sure to change the create cloud to the correct one

        fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
        """

        points = self.update_depth()

        msg = pc2.create_cloud_xyz32(header, points)

        return msg

    def timer_callback(self):
        msg = self.create_pointcloud()
        
        self.publisher.publish(msg)

        self.get_logger().info(f'Working')


def main(args=None):
    rclpy.init(args=args)

    pointcloud_publisher = PointCloudPublisher()

    rclpy.spin(pointcloud_publisher)

    pointcloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
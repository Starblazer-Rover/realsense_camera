from rclpy.clock import Clock
import pyrealsense2 as rs
import numpy as np

from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

class PointCloudPublisher():

    def __create_header(self, frame_id):
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
    
    def __update_depth(self, depth_frame):
        """Grabs the depth of all the points in the frame

        Returns:
            List: List containing x, y, z coordinates of each point
        """

        height = depth_frame.get_height()
        width = depth_frame.get_width()

        points = np.zeros((height, width, 3), dtype=np.float32)

        for y in range(height):
            for x in range(width):
                depth = depth_frame.get_distance(x, y)

                points[y, x, 0] = x
                points[y, x, 1] = y
                points[y, x, 2] = depth

        return points
    
    def create_pointcloud(self, depth_frame):
        header = self.__create_header('camera_link')

        """
        Use this format if you want to add anything else like rgb, make sure to change the create cloud to the correct one

        fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
        """

        points = self.__update_depth(depth_frame)

        msg = pc2.create_cloud_xyz32(header, points)

        return msg


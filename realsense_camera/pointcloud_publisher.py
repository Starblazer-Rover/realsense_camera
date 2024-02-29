from rclpy.clock import Clock
import rclpy
import pyrealsense2 as rs
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import PointCloud2

from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

class PointCloudPublisher(Node):

    def __init__(self, pipeline):
        self.pipeline = pipeline
        super().__init__('pointcloud_publisher')
        self.__pointcloud_publisher = self.create_publisher(PointCloud2, '/map/PointCloud2', 10)
        timer_period = 1/90
        self.timer = self.create_timer(timer_period, self.timer_callback)

        timer = Clock().now().to_msg()
        self.start_time = timer.sec + (timer.nanosec / 1000000000)
        self.counter = 0

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
        
        depths = np.array(depth_frame.get_data())

        y_coords, x_coords = np.indices((height, width))

        y_coords = y_coords[::-1]

        points = np.dstack((depths, x_coords, y_coords))
#--------------------------------------------------------------------
        #points are being manipulated physically to offset the rover object with the point cloud
        #points [:,:,1 and :,:,2 respectively refer to x and y]
        #the offset is taking the rover width and substracting the pixel size and position to center the rover inside the pointcloud
        
       
        points[:,:,1] -= (width)//2
        points[:,:,2] -= (height)//2 
    
        return points
    
    def create_pointcloud(self, depth_frame):
        header = self.__create_header('base_link')

        """
        Use this format if you want to add anything else like rgb, make sure to change the create cloud to the correct one

        fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
        """

        points = self.__update_depth(depth_frame)

        msg = pc2.create_cloud_xyz32(header, points)

        return msg
    
    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()

        timer = Clock().now().to_msg()

        time = timer.sec + (timer.nanosec / 1000000000)

        time = time - self.start_time

        if depth_frame.is_depth_frame():
            pointcloud_msg = self.create_pointcloud(depth_frame)
            self.__pointcloud_publisher.publish(pointcloud_msg)

            self.counter += 1

            self.get_logger().info(f'{self.counter / time}')


def __initialize_camera():
    # Initializes the camera for all the frames being analyzed
        imu_pipeline = rs.pipeline()
        imu_config = rs.config()
        imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
        imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        imu_pipeline.start(imu_config)

        camera_pipeline = rs.pipeline()
        camera_config = rs.config()

        camera_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 90)
        camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        camera_pipeline.start(camera_config)
        

        return imu_pipeline, camera_pipeline


def main(args=None):
    rclpy.init(args=args)

    imu_pipeline, camera_pipeline = __initialize_camera()

    pointcloud_publisher = PointCloudPublisher(camera_pipeline)

    rclpy.spin(pointcloud_publisher)

    pointcloud_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




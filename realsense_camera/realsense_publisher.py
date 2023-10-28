import rclpy
from rclpy.node import Node
import pyrealsense2 as rs

from image_publisher import ImagePublisher
from imu_publisher import ImuPublisher
from pointcloud_publisher import PointCloudPublisher

from sensor_msgs.msg import CompressedImage, Imu, PointCloud2

class RealsensePublisher(Node):
    
    def __init__(self):
        super().__init__('realsense_publisher')
        self.__image_publisher = self.create_publisher(CompressedImage, '/camera/compressed_image', 1)
        self.__imu_publisher = self.create_publisher(Imu, '/odom/imu', 10)
        self.__pointcloud_publisher = self.create_publisher(PointCloud2, '/camera/pointcloud2', 10)

        self.image = ImagePublisher()
        self.imu = ImuPublisher()
        self.pointcloud = PointCloudPublisher()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.accel)
        self.config.enable_stream(rs.stream.gyro)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        self.pipeline.start(self.config)

        self.image_counter = 0
        self.imu_counter = 0
        self.pointcloud_counter = 0

        timer_period = 0.0001
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)
        depth_frame = frames.get_depth_frame()


        if self.image_counter == 5:
            msg = self.image.create_image(color_frame)
            self.__image_publisher.publish(msg)
            self.image_counter = 0
        else:
            self.image_counter += 1

        if self.imu_counter == 4:
            msg = self.imu.create_imu(accel_frame, gyro_frame)
            self.__imu_publisher.publish(msg)
            self.get_logger().info("work")
            #self.get_logger().info(f'{msg.linear_acceleration.x}, {msg.linear_acceleration.y}, {msg.linear_acceleration.z}')
            #self.get_logger().info(f'{msg.angular_velocity.x}, {msg.angular_velocity.y}, {msg.angular_velocity.z}')
            self.imu_counter = 0
        else:
            self.imu_counter += 1

        if self.pointcloud_counter == 5:
            msg = self.pointcloud.create_pointcloud(depth_frame)
            self.__pointcloud_publisher.publish(msg)
            self.pointcloud_counter = 0
        else:
            self.pointcloud_counter += 1


def main(args=None):
    rclpy.init(args=args)

    realsense_publisher = RealsensePublisher()

    rclpy.spin(realsense_publisher)

    realsense_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

        



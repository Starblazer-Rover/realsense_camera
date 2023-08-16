import rclpy
import pyrealsense2 as rs
from rclpy.node import Node
from rclpy.clock import Clock

from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

#! /usr/env/bin python

class ImuPublisher(Node):

    def __init__(self):
        super().__init__('ImuPublisher')
        self.publisher = self.create_publisher(Imu, '/odom/Imu', 10)

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.accel)
        self.config.enable_stream(rs.stream.gyro)
        self.pipeline.start(self.config)
        
        counter = 0
        acceleration_x = []
        acceleration_y = []
        acceleration_z = []

        while counter < 1000:
            frames = self.pipeline.wait_for_frames()

            for frame in frames:
                if frame.is_motion_frame():
                    motion_data = frame.as_motion_frame().get_motion_data()
                    if frame.profile.stream_type() == rs.stream.accel:
                        acceleration_x.append(motion_data.x)
                        acceleration_y.append(motion_data.y)
                        acceleration_z.append(motion_data.z)
                        counter += 1

        self.acceleration_x_offset = sum(acceleration_x) / len(acceleration_x)
        self.acceleration_y_offset = sum(acceleration_y) / len(acceleration_y)
        self.acceleration_z_offset = sum(acceleration_z) / len(acceleration_z)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        msg = Imu()
        header = Header()
        
        timer = Clock().now()

        header.stamp = timer.to_msg()
        header.frame_id = "/odom/Imu"

        msg.header = header
        

        linear_acceleration_x = []
        linear_acceleration_y = []
        linear_acceleration_z = []
        angular_velocity_x = []
        angular_velocity_y = []
        angular_velocity_z = []

        frames = self.pipeline.wait_for_frames()

        for frame in frames:
            if frame.is_motion_frame():
                motion_data = frame.as_motion_frame().get_motion_data()
                
                if frame.profile.stream_type() == rs.stream.accel:
                    linear_acceleration_x.append(motion_data.x - self.acceleration_x_offset)
                    linear_acceleration_y.append(motion_data.y - self.acceleration_y_offset)
                    linear_acceleration_z.append(motion_data.z - self.acceleration_z_offset)
                elif frame.profile.stream_type() == rs.stream.gyro:
                    angular_velocity_x.append(motion_data.x)
                    angular_velocity_y.append(motion_data.y)
                    angular_velocity_z.append(motion_data.z)

        linear_acceleration_x = sum(linear_acceleration_x) / len(linear_acceleration_x)
        linear_acceleration_y = sum(linear_acceleration_y) / len(linear_acceleration_y)
        linear_acceleration_z = sum(linear_acceleration_z) / len(linear_acceleration_z)
        angular_velocity_x = sum(angular_velocity_x) / len(angular_velocity_x)
        angular_velocity_y = sum(angular_velocity_y) / len(angular_velocity_y)
        angular_velocity_z = sum(angular_velocity_z) / len(angular_velocity_z)

        angular_velocity = Vector3()
        angular_velocity.x = angular_velocity_x
        angular_velocity.y = angular_velocity_y
        angular_velocity.z = angular_velocity_z

        linear_acceleration = Vector3()
        linear_acceleration.x = linear_acceleration_x
        linear_acceleration.y = linear_acceleration_y
        linear_acceleration.z = linear_acceleration_z

        msg.angular_velocity = angular_velocity
        msg.linear_acceleration = linear_acceleration

        self.publisher.publish(msg)
        
        
        self.get_logger().info(f'LinearX: {msg.linear_acceleration.x}, LinearY: {msg.linear_acceleration.y}, LinearZ: {msg.linear_acceleration.z}')
        self.get_logger().info(f'AngularX: {msg.angular_velocity.x}, AngularY: {msg.angular_velocity.y}, AngularZ: {msg.angular_velocity.z}')
        self.get_logger().info(f'Time: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        




def main(args=None):
    rclpy.init(args=args)

    imu_publisher = ImuPublisher()

    rclpy.spin(imu_publisher)

    imu_publisher.pipeline.shutdown()
    imu_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

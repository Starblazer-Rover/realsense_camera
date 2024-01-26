import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
from rclpy.clock import Clock

from image_publisher import ImagePublisher
from imu_publisher import ImuPublisher
from pointcloud_publisher import PointCloudPublisher

from sensor_msgs.msg import CompressedImage, Imu, PointCloud2
from rclpy.executors import MultiThreadedExecutor

class RealsensePublisher(Node):
    
    def __init__(self):
        # Initializes Node and Publishers
        super().__init__('realsense_publisher')

        self.__image_publisher = self.create_publisher(CompressedImage, '/camera/CompressedImage', 1)
        self.__imu_publisher = self.create_publisher(Imu, '/odom/Imu', 15)
        #self.__pointcloud_publisher = self.create_publisher(PointCloud2, '/camera/PointCloud2', 10)

        # Creates objects of all data which will be published
        self.image = ImagePublisher()
        self.imu = ImuPublisher()
        self.pointcloud = PointCloudPublisher()
        timer = Clock().now().to_msg()
        self.start_time = timer.sec + (timer.nanosec / 1000000000)

        # Values for the IMU publisher
        self.imu_counter = 0
        self.imu_input = []
        self.imu_output = []
        self.first_data_set = True

        # Timer for the loop
        timer_period = 1/225
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0
    
    def publish_imu(self, state, msg):
        """A switch function which does an initialization of the imu
           or does a normal cycle

           --init--
           This creates a list of 15 IMU messages and processes them through a lowpass filter
           It then sets it to the output list so it can start compiling messages as it is publishing

           --cycle--
           This is the normal cycle after it gets the first 15 IMU messages
           It grabs the new imu data and publishes the old data which is delayed 0.5 seconds
           Once the counter reaches 15, it will send the new data through the lowpass filter and change it to the old data

           This will always give data with a 0.5 second delay. The messages keep their original timestamps, so it is still accurate
           You just get the data a little later.

        Args:
            state (string): switch statement
            msg (IMU msg): IMU Message
        """

        if state == "init":
            if self.imu_counter < 50:
                self.imu_input.append(msg)
                self.imu_counter += 1
            else:
                self.first_data_set = False
                self.imu_counter = 0
                self.imu_output = self.imu.low_pass_filter(self.imu_input[:])
                self.imu_input = []

                self.publish_imu("cycle", msg)

        elif state == "cycle":
            if self.imu_counter == 50:
                self.imu_counter = 0
                self.imu_output = self.imu.low_pass_filter(self.imu_input[:])
                self.imu_input = []

            self.imu_input.append(msg)
            imu_msg = self.imu_output[self.imu_counter]
            self.__imu_publisher.publish(imu_msg)

            #self.get_logger().info(f'Linear_X: {imu_msg.linear_acceleration.x}, Linear_Y: {imu_msg.linear_acceleration.y}, Linear_Z: {imu_msg.linear_acceleration.z}')
            #self.get_logger().info(f'Angular_X: {imu_msg.angular_velocity.x}, Angular Y: {imu_msg.angular_velocity.y}, Angular Z: {imu_msg.angular_velocity.z}')

            self.imu_counter += 1

    def timer_callback(self):
        frames = self.pipeline.poll_for_frames()

        color_frame = frames.get_color_frame()
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)
        depth_frame = frames.get_depth_frame()

        if color_frame.is_video_frame():
            image_msg = self.image.create_image(color_frame)
            self.__image_publisher.publish(image_msg)
            print("camera")
        
        if accel_frame.is_motion_frame() and gyro_frame.is_motion_frame():
            self.get_logger().info("IMU")
            imu_msg = self.imu.create_imu(accel_frame, gyro_frame)

            if self.first_data_set:
                self.publish_imu("init", imu_msg)
            else:
                self.publish_imu("cycle", imu_msg)

def __initialize_camera():
    # Initializes the camera for all the frames being analyzed
        imu_pipeline = rs.pipeline()
        imu_config = rs.config()
        imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
        imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        imu_pipeline.start(imu_config)

        camera_pipeline = rs.pipeline()
        camera_config = rs.config()

        camera_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        camera_pipeline.start(camera_config)
        

        return imu_pipeline, camera_pipeline


def main(args=None):
    rclpy.init(args=args)

    imu_pipeline, camera_pipeline = __initialize_camera()

    image_publisher = ImagePublisher(camera_pipeline)
    imu_publisher = ImuPublisher(imu_pipeline)

    executor = MultiThreadedExecutor()

    executor.add_node(image_publisher)
    executor.add_node(imu_publisher)

    try:
        # Use the executor to spin the node in multiple threads
        executor.spin()
    except KeyboardInterrupt:
        pass

    image_publisher.destroy_node()
    imu_publisher.destroy_node()



if __name__ == '__main__':
    main()

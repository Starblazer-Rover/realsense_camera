import rclpy
import pyrealsense2 as rs

from image_publisher import ImagePublisher
from imu_publisher import ImuPublisher

from rclpy.executors import MultiThreadedExecutor


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

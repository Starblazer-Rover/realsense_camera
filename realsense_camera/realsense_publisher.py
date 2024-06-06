import rclpy
import pyrealsense2 as rs

from image_publisher import ImagePublisher
from pointcloud_publisher import PointCloudPublisher
from depth_publisher import DepthPublisher

from rclpy.executors import MultiThreadedExecutor


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

    camera_pipeline = __initialize_camera()

    image_publisher = ImagePublisher(camera_pipeline)
    pointcloud_publisher = PointCloudPublisher(camera_pipeline)
    depth_publisher = DepthPublisher(camera_pipeline)
    

    executor = MultiThreadedExecutor()

    executor.add_node(image_publisher)
    executor.add_node(pointcloud_publisher)
    executor.add_node(depth_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        camera_pipeline.stop()
        pass



if __name__ == '__main__':
    main()

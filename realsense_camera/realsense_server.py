import rclpy
from rclpy.node import Node
from depth_server import DepthServer
from image_server import ImageServer
from pointcloud_publisher import PointCloudPublisher
from image_publisher import ImagePublisher

from rclpy.executors import MultiThreadedExecutor

import pyrealsense2 as rs
import time

def __initialize_camera():
    # Initializes the camera for all the frames being analyzed
        camera_pipeline = rs.pipeline()
        camera_config = rs.config()

        camera_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        camera_pipeline.start(camera_config)

        return camera_pipeline

camera_pipeline = __initialize_camera()
        

def main(args=None):
    rclpy.init(args=args)

    image_server = ImageServer(camera_pipeline)
    depth_server = DepthServer(camera_pipeline)
    #local_pointcloud = PointCloudPublisher(camera_pipeline)
    #local_image = ImagePublisher(camera_pipeline)

    executor = MultiThreadedExecutor()

    executor.add_node(image_server)
    executor.add_node(depth_server)
    #executor.add_node(local_pointcloud)
    #executor.add_node(local_image)


    try:
        executor.spin()
    except TimeoutError:
         print("Main Timeout")
         rclpy.shutdown()
         image_server.server_socket.close()
         depth_server.server_socket.close()
         image_server = None
         depth_server = None
         # local_pointcloud.destroy_node()
         # local_image.destroy_node()
         time.sleep(5)
         main()
    except KeyboardInterrupt:
         image_server.destroy_node()
         depth_server.destroy_node()
         #local_pointcloud.destroy_node()
         #local_image.destroy_node()
         camera_pipeline.stop()



if __name__ == '__main__':
    main()




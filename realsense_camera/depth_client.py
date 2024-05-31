import socket
import numpy as np
import signal
import time
import sys
import zlib

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


class PointCloudPublisher(Node):

    def __init__(self, port):
        super().__init__('depth_publisher')

        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/depth/PointCloud2', 10)

        signal.signal(signal.SIGALRM, self.timeout_handler)

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.server_address = ('192.168.1.11', port)
        print("connected")

        self.time = Clock().now()
        self.counter = 0

        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timeout_handler(self, signum, frame):
       raise TimeoutError()
    
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
    
    def depth_client(self):
        signal.alarm(1)

        try:
            self.client_socket.sendto("Message".encode(), self.server_address)

            section_0, _ = self.client_socket.recvfrom(131072)
            self.client_socket.sendto("Message".encode(), self.server_address)

            section_1, _ = self.client_socket.recvfrom(131072)
            self.client_socket.sendto("Message".encode(), self.server_address)

            section_2, _ = self.client_socket.recvfrom(131072)
            self.client_socket.sendto("Message".encode(), self.server_address)

            section_3, _ = self.client_socket.recvfrom(131072)
            self.client_socket.sendto("Message".encode(), self.server_address)

            section_0 = np.frombuffer(zlib.decompress(section_0), dtype=np.uint16)
            section_1 = np.frombuffer(zlib.decompress(section_1), dtype=np.uint16)
            section_2 = np.frombuffer(zlib.decompress(section_2), dtype=np.uint16)
            section_3 = np.frombuffer(zlib.decompress(section_3), dtype=np.uint16)

            image_data = [section_0, section_1, section_2, section_3]

            image_data = np.concatenate(image_data)

            image_data = image_data.reshape((480, 640))

            signal.alarm(0)

            return image_data

        except TimeoutError:
            print("Timeout")
            time.sleep(0.75)
            self.client_socket.close()

            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            signal.alarm(0)
            self.time = Clock().now()
            self.counter = 0
            return None
        except (TypeError):
            print("Failed")
            self.client_socket.close()

            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.time = Clock().now()
            self.counter = 0
            try:
                time.sleep(1)
            except TimeoutError:
                signal.alarm(0)
                return None
            
    def update_depth(self, data, f_mm):
        """Grabs the depth of all the points in the frame

        Returns:
            List: List containing x, y, z coordinates of each point
        """
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

        x_coords = (width - 1 - x_coords - cx) * data / f_pixels
        y_coords = (height - 1 - y_coords - cy) * data / f_pixels

        points = np.dstack((data, x_coords, y_coords))

        indices = np.where((points[:,:,1] >=53.7) & (points[:,:,1] <=77.8) & (points[:,:,0] >= 164.0) & (points[:,:,0] <= 190.2))

        #points[indices[0], indices[1], 2] = np.nan
    
        return points

    def timer_callback(self):
        data = self.depth_client()

        try:
            assert isinstance(data, np.ndarray)
        except AssertionError:
            return
        
        points = self.update_depth(data, 1.93)

        header = self.create_header('camera_link')

        msg = pc2.create_cloud_xyz32(header, points)

        time = Clock().now()
        self.counter += 1

        time = (time.nanoseconds - self.time.nanoseconds) / 1000000000

        #print(self.counter / time)

        print('here')

        self.pointcloud_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    port = 12350

    pointcloud_publisher = PointCloudPublisher(port)

    try:
        rclpy.spin(pointcloud_publisher)
    except KeyboardInterrupt:
        pointcloud_publisher.client_socket.close()

    pointcloud_publisher.destroy_node()


if __name__ == '__main__':
    main()

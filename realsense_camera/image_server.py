# This is server code to send video frames over UDP
import cv2, socket
import numpy as np
import time
import base64
import pyrealsense2 as rs
from cv_bridge import CvBridge
from std_msgs.msg import Header
from rclpy.clock import Clock
import time

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

def create_image(color_frame):
        global bridge

        image_data = np.asarray(color_frame.get_data())

        msg = bridge.cv2_to_compressed_imgmsg(image_data, 'jpg')
        msg.header = create_header('camera_link')

        return msg

def create_header(frame_id):
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

def main():
    global bridge

    BUFF_SIZE = 65536
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
    host_ip = 'localhost'  # Change this to your server's IP
    port = 8080
    socket_address = (host_ip, port)
    server_socket.bind(socket_address)

    bridge = CvBridge()

    _,pipeline = __initialize_camera()

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = create_image(color_frame)
        image_data = image.data

        # Convert image data to bytes
        image_bytes = image_data.tobytes()

        # Send image data over UDP
        server_socket.sendto(image_bytes, socket_address)

        # For demonstration purposes, printing a message when data is sent
        print('Image data sent')

        time.sleep(0.5)



if __name__ == '__main__':
    main()
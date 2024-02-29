import socket
import numpy as np
import pyrealsense2 as rs

def initialize_camera():
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

def publish_image(client_socket, camera_pipeline):
    while True:
        frames = camera_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image_data = np.asarray(color_frame.get_data(), dtype=np.int32)

        # Split the image into two halves
        half_width = image_data.shape[1] // 2
        left_half = image_data[:, :half_width]
        right_half = image_data[:, half_width:]

        # Convert halves to bytes
        left_half_bytes = left_half.tobytes()
        right_half_bytes = right_half.tobytes()

        # Determine chunk size (adjust as needed)
        chunk_size = 1024

        # Send left half in chunks
        for i in range(0, len(left_half_bytes), chunk_size):
            chunk = left_half_bytes[i:i+chunk_size]

            client_socket.sendto(chunk, ('localhost', 8080))  # Change 'localhost' to the client's IP

        # For demonstration purposes, printing a message when left half is sent
        print('Left half sent')

        # Send right half in chunks

        for i in range(0, len(right_half_bytes), chunk_size):
            chunk = right_half_bytes[i:i+chunk_size]
           
            client_socket.sendto(chunk, ('localhost', 8080))  # Change 'localhost' to the client's IP

        # For demonstration purposes, printing a message when right half is sent
        print('Right half sent')

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Use SOCK_DGRAM for UDP
    host_ip = 'localhost'  # Change this to your server's IP
    port = 8080
    socket_address = (host_ip, port)
    server_socket.bind(socket_address)

    imu_pipeline, camera_pipeline = initialize_camera()

    print("Server is running and waiting to send image data...")

    try:
        publish_image(server_socket, camera_pipeline)
    finally:
        server_socket.close()

if __name__ == '__main__':
    main()

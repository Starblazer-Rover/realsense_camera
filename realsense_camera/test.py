import pyrealsense2 as rs

def main():
    # Initialize the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)
    
    # Start streaming
    pipeline.start(config)
    
    counter = 0
    acceleration_x = []
    acceleration_y = []
    acceleration_z = []
    try:
        
        while counter < 1000:
            frames = pipeline.wait_for_frames()

            for frame in frames:
                if frame.is_motion_frame():
                    motion_data = frame.as_motion_frame().get_motion_data()
                    if frame.profile.stream_type() == rs.stream.accel:
                        acceleration_x.append(motion_data.x)
                        acceleration_y.append(motion_data.y)
                        acceleration_z.append(motion_data.z)
                        counter += 1

        acceleration_x_offset = sum(acceleration_x) / len(acceleration_x)
        acceleration_y_offset = sum(acceleration_y) / len(acceleration_y)
        acceleration_z_offset = sum(acceleration_z) / len(acceleration_z)
        

        while True:
            # Wait for the next set of frames from the camera
            
            # Get IMU data
            frames = pipeline.wait_for_frames()

            for frame in frames:
                if frame.is_motion_frame():
                    motion_data = frame.as_motion_frame().get_motion_data()
                    if frame.profile.stream_type() == rs.stream.accel:
                        print("Accelerometer data:", motion_data.x - acceleration_x_offset, motion_data.y - acceleration_y_offset, motion_data.z - acceleration_z_offset)
                    elif frame.profile.stream_type() == rs.stream.gyro:
                        print("Gyroscope data:", motion_data.x, motion_data.y, motion_data.z)
    
    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()

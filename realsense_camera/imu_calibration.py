import pyrealsense2 as rs

def optical_to_ros(axis):
    """Changes the frame from the optical frame to REP103 which is what ROS uses
       REP 103

    Args:
        axis (Tuple): A tuple which contains xyz coordinates in the camera frame

    Returns:
        _type_: A tuple which contains xyz coordinates in the REP103 frame
    """
    new_axis = (axis[2], -axis[0], -axis[1])

    return new_axis

def average_data(data):
    """Takes a list of xyz data and gives the average of each

    Args:
        data (tuple): Tuple list containing (x, y, z) values

    Returns:
        tuple: Tuple containing (x, y, z)
    """
    x_sum = 0
    y_sum = 0
    z_sum = 0
    for item in data:
        x_sum += item[0]
        y_sum += item[1]
        z_sum += item[2]

    x = x_sum / len(data)
    y = y_sum / len(data)
    z = z_sum / len(data)

    return (x, y, z)

def main():

    # Camera initialization
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)
    pipeline.start(config)

    linear_acceleration = []
    angular_velocity = []
    counter = 0

    frames = pipeline.wait_for_frames()

    while counter < 1000:
        for frame in frames:
            if not frame.is_motion_frame():
                continue

            motion_data = frame.as_motion_frame().get_motion_data()
            motion_data = (motion_data.x, motion_data.y, motion_data.z)

            if frame.profile.stream_type() == rs.stream.accel:
                linear_acceleration.append(optical_to_ros(motion_data))
            elif frame.profile.stream_type() == rs.stream.gyro:
                angular_velocity.append(optical_to_ros(motion_data))

        counter += 1

    linear_acceleration = average_data(linear_acceleration)
    angular_velocity = average_data(angular_velocity)

    try:
        with open("/home/billee/billee_ws/src/realsense_camera/resource/imu_calibration.dat", "w") as file:
            file.write(f"{linear_acceleration[0]} {linear_acceleration[1]} {linear_acceleration[2]}")
            file.write("\n")
            file.write(f"{angular_velocity[0]} {angular_velocity[1]} {angular_velocity[2]}")
    except FileNotFoundError:
        print("Wrong File")
    except Exception as e:
        print(e)

    print(linear_acceleration)
    print(angular_velocity)





if __name__ == '__main__':
    main()
import rclpy
import pyrealsense2 as rs
from madgwickahrs import MadgwickAHRS
from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

#! /usr/env/bin python

class ImuPublisher(Node):
    

    def __init__(self):
        # ROS Publisher Initialization
        super().__init__('ImuPublisher')
        self.publisher = self.create_publisher(Imu, '/odom/Imu', 10)

        # Camera Initialization
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.accel)
        self.config.enable_stream(rs.stream.gyro)
        self.pipeline.start(self.config)

        # ROS Timer setup
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Quaternion Initialization
        self.madgwick = MadgwickAHRS()

    def read_calibration(self, file_path):
        try:
            with open(file_path, "r") as file:
                data = file.read().split("\n")

                linear_acceleration = data[0].split(" ")
                angular_velocity = data[1].split(" ")

                self.linear_acceleration_offset = [float(linear_acceleration[0]), float(linear_acceleration[1]), float(linear_acceleration[2])]
                self.angular_velocity_offset = [float(angular_velocity[0]), float(angular_velocity[1]), float(angular_velocity[2])]  
        except FileNotFoundError:
            "File Not found"
        except Exception as e:
            print(e)
        


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
    
    def average_data(self, data):
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

        return [x, y, z]
    
    def create_vector3(self, data):
        """Creates a vector object

        Vector3:
            This can contain multiple sets of information like
            linear acceleration and angular velocity

            x: X value
            y: Y value
            z: Z value

        Args:
            data (Tuple): Tuple containing (x, y, z)

        Returns:
            Vector: Vector which has x y z from the tuple
        """
        vector = Vector3()

        vector.x = data[0]
        vector.y = data[1]
        vector.z = data[2]

        return vector
    
    def update_quaternion(self):
        """Creates a quaternion object which keeps track of the object's orientation

        Quaternion:
            This uses linear acceleration and angular velocity as well as the previous quaternion
            to keep track of the entire position of the object

            x: X value
            y: Y value
            z: Z value
            w: W value

        Returns:
            Quaternion: Quaternion of the given acceleration and angular velocity
        """
        quaternion = Quaternion()

        # Uses madgwick to calculate the quaternion
        self.madgwick.update_imu(self.angular_velocity, self.linear_acceleration)

        quaternion.w = float(self.madgwick.quaternion[0])
        quaternion.x = float(self.madgwick.quaternion[1])
        quaternion.y = float(self.madgwick.quaternion[2])
        quaternion.z = float(self.madgwick.quaternion[3])

        return quaternion

    def optical_to_ros(self, axis):
        """Changes the frame from the optical frame to REP103 which is what ROS uses
           REP 103

        Args:
            axis (Tuple): A tuple which contains xyz coordinates in the camera frame

        Returns:
            _type_: A tuple which contains xyz coordinates in the REP103 frame
        """
        new_axis = [axis[2], -axis[0], -axis[1]]

        return new_axis
    
    def update_imu(self):
        """Updates the IMU readings of linear_acceleration and angular velocity

        Returns:
            Tuple: Returns two tuples which contain the relevant IMU data
        """
        linear_acceleration = []
        angular_velocity = []

        # Read the current frames that the camera is seeing
        frames = self.pipeline.wait_for_frames()


        for frame in frames:
            # If it isn't a motion frame, ignore it
            if not frame.is_motion_frame():
                continue

            motion_data = frame.as_motion_frame().get_motion_data()
            motion_data = [motion_data.x, motion_data.y, motion_data.z]

            if frame.profile.stream_type() == rs.stream.accel:
                linear_acceleration.append(self.optical_to_ros(motion_data))
            elif frame.profile.stream_type() == rs.stream.gyro:
                angular_velocity.append(self.optical_to_ros(motion_data))

        # Averages all the data and then returns that data
        linear_acceleration = self.average_data(linear_acceleration)
        angular_velocity = self.average_data(angular_velocity) 

        for i in range(3):
            linear_acceleration[i] -= self.linear_acceleration_offset[i]
            angular_velocity[i] -= self.angular_velocity_offset[i]

            linear_acceleration[i] = float("{:.1f}".format(linear_acceleration[i]))
            angular_velocity[i] = float("{:.1f}".format(angular_velocity[i]))

        return linear_acceleration, angular_velocity
    
    def create_imu(self):
        """Creates the IMU message which will be published

        IMU:
            Header: Header
            Linear Acceleration: Vector3
            Angular Velocity: Vector3
            Quaternion: Quaternion
            Linear Acceleration Covariance: 3x3 float[] 
            Angular Velocity Covariance: 3x3 float[]
            Quaternion Covariance: 3x3 float[]

        Returns:
            Imu: Holds all the IMU data listed above
        """
        default_covariance = [0.1, 0.0, 0.0, 
                              0.0, 0.1, 0.0, 
                              0.0, 0.0, 0.1]
        
        msg = Imu()
        msg.header = self.create_header("camera_link")
        msg.angular_velocity = self.create_vector3(self.angular_velocity)
        msg.linear_acceleration = self.create_vector3(self.linear_acceleration)
        msg.orientation = self.quaternion
        msg.orientation_covariance = default_covariance
        msg.angular_velocity_covariance = default_covariance
        msg.linear_acceleration_covariance = default_covariance
        
        return msg
    
    def logger(self, msg):
        self.get_logger().info(f'LinearX: {msg.linear_acceleration.x}, LinearY: {msg.linear_acceleration.y}, LinearZ: {msg.linear_acceleration.z}')
        self.get_logger().info(f'AngularX: {msg.angular_velocity.x}, AngularY: {msg.angular_velocity.y}, AngularZ: {msg.angular_velocity.z}')
        self.get_logger().info(f'QuaternionW: {msg.orientation.w}, QuaternionX: {msg.orientation.x}, QuaternionY: {msg.orientation.y}, QuaternionZ: {msg.orientation.z}')
        self.get_logger().info(f'Time: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

    def timer_callback(self):
        """
        Repeated Function when the message is going on
        """

        # Grab data
        self.read_calibration("/home/billee/billee_ws/src/realsense_camera/resource/imu_calibration.dat")
        self.linear_acceleration, self.angular_velocity = self.update_imu()
        self.quaternion = self.update_quaternion()

        # Create and publish the message
        msg = self.create_imu()
        self.publisher.publish(msg)

        # Print information for debugging purposes
        self.logger(msg)
        

def main(args=None):
    rclpy.init(args=args)

    imu_publisher = ImuPublisher()

    rclpy.spin(imu_publisher)

    imu_publisher.pipeline.shutdown()
    imu_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

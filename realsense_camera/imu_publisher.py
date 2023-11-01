from madgwickahrs import MadgwickAHRS
import pyrealsense2 as rs
from rclpy.clock import Clock

from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

#! /usr/env/bin python

class ImuPublisher():
    
    def __init__(self):
        # Quaternion Initialization
        self.madgwick = MadgwickAHRS()

    def __read_calibration(self, file_path):
        try:
            with open(file_path, "r") as file:
                data = file.read().split("\n")

                linear_acceleration = data[0].split(" ")
                angular_velocity = data[1].split(" ")

                self.linear_offset = [float(linear_acceleration[0]), float(linear_acceleration[1]), float(linear_acceleration[2])]
                self.angular_offset = [float(angular_velocity[0]), float(angular_velocity[1]), float(angular_velocity[2])]  
        except FileNotFoundError:
            "File Not found"
        except Exception as e:
            print(e)

    def __create_header(self, frame_id):
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
    
    def __create_vector3(self, data):
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
    
    def __update_quaternion(self):
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

    def __optical_to_ros(self, axis):
        """Changes the frame from the optical frame to REP103 which is what ROS uses
           REP 103

        Args:
            axis (Tuple): A tuple which contains xyz coordinates in the camera frame

        Returns:
            _type_: A tuple which contains xyz coordinates in the REP103 frame
        """
        new_axis = [axis[2], -axis[0], -axis[1]]

        return new_axis
    
    def __update_imu(self, accel_frame, gyro_frame):
        """Updates the IMU readings of linear_acceleration and angular velocity

        Returns:
            Tuple: Returns two tuples which contain the relevant IMU data
        """
        linear_acceleration = []
        angular_velocity = []

        accel_data = accel_frame.as_motion_frame().get_motion_data()
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()

        

        linear_acceleration = [accel_data.x, accel_data.y, accel_data.z]
        angular_velocity = [gyro_data.x, gyro_data.y, gyro_data.z]

        biased_linear_acceleration = self.__optical_to_ros(linear_acceleration)
        biased_angular_velocity = self.__optical_to_ros(angular_velocity)

        for i in range(3):
            biased_linear_acceleration[i] -= self.linear_offset[i]
            biased_angular_velocity[i] -= self.angular_offset[i]

        return biased_linear_acceleration, biased_angular_velocity
    
    def __if_accel_zero(self):
        for i in range(3):
            if self.linear_acceleration[i] > 0.0001 or self.angular_velocity[i] > 0.0001:
                return False
            
        return True
    
    def __isnt_moving(self):
        for i in range(3):
            self.linear_acceleration[i] = 0.0
            self.angular_velocity[i] = 0.0
    
    def create_imu(self, accel_frame, gyro_frame):
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

        self.__read_calibration("/home/billee/billee_ws/src/realsense_camera/resource/imu_calibration.dat")
        self.linear_acceleration, self.angular_velocity = self.__update_imu(accel_frame, gyro_frame)

        if self.__if_accel_zero():
            self.__isnt_moving()

        self.quaternion = self.__update_quaternion()
        
        default_covariance = [0.1, 0.0, 0.0, 
                              0.0, 0.1, 0.0, 
                              0.0, 0.0, 0.1]
        
        msg = Imu()
        msg.header = self.__create_header("camera_link")
        msg.angular_velocity = self.__create_vector3(self.angular_velocity)
        msg.linear_acceleration = self.__create_vector3(self.linear_acceleration)
        msg.orientation = self.quaternion
        msg.orientation_covariance = default_covariance
        msg.angular_velocity_covariance = default_covariance
        msg.linear_acceleration_covariance = default_covariance
        
        return msg


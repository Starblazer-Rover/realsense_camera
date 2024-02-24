from madgwickahrs import MadgwickAHRS
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from scipy.signal import butter, lfilter

from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

#! /usr/env/bin python

class ImuPublisher(Node):
    
    def __init__(self, pipeline):
        super().__init__('IMU_publisher')
        self.__imu_publisher = self.create_publisher(Imu, '/odom/Imu', 50)
        timer_period = 1/300

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pipeline = pipeline

        self.__read_calibration("/home/billee/billee_ws/src/realsense_camera/resource/imu_calibration.dat")

        self.imu_counter = 0
        self.imu_input = []
        self.imu_output = []
        self.first_data_set = True

        # Quaternion Initialization
        self.madgwick = MadgwickAHRS()

    def __read_calibration(self, file_path):
        try:
            with open(file_path, "r") as file:
                data = file.read().split("\n")

                linear_acceleration = data[0].split(" ")
                angular_velocity = data[1].split(" ")
                linear_acceleration_threshold = data[2].split(" ")
                angular_velocity_threshold = data[3].split(" ")

                self.linear_offset = [float(linear_acceleration[0]), float(linear_acceleration[1]), float(linear_acceleration[2])]
                self.angular_offset = [float(angular_velocity[0]), float(angular_velocity[1]), float(angular_velocity[2])] 

                threshold = linear_acceleration_threshold + angular_velocity_threshold 

                self.threshold = []

                i = 0
                while i < 12:
                    self.threshold.append((float(threshold[i]), float(threshold[i + 1])))
                    i += 2

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
    
    def __update_quaternion(self, linear_acceleration, angular_velocity):
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
        self.madgwick.update_imu(angular_velocity, linear_acceleration)

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
    
    def __butter_lowpass(self, cutoff, frequency, order):
        """Generates info about the lowpass based on the arguments given

        Args:
            cutoff (float): cutoff frequency in Hz
            frequency (float): Frequency of the sensor in Hz
            order (int): Order of the filter. Keep it low for IMU

        Returns:
            arguments: Returns something that another function can use for the filter
        """

        # Lookup lowpass filters if you need more info on this
        nyquist = 0.5 * frequency
        normal_cutoff = cutoff / nyquist

        return butter(order, normal_cutoff, btype='low', analog=False)
        
    
    def __butter_offset(self, data):
        """Establishes basic datapoints of the butter filter and executes them through the filter

        Args:
            data (list): List of the data being filtered

        Returns:
            list: Filtered data in the list
        """

        # Cutoff frequency: Set at 0.5Hz
        CUTOFF = 0.5
        # Order. Kept low because Odometry needs to keep accuracy
        ORDER = 2

        # Offset the data so the middle-point of the raw data is centered at 0
        #data = self.offset(data)

        # Grab the filter preset
        b, a = self.__butter_lowpass(CUTOFF, 30, ORDER)

        # Create filtered data using the preset and the list of raw data
        return lfilter(b, a, data)
        
    def __establish_degrees(self, msg):
        """Creates a list of all the data in the msg

        Args:
            msg (Imu Msg): Imu Msg

        Returns:
            Imu Msg: Imu Msg
        """

        return [msg.linear_acceleration.x, 
                msg.linear_acceleration.y, 
                msg.linear_acceleration.z, 
                msg.angular_velocity.x, 
                msg.angular_velocity.y, 
                msg.angular_velocity.z
                ]

    def low_pass_filter(self, input_data):
        """Grabs data of 15 messages and puts it through a lowpass filter to remove noise

        Args:
            input_data (list): List of IMU msgs

        Returns:
            list: List of filtered IMU msgs
        """
        
        # Generate 6 empty lists inside the list
        # Each sub-list represents a degree of freedom
        # 0: Linear_acceleration_x
        # 1: Linear_acceleration_y
        # 2: Linear_acceleration_z
        # 3: Angular_velocity_x
        # 4: Angular_velocity_y
        # 5: Angular_velocity_z
        lists = [[] for _ in range(6)]
        
        # Data is grabbed and put in its respective list
        for i in range(len(input_data)):

            # Grab all the data and put it in a list
            degrees = self.__establish_degrees(input_data[i])

            # Take the data and put it in the respective section
            for j in range(len(lists)):
                lists[j].append(degrees[j])

        # Now that the data is compiled, we can put it through the filter and change it
        new_list = [self.__butter_offset(item) for item in lists]

        # Since Python doesnt allow for pointers, this has to be coded like this
        for i in range(len(input_data)):
            input_data[i].linear_acceleration.x = new_list[0][i] if (new_list[0][i] > self.threshold[0][0] or new_list[0][i] < self.threshold[0][1]) else 0.0
            input_data[i].linear_acceleration.y = new_list[1][i] if new_list[1][i] > self.threshold[1][0] or new_list[1][i] < self.threshold[1][1] else 0.0
            input_data[i].linear_acceleration.z = new_list[2][i] if new_list[2][i] > self.threshold[2][0] or new_list[2][i] < self.threshold[2][1] else 0.0
            input_data[i].angular_velocity.x = new_list[3][i] if new_list[3][i] > self.threshold[3][0] or new_list[3][i] < self.threshold[3][1] else 0.0
            input_data[i].angular_velocity.y = new_list[4][i] if new_list[4][i] > self.threshold[4][0] or new_list[4][i] < self.threshold[4][1] else 0.0
            input_data[i].angular_velocity.z = new_list[5][i] if new_list[5][i] > self.threshold[5][0] or new_list[5][i] < self.threshold[5][1] else 0.0

            # Establish linear accel and angular velocity for quaternion calculation
            linear_acceleration = (input_data[i].linear_acceleration.x, input_data[i].linear_acceleration.y, input_data[i].linear_acceleration.z)
            angular_velocity = (input_data[i].angular_velocity.x, input_data[i].angular_velocity.y, input_data[i].angular_velocity.z)

            # Calculate quaternion
            input_data[i].orientation = self.__update_quaternion(linear_acceleration, angular_velocity)

        return input_data
    
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

        linear_acceleration, angular_velocity = self.__update_imu(accel_frame, gyro_frame)
        
        default_covariance = [0.1, 0.0, 0.0, 
                              0.0, 0.1, 0.0, 
                              0.0, 0.0, 0.1]
        
        msg = Imu()
        msg.header = self.__create_header("camera_link")
        msg.angular_velocity = self.__create_vector3(angular_velocity)
        msg.linear_acceleration = self.__create_vector3(linear_acceleration)

        msg.orientation_covariance = default_covariance
        msg.angular_velocity_covariance = default_covariance
        msg.linear_acceleration_covariance = default_covariance
        
        return msg
    
    def publish_imu(self, state, msg):
        """A switch function which does an initialization of the imu
           or does a normal cycle

           --init--
           This creates a list of 15 IMU messages and processes them through a lowpass filter
           It then sets it to the output list so it can start compiling messages as it is publishing

           --cycle--
           This is the normal cycle after it gets the first 15 IMU messages
           It grabs the new imu data and publishes the old data which is delayed 0.5 seconds
           Once the counter reaches 15, it will send the new data through the lowpass filter and change it to the old data

           This will always give data with a 0.5 second delay. The messages keep their original timestamps, so it is still accurate
           You just get the data a little later.

        Args:
            state (string): switch statement
            msg (IMU msg): IMU Message
        """

        LOWPASS_OFFSET = 50

        if state == "init":
            if self.imu_counter < LOWPASS_OFFSET:
                self.imu_input.append(msg)
                self.imu_counter += 1
            else:
                self.first_data_set = False
                self.imu_counter = 0
                self.imu_output = self.low_pass_filter(self.imu_input[:])
                self.imu_input = []

                self.publish_imu("cycle", msg)

        elif state == "cycle":
            if self.imu_counter == LOWPASS_OFFSET:
                self.imu_counter = 0
                self.imu_output = self.low_pass_filter(self.imu_input[:])
                self.imu_input = []

            self.imu_input.append(msg)
            imu_msg = self.imu_output[self.imu_counter]
            self.__imu_publisher.publish(imu_msg)

            #self.get_logger().info(f'Linear_X: {imu_msg.linear_acceleration.x}, Linear_Y: {imu_msg.linear_acceleration.y}, Linear_Z: {imu_msg.linear_acceleration.z}')
            #self.get_logger().info(f'Angular_X: {imu_msg.angular_velocity.x}, Angular Y: {imu_msg.angular_velocity.y}, Angular Z: {imu_msg.angular_velocity.z}')

            self.imu_counter += 1
    
    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()

        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        if accel_frame.is_motion_frame() and gyro_frame.is_motion_frame():

            imu_msg = self.create_imu(accel_frame, gyro_frame)

            if self.first_data_set:
                self.publish_imu("init", imu_msg)
            else:
                self.publish_imu("cycle", imu_msg)


def __initialize_camera():
    # Initializes the camera for all the frames being analyzed
        imu_pipeline = rs.pipeline()
        imu_config = rs.config()
        imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
        imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        imu_pipeline.start(imu_config)

        return imu_pipeline

def main(args=None):
    rclpy.init(args=args)

    imu_pipeline = __initialize_camera()

    imu_publisher = ImuPublisher(imu_pipeline)

    try:
        rclpy.spin(imu_publisher)
    except:
        imu_publisher.destroy_node()
        imu_pipeline.stop()

if __name__ == '__main__':
    main()


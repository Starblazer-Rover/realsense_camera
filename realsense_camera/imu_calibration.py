import pyrealsense2 as rs
import rclpy
import sys
from rclpy.node import Node
from scipy.signal import butter, lfilter

class ImuCalibrator(Node):
    
    def __init__(self):
        super().__init__('calibration_node')

        # Camera initialization
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        self.pipeline.start(config)

        # Imu value initialization
        self.counter = 0
        self.data = [[] for _ in range(6)]
        self.offsets = [[] for _ in range(6)]
        self.thresholds = [[] for _ in range(6)]

        # Timer
        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def __optical_to_ros(self, accel_data, gyro_data):
        """Changes the frame from the optical frame to REP103 which is what ROS uses
        REP 103

        Args:
            axis (Tuple): A tuple which contains xyz coordinates in the camera frame

        Returns:
            _type_: A tuple which contains xyz coordinates in the REP103 frame
        """

        return (accel_data.z, -accel_data.x, -accel_data.y, gyro_data.z, -gyro_data.x, -gyro_data.y)

    def __offset(self, data):
        """Return the median of the data

        Args:
            data (list): List of all the data for the respective degree

        Returns:
            int: median of data
        """
        return (max(data) + min(data)) / 2
    
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
    
    def __print_calibration(self, filename):
        try:
            with open(filename, "w") as file:
                file.write(f"{self.offsets[0]} {self.offsets[1]} {self.offsets[2]}")
                file.write("\n")
                file.write(f"{self.offsets[3]} {self.offsets[4]} {self.offsets[5]}")
                file.write("\n")
                file.write(f"{self.thresholds[0][0]} {self.thresholds[0][1]} {self.thresholds[1][0]} {self.thresholds[1][1]} {self.thresholds[2][0]} {self.thresholds[2][1]}")
                file.write("\n")
                file.write(f"{self.thresholds[3][0]} {self.thresholds[3][1]} {self.thresholds[4][0]} {self.thresholds[4][1]} {self.thresholds[5][0]} {self.thresholds[5][1]}")

            self.get_logger().info("Finished")
            sys.exit()

        except FileNotFoundError:
            self.get_logger().info("Wrong File")
        except Exception as e:
            self.get_logger().info(e)

    def timer_callback(self):

        frames = self.pipeline.wait_for_frames()

        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        accel_data = accel_frame.as_motion_frame().get_motion_data()
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()

        data = self.__optical_to_ros(accel_data, gyro_data)

        for i in range(6):
            self.data[i].append(data[i])


        if self.counter == 10000:
            for i in range(6):
                self.offsets[i] = self.__offset(self.data[i])

                for j in range(len(self.data[i])):
                    self.data[i][j] -= self.offsets[i]

                self.data[i] = self.__butter_offset(self.data[i])

                if max(self.data[i]) >= 0:
                    maximum = max(self.data[i]) * 1.1
                else:
                    maximum = max(self.data[i]) * 0.9

                if min(self.data[i]) <= 0:
                    minimum = min(self.data[i]) * 1.1
                else:
                    minimum = min(self.data[i]) * 0.9

                self.thresholds[i] = (maximum, minimum)

            self.__print_calibration("/home/billee/billee_ws/src/realsense_camera/resource/imu_calibration.dat")

        print(f"{self.counter / 100}%", end="\r")
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    imu_calibrator = ImuCalibrator()

    rclpy.spin(imu_calibrator)

    imu_calibrator.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


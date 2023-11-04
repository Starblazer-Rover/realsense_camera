import rclpy
from rclpy.node import Node

import numpy as np
from scipy.signal import butter, lfilter
import matplotlib.pyplot as plt

from sensor_msgs.msg import Imu

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.acceleration_x = []
        self.new_acceleration_x = []
        self.acceleration_y = []
        self.acceleration_z = []
        self.time_list = []
        self.counter = 0
        self.timer = 0
        self.subscription = self.create_subscription(Imu, '/odom/Imu', self.listener_callback, 10)

    def butter_lowpass(self, cutoff, fs, order=5):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def listener_callback(self, msg):
        if self.counter == 0:
            self.timer = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1000000000)

        if self.counter < 90:
            self.acceleration_x.append(msg.linear_acceleration.x)
            self.acceleration_y.append(msg.linear_acceleration.y)
            self.acceleration_z.append(msg.linear_acceleration.z)
            self.time_list.append(self.timer - (msg.header.stamp.sec + (msg.header.stamp.nanosec / 1000000000)))

        if self.counter == 90:

            max_val = max(self.acceleration_x)
            min_val = min(self.acceleration_x)

            offset = (max_val + min_val) / 2

            cutoff_frequency = 0.5
            filter_order = 2

            for i in range(len(self.acceleration_x)):
                self.acceleration_x[i] -= offset

            b, a = self.butter_lowpass(cutoff_frequency, 30, filter_order)
            
            self.new_acceleration_x = lfilter(b, a, self.acceleration_x)


            """
            for i in range(len(self.acceleration_x)):
                self.acceleration_x[i] = self.acceleration_x[i]**2
            noise_power = np.mean(self.acceleration_x)
            SNR = -10 * np.log10(noise_power)
            self.get_logger().info(f'{SNR}, {offset}')
            """
            plt.figure(figsize=(10, 6))
            plt.plot(self.time_list, self.acceleration_x, label="raw")
            plt.plot(self.time_list, self.new_acceleration_x, label="butter")
            plt.xlabel('Time')
            plt.ylabel('Data')
            plt.title('ZACH')
            plt.legend()
            plt.grid(True)

            plt.show()
            
            self.counter += 1

        else:
            self.counter += 1

def main(args=None):
    rclpy.init(args=args)

    test_subscriber = TestSubscriber()

    rclpy.spin(test_subscriber)

    test_subscriber.destroy_node()
    rclpy.shutdown()


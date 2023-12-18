import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.i = 0

        self.publisher = self.create_publisher(String, 'test_publisher', 10)

        timer_period = 1/30

        self.timer = self.create_timer(timer_period, self.listener_callback)

    def listener_callback(self):
        msg = String()

        msg.data = f"{self.i}"

        self.publisher.publish(msg)

        self.i += 1

        self.get_logger().info(f'{self.i}')
        

def main(args=None):
    rclpy.init(args=args)

    test_subscriber = TestSubscriber()

    rclpy.spin(test_subscriber)

    test_subscriber.destroy_node()
    rclpy.shutdown()

main()

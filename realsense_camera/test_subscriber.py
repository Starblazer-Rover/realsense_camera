import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(Image, '/camera/raw_image', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        print(msg)

def main(args=None):
    rclpy.init(args=args)

    test_subscriber = TestSubscriber()

    rclpy.spin(test_subscriber)

    test_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
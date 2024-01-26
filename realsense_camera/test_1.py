import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.create_timer(1, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'First Counter: {self.counter}')
        self.counter += 1


class SecondNode(Node):
    def __init__(self):
        super().__init__('second_node')
        self.create_timer(0.1, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Second Counter: {self.counter}')
        self.counter += 1


def main():
    rclpy.init()

    my_node = MyNode()
    second_node = SecondNode()

    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()

    # Add the node to the executor
    executor.add_node(my_node)
    executor.add_node(second_node)

    try:
        # Use the executor to spin the node in multiple threads
        executor.spin()
    except KeyboardInterrupt:
        pass

    # Cleanup
    my_node.destroy_node()
    second_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

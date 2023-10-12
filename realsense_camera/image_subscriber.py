import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(Image, '/camera/raw_image', self.image_callback, 10)
    self.bridge = CvBridge()

  def image_callback(self, msg):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
      cv2.imshow('Image', cv_image)
      cv2.waitKey(1)
    except Exception as e:
      self.get_logger().error('Error processing image: %s' % str(e))

def main(args=None):
  rclpy.init(args=args)
  node = ImageSubscriber()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

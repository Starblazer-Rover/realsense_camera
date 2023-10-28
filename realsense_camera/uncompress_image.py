import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('image_node')
    self.publisher = self.create_publisher(Image, '/camera/RawImage', 1)
    self.subscription = self.create_subscription(CompressedImage, '/camera/CompressedImage', self.image_callback, 1)
    self.bridge = CvBridge()

  def image_callback(self, msg):
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
      
      cv_image = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')

      self.publisher.publish(cv_image)

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

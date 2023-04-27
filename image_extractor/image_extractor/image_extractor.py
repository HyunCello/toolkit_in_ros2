import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageExtractor(Node):

    def __init__(self):
        super().__init__('image_extractor')
        self.subscription = self.create_subscription(
            Image,
            '/zed2i/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)
        self.subscription
        self.bridge = CvBridge()
        self.image_count = 0
        self.output_dir = 'extracted_images'

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        output_path = os.path.join(self.output_dir, f'image_{self.image_count:04d}.png')
        cv2.imwrite(output_path, cv_image)
        self.image_count += 1
        self.get_logger().info(f'Saved image: {output_path}')

def main(args=None):
    rclpy.init(args=args)

    image_extractor = ImageExtractor()

    rclpy.spin(image_extractor)

    image_extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

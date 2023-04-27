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
            'zed2i/zed_node/depth/depth_registered',
            self.image_callback,
            10)
        self.subscription
        self.bridge = CvBridge()
        self.image_count = 0
        self.output_dir = 'extracted_depths'

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Set custom min and max values for depth normalization
        custom_min_depth = 0.1  # Adjust this value based on your specific use case
        custom_max_depth = 20.0  # Adjust this value based on your specific use case (20 meters for ZED 2i)

        # Clip the values to the custom depth range and normalize
        clipped_image = cv2.min(cv_image, custom_max_depth)
        normalized_image = (clipped_image - custom_min_depth) / (custom_max_depth - custom_min_depth)
        normalized_image = (normalized_image * 255).astype('uint8')

        # Set values beyond the max depth to a non-white color (e.g., black)
        beyond_max_depth_color = 0  # Set this value to a non-white color (e.g., 0 for black)
        normalized_image[cv_image >= custom_max_depth] = beyond_max_depth_color

        output_path = os.path.join(self.output_dir, f'image_{self.image_count:04d}.png')
        cv2.imwrite(output_path, normalized_image)
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

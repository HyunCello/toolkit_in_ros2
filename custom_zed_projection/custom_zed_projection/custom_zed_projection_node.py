import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

class CustomZedProjectionNode(Node):
    def __init__(self):
        super().__init__('custom_zed_projection_node')
        self.subscription = self.create_subscription(
            Image,
            'zed2i/zed_node/depth/depth_registered',  # Replace this with your depth image topic
            self.depth_callback,
            10)
        self.binary_image_pub = self.create_publisher(Image, "binary_image", 10)
        self.bridge = CvBridge()

    def depth_callback(self, depth_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        except CvBridgeError as e:
            print(e)

        binary_image = self.create_binary_image(depth_image)
        binary_image_msg = self.bridge.cv2_to_imgmsg(binary_image, encoding="mono8")

        self.binary_image_pub.publish(binary_image_msg)

    def create_binary_image(self, depth_image):
        # Modify the following line to define the depth range you are interested in
        mask = cv2.inRange(depth_image, 0.5, 3.0)

        # Apply the mask to the depth image
        binary_image = cv2.bitwise_and(depth_image, depth_image, mask=mask)

        # Threshold the depth image to create a binary image
        _, binary_image = cv2.threshold(binary_image, 0, 255, cv2.THRESH_BINARY)

        return binary_image

def main(args=None):
    rclpy.init(args=args)
    custom_zed_projection_node = CustomZedProjectionNode()
    rclpy.spin(custom_zed_projection_node)
    custom_zed_projection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

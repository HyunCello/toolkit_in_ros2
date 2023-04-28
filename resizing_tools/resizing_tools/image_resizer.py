import cv2
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile

class ImageResizer(Node):

    def __init__(self):
        super().__init__('image_resizer')

        self.bridge = CvBridge()
        self.declare_parameter("width", 800)
        self.declare_parameter("height", 600)
        self.declare_parameter("input_topic", '/front_top/color/image_raw/compressed')

        qos_profile = QoSProfile(depth=1)
        self.sub = self.create_subscription(Image, self.get_parameter("input_topic").value , self.callback, qos_profile=qos_profile)
        self.pub = self.create_publisher(Image, '/resize/image', qos_profile=qos_profile)

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Get the desired size from the parameters
        desired_width = self.get_parameter("width").value
        desired_height = self.get_parameter("height").value

        height, width, _ = image.shape
        aspect_ratio = width / height

        if (desired_width / desired_height) > aspect_ratio:
            new_width = int(desired_height * aspect_ratio)
            resized_image = cv2.resize(image, (new_width, desired_height))
            margin = int((desired_width - new_width) / 2)
            if resized_image.shape[1] % 2 != 0:
            # height, width, _ = resized_image.shape
            # self.get_logger().info("1"+str([height, width]))
                resized_img = cv2.copyMakeBorder(resized_image, 0, 0, margin+1, margin, cv2.BORDER_CONSTANT, (255,255,255))
            else:
                resized_img = cv2.copyMakeBorder(resized_image, 0, 0, margin, margin, cv2.BORDER_CONSTANT, (255,255,255))

        else:
            new_height = int(desired_width / aspect_ratio)
            resized_image = cv2.resize(image, (desired_width, new_height))
            margin = int((desired_height - new_height) / 2)
            if resized_image.shape[0] % 2 != 0:
                # height, width, _ = resized_image.shape
                # self.get_logger().info("2"+str([height, width]))

                resized_img = cv2.copyMakeBorder(resized_image, margin+1, margin, 0, 0, cv2.BORDER_CONSTANT,  (255,255,255))
            else:
                resized_img = cv2.copyMakeBorder(resized_image, margin, margin, 0, 0, cv2.BORDER_CONSTANT,  (255,255,255))
        height, width, _ = resized_img.shape
        # self.get_logger().info("2"+str([height, width]))
        # self.get_logger().info(str([height, width]))

        try:
            resized_msg = self.bridge.cv2_to_imgmsg(resized_img, "bgr8")
        except CvBridgeError as e:
            print(e)

        resized_msg.header = msg.header

        self.pub.publish(resized_msg)

def main(args=None):
    rclpy.init(args=args)
    image_resizer = ImageResizer()

    rclpy.spin(image_resizer)

    image_resizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2

class CustomZedProjectionNode(Node):
    def __init__(self):
        super().__init__('custom_zed_projection_node')
        self.publisher_ = self.create_publisher(PointCloud2, 'custom_point_cloud', 10)
        self.subscription = self.create_subscription(PointCloud2, 'zed2i/zed_node/point_cloud/cloud_registered', self.point_cloud_callback, 10)
        self.depth_sub = self.create_subscription(Image, 'zed2i/zed_node/depth/depth_registered', self.depth_callback, 10)
        self.bridge = CvBridge()
        self.binary_image_publisher_ = self.create_publisher(Image, 'binary_image', 10)

    def depth_callback(self, msg: Image):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        road_depth_min = 1.0
        road_depth_max = 5.0

        img_height, img_width = depth_image.shape
        top_width = 0.3 * img_width
        bottom_width = 0.9 * img_width
        height_fraction = 0.8

        mask = np.zeros_like(depth_image, dtype=np.uint8)
        vertices = np.array([[
            (img_width * (1 - top_width) / 2, img_height * (1 - height_fraction)),
            (img_width * (1 - bottom_width) / 2, img_height),
            (img_width * (1 + bottom_width) / 2, img_height),
            (img_width * (1 + top_width) / 2, img_height * (1 - height_fraction))
        ]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, 1)

        binary_image = np.zeros_like(depth_image, dtype=np.uint8)
        binary_image[(depth_image >= road_depth_min) & (depth_image <= road_depth_max) & (mask == 1)] = 1

        self.depth_image = depth_image
        self.binary_image = binary_image

        # Create a color version of the binary image
        color_binary_image = cv2.cvtColor(self.binary_image * 255, cv2.COLOR_GRAY2BGR)
        color_binary_image[:, :, 1] = 0  # Set the green channel to 0
        color_binary_image[:, :, 2] = 0  # Set the red channel to 0

        # Convert the color binary image to an Image message and publish it
        color_binary_image_msg = self.bridge.cv2_to_imgmsg(color_binary_image, encoding="bgr8")
        color_binary_image_msg.header.stamp = msg.header.stamp
        color_binary_image_msg.header.frame_id = msg.header.frame_id
        self.binary_image_publisher_.publish(color_binary_image_msg)

    def point_cloud_callback(self, msg):
        if not hasattr(self, 'depth_image') or not hasattr(self, 'binary_image'):
            return

        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        modified_points = []

        for point in points:
            x, y, z = point
            i = int(round(y))
            j = int(round(x))

            if 0 <= i < self.binary_image.shape[0] and 0 <= j < self.binary_image.shape[1]:
                binary_value = self.binary_image[i, j]
                if binary_value == 1:
                    z_modified = 0.0
                    modified_points.append([x, y, z_modified])
                else:
                    modified_points.append([x, y, z])

        # Create a new PointCloud2 message with modified points
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        new_point_cloud_msg = pc2.create_cloud(msg.header, fields, modified_points)
        self.publisher_.publish(new_point_cloud_msg)


def main(args=None):
    rclpy.init(args=args)

    custom_zed_projection_node = CustomZedProjectionNode()

    rclpy.spin(custom_zed_projection_node)

    custom_zed_projection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

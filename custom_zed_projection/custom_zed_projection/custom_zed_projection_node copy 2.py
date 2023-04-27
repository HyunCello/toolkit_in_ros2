import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import math
import cv2
import numpy as np
import tf2_ros
from tf_transformations import euler_from_quaternion


class CustomDepthToOccupancyGridNode(Node):
    def __init__(self):
        super().__init__("custom_depth_to_occupancy_grid_node")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, "zed2i/zed_node/depth/depth_registered", self.depth_callback, 10
        )
        self.occupancy_grid_publisher_ = self.create_publisher(OccupancyGrid, "occupancy_grid", 10)
        self.binary_image_publisher_ = self.create_publisher(
            Image, "colored_binary_image", 10
        )
        self.pose_subscriber_ = self.create_subscription(
            PoseStamped, "/zed2i/zed_node/pose", self.pose_callback, 10
        )
        self.camera_pose_ = None
        self.previous_occupancy_grid_data = None
        self.global_occupancy_grid_data = None
        self.global_occupancy_grid_metadata = MapMetaData()

    def pose_callback(self, msg: PoseStamped):
        self.camera_pose_ = msg.pose

    def create_binary_image(self, depth_image):
        img_height, img_width = depth_image.shape

        # Create a trapezoidal binary image
        binary_image = np.zeros((img_height, img_width), dtype=np.uint8)
        vertices = np.array(
            [
                [
                    (img_width * 0.25, img_height * 0.25),
                    (img_width * 0.25, img_height * 0.75),
                    (img_width * 0.75, img_height * 0.75),
                    (img_width * 0.75, img_height * 0.25),
                ]
            ],
            dtype=np.int32,
        )
        cv2.fillPoly(binary_image, vertices, 1)

        return binary_image

    def publish_colored_binary_image(self, binary_image):
        # Create a colored version of the binary image
        colored_binary_image = cv2.cvtColor(binary_image * 255, cv2.COLOR_GRAY2BGR)

        # Set the color of the trapezoidal area
        color = (0, 255, 0)  # Green
        colored_binary_image[
            np.where((colored_binary_image == [255, 255, 255]).all(axis=2))
        ] = color

        # Convert the colored binary image to an Image message and publish it
        binary_image_msg = self.bridge.cv2_to_imgmsg(
            colored_binary_image, encoding="bgr8"
        )
        binary_image_msg.header.stamp = self.get_clock().now().to_msg()
        binary_image_msg.header.frame_id = "base_link"
        self.binary_image_publisher_.publish(binary_image_msg)

    def depth_callback(self, msg: Image):
        if self.camera_pose_ is None:
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        binary_image = self.create_binary_image(depth_image)
        self.publish_colored_binary_image(binary_image)

        img_height, img_width = depth_image.shape
        occupancy_grid_data = []

        for i in range(img_height):
            for j in range(img_width):
                binary_value = binary_image[i, j]
                if binary_value == 1:
                    depth_value = depth_image[i, j]
                    if not np.isnan(depth_value) and not np.isinf(depth_value):
                        occupancy_value = int(depth_value * 100)
                    else:
                        occupancy_value = -1
                else:
                    occupancy_value = -1
                occupancy_value = min(max(occupancy_value, -128), 127)
                occupancy_grid_data.append(occupancy_value)

        # Update the global occupancy grid data
        self.update_global_occupancy_grid(occupancy_grid_data, img_width, img_height)

        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = msg.header.stamp
        occupancy_grid_msg.header.frame_id = "map"
        occupancy_grid_msg.info = self.global_occupancy_grid_metadata
        occupancy_grid_msg.data = self.global_occupancy_grid_data

        self.occupancy_grid_publisher_.publish(occupancy_grid_msg)

    def update_global_occupancy_grid(self, new_data, new_width, new_height):
        if self.global_occupancy_grid_data is None:
            self.global_occupancy_grid_data = new_data
            self.global_occupancy_grid_metadata.width = new_width
            self.global_occupancy_grid_metadata.height = new_height
            self.global_occupancy_grid_metadata.resolution = 0.01
            self.global_occupancy_grid_metadata.origin.position = (
                self.camera_pose_.position
            )
            self.global_occupancy_grid_metadata.origin.orientation = (
                self.camera_pose_.orientation
            )
            return

        # Get the relative position and orientation of the new occupancy grid data
        x_rel, y_rel, yaw_rel = self.get_relative_pose()

        # Calculate the pixel offset
        dx = int(math.floor(x_rel / self.global_occupancy_grid_metadata.resolution))
        dy = int(math.floor(y_rel / self.global_occupancy_grid_metadata.resolution))

        for i in range(new_height):
            for j in range(new_width):
                x_old = j - dx
                y_old = i - dy

                if 0 <= x_old < new_width and 0 <= y_old < new_height:
                    old_index = y_old * new_width + x_old
                    new_index = i * new_width + j

                    if new_data[new_index] != -1:
                        self.global_occupancy_grid_data[old_index] = new_data[new_index]

    def get_relative_pose(self):
        x = (
            self.camera_pose_.position.x
            - self.global_occupancy_grid_metadata.origin.position.x
        )
        y = (
            self.camera_pose_.position.y
            - self.global_occupancy_grid_metadata.origin.position.y
        )
        yaw = self.get_yaw_from_orientation(
            self.camera_pose_.orientation
        ) - self.get_yaw_from_orientation(
            self.global_occupancy_grid_metadata.origin.orientation
        )
        return x, y, yaw

    def get_yaw_from_orientation(self, orientation):
        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        return yaw


def main(args=None):
    rclpy.init(args=args)
    custom_depth_to_occupancy_grid_node = CustomDepthToOccupancyGridNode()
    rclpy.spin(custom_depth_to_occupancy_grid_node)
    custom_depth_to_occupancy_grid_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

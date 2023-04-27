import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import glob
import time

class ImagePubSub(Node):
    def __init__(self, files):
        super().__init__('image_pubsub')
        self.publisher_ = self.create_publisher(Image, '/zed2i/zed_node/rgb/image_rect_color', 10)
        self.subscription = self.create_subscription(
            Image,
            '/unet/colored_segmentation_mask',
            self.binary_image_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()
        self.image_files = files
        self.image_index = 0
        # self.video_file = files
        # self.video_capture = cv2.VideoCapture(self.video_file)
        # self.process_video = True


    # def binary_image_callback(self, msg):
    #     current_time = self.get_clock().now()
    #     elapsed_time = (current_time - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
    #     fps = 1 / elapsed_time
    #     self.get_logger().info('FPS: %f' % fps)

    #     if self.process_video:
    #         self.publish_next_frame()

    # def publish_next_frame(self):
    #     ret, frame = self.video_capture.read()
    #     if ret:
    #         if frame is not None:
    #             print("Frame data type:", type(frame))  # Debugging line
    #             msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
    #             msg.header.stamp = self.get_clock().now().to_msg()
    #             self.publisher_.publish(msg)
    #         else:
    #             self.get_logger().error('Failed to read a frame from the video file.')
    #             self.process_video = False
    #             self.video_capture.release()
    #     else:
    #         self.get_logger().info('End of the video file.')
    #         self.process_video = False
    #         self.video_capture.release()

    def binary_image_callback(self, msg):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        fps = 1 / elapsed_time
        self.get_logger().info('FPS: %f' % fps)

        self.image_index += 1
        if self.image_index < len(self.image_files):
            self.publish_next_image()
        else:
            self.get_logger().info('All images have been processed.')

    def publish_next_image(self):
        time.sleep(0.3)
        image_file = self.image_files[self.image_index]
        cv_image = cv2.imread(image_file)
        msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')

        # Add timestamp
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        self.get_logger().info('Published image: "%s"' % image_file)

def main(args=None):
    rclpy.init(args=args)

    image_files = glob.glob('/home/yh/datasets/sidewalk/images/train/*.jpg')  # Adjust file extension if needed
    if len(image_files) == 0:
        print('No images found in the specified folder')
        return

    image_pubsub = ImagePubSub(image_files)
    image_pubsub.publish_next_image()

    # video_file = '/home/yh/datasets/IMG_7076.mov'  # Replace with the path to your video file

    # if not os.path.isfile(video_file):
    #     print('Video file not found.')
    #     return

    # image_pubsub = ImagePubSub(video_file)
    # image_pubsub.publish_next_frame()


    rclpy.spin(image_pubsub)
    image_pubsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

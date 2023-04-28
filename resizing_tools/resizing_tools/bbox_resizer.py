import cv2
import rclpy
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError

from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

from rclpy.node import Node
from rclpy.qos import QoSProfile

class BBoxResizer(Node):
    QUEUE_SIZE = 10

    def __init__(self):
        super().__init__('bbox_resizer')

        self.bridge = CvBridge()
        self.declare_parameter("width", 600)
        self.declare_parameter("height", 600)

        self._detections_subscription = message_filters.Subscriber(
            self,
            Detection2DArray,
            # 'detectnet/detections')
            '/object_detections')

        self._image_subscription = message_filters.Subscriber(
            self,
            Image,
            # 'image')
            'camera/color/image_raw')

        self.time_synchronizer = message_filters.TimeSynchronizer(
            [self._detections_subscription, self._image_subscription],
            self.QUEUE_SIZE)

        self.time_synchronizer.registerCallback(self.callback)

        qos_profile = QoSProfile(depth=1)
        self.pub = self.create_publisher(Detection2DArray, '/resize/bbox', qos_profile=qos_profile)

    def callback(self, detections_msg, img_msg):
        image = self.bridge.imgmsg_to_cv2(img_msg)

        original_height, original_width, _ = image.shape
        # Get the desired size from the parameters
        desired_width = self.get_parameter("width").value
        desired_height = self.get_parameter("height").value

        desired_img = [desired_height, desired_width]

        detections_arr = Detection2DArray()


        for detection in detections_msg.detections:
            # object_hypothesis = ObjectHypothesisWithPose()

            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            width = detection.bbox.size_x
            height = detection.bbox.size_y
            id = detection.id
            class_id = detection.results[0].hypothesis.class_id
            score = detection.results[0].hypothesis.score


            origin_bbox = self.original_bbox(desired_img, image, [center_x, center_y, width, height])

            obj = Detection2D()
            obj.bbox.size_x = origin_bbox[2]
            obj.bbox.size_y = origin_bbox[3]
            obj.bbox.center.position.x = origin_bbox[0]
            obj.bbox.center.position.y = origin_bbox[1]
            obj.id = id
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_id
            hyp.hypothesis.score = score
            obj.results.append(hyp)
            detections_arr.detections.append(obj)

        detections_arr.header = detections_msg.header

        for det in detections_arr.detections:
                    det.header = detections_msg.header

        self.pub.publish(detections_arr)

    def original_bbox(self, resized_size, original_img, bbox):

        height, width = resized_size
        original_height, original_width = original_img.shape[:2]
        # scale = max(original_height, original_width) / max(height, width)
        scale_width = original_width / width
        scale_height = original_height / height

        center_x, center_y, w, h = bbox

        margin_width = abs(width - original_width)
        margin_height = abs(height - original_height)

        aspect_ratio = original_width / original_height

        new_width = int(height * aspect_ratio)
        new_height = int(width / aspect_ratio)

        if (width / height) > aspect_ratio:
            # resized_image = cv2.resize(original_img, (new_width, height))
            margin = int(abs((width - new_width) / 2))
            # margin = width / new_width
            # w = w / (1 - margin * 2)
            center_x = center_x - margin

            resize_ratio = height / original_height

        else:
            # resized_image = cv2.resize(original_img, (width, new_height))
            margin = int(abs((height - new_height) / 2))
            # margin = height / new_height

            # h = h / (1 - margin * 2)
            center_y = center_y - margin

            resize_ratio = width / original_width

        # # remove the margin from the bounding box
        # w = int(w / (1 - margin_width * 2))
        # h = int(h / (1 - margin_height * 2))
        # center_x = (center_x + (w * margin_width))
        # center_y = (center_y + (h * margin_height))

        # resize the bounding box
        w = float(w / resize_ratio)
        h = float(h / resize_ratio)
        center_x = float(center_x / resize_ratio)
        center_y = float(center_y / resize_ratio)

        return [center_x, center_y, w, h]

def main(args=None):
    rclpy.init(args=args)
    bbox_resizer = BBoxResizer()

    rclpy.spin(bbox_resizer)

    bbox_resizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

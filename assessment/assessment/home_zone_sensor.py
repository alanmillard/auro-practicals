import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Image
from assessment_interfaces.msg import HomeZone

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class HomeZoneSensor(Node):

    def __init__(self):
        super().__init__('home_zone_sensor')

        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.home_zone_publisher = self.create_publisher(HomeZone, 'home_zone', 10)
        self.image_publisher = self.create_publisher(Image, 'camera/image_home_zone', 10)


    def image_callback(self, data):

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info(f"CvBridgeError: {e}")

        image_width = frame.shape[1]
        image_height = frame.shape[0]

        augmented = frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)        

        SATURATION_LOWER = 100
        SATURATION_UPPER = 255
        VALUE_LOWER = 20
        VALUE_UPPER = 255

        lower = np.array([80, SATURATION_LOWER, VALUE_LOWER])
        upper = np.array([100, SATURATION_UPPER, VALUE_UPPER])

        mask = cv2.inRange(hsv, lower, upper)
        moments = cv2.moments(mask)

        msg = HomeZone()

        if moments["m00"] != 0:
            centre_x = int(moments["m10"] / moments["m00"])
            centre_y = int(moments["m01"] / moments["m00"])

            black = (0, 0, 0)
            white = (255, 255, 255)

            text = "HOME"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            font_thickness = 2
            text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
            text_position = (int(centre_x - text_size[0] / 2), int(centre_y + text_size[1] * 2))

            cv2.putText(augmented, text, text_position, font, font_scale, black, font_thickness * 4, cv2.LINE_AA)
            cv2.putText(augmented, text, text_position, font, font_scale, white, font_thickness, cv2.LINE_AA)

            cv2.circle(augmented, (centre_x, centre_y), 10, black, -1, lineType=cv2.LINE_AA)
            cv2.circle(augmented, (centre_x, centre_y), 6, white, -1, lineType=cv2.LINE_AA)

            msg.visible = True
            msg.x = int((image_width / 2) - centre_x)
            msg.y = int((image_height / 2) - centre_y)
            msg.size = np.count_nonzero(mask) / (image_width * image_height)
        else:
            msg.visible = False
            msg.x = 0
            msg.y = 0
            msg.size = 0.0

        self.home_zone_publisher.publish(msg)

        try:
            image_msg = self.bridge.cv2_to_imgmsg(augmented, encoding="bgr8")
            image_msg.header = data.header
            self.image_publisher.publish(image_msg)
        except CvBridgeError as e:
            self.get_logger().info(f"CvBridgeError: {e}")


def main(args=None):

    rclpy.init(args = args)

    node = HomeZoneSensor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
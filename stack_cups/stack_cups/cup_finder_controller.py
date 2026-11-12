import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

MIN_AREA = 100
IMAGE_TOPIC = "/camera/color/image_raw"

CENTER_THRESH = 2  # pixels of dead zone
KP = 0.009  # m/s per pixel of error
MAX_SPEED = 0.22


class FindCup(Node):
    def __init__(self):
        super().__init__("FindCup")
        self.bridge = CvBridge()
        self.create_subscription(
            Image, IMAGE_TOPIC, self.snapshot, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

    # Callback for camera
    def snapshot(self, msg):
        # Capture cv
        # Figure out direction
        # Threshold

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower1, upper1 = (
            np.array([0, 120, 70], np.uint8),
            np.array([10, 255, 255], np.uint8),
        )
        lower2, upper2 = (
            np.array([170, 120, 70], np.uint8),
            np.array([180, 255, 255], np.uint8),
        )
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2)
        )

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(biggest) > MIN_AREA:
                x, y, w, h = cv2.boundingRect(biggest)
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.get_logger().info(
                    f"Cup at pixel ({cx}, {cy})", throttle_duration_sec=1.0
                )
                # 120 is half the width of the camera (which is flipped on its head)
                error = cy - 120
                msg = Twist()
                if abs(error) > CENTER_THRESH:
                    # Sets velocity to be proportional to the distance of the cup
                    msg.linear.x = float(np.clip(KP * error, -MAX_SPEED, MAX_SPEED))
                self.pub.publish(msg)
        # Debugging messages
        cv2.imshow("red cup", img)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)


def main():
    rclpy.init()
    rclpy.spin(FindCup())


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

def detect_yellow_balls(image):
    # Convert the image to HSV
    im_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Define the range of yellow color in HSV
    lower_yellow = np.array([22, 93, 0])
    upper_yellow = np.array([55, 255, 255])
    r = 1
    # Create a mask for the yellow pixels
    mask = cv2.inRange(im_hsv, lower_yellow, upper_yellow)

    # Apply morphological operations to remove noise
    # kernel = np.ones((5,5),np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (r ,r))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Get contours in mask image
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize an empty list to store the circles' information
    balls = []
    # Iterate over the contours
    for c in contours:
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        area = cv2.contourArea(c)
        # Append the circle's information to the list
        x, y, radius = int(x), int(y), int(radius)
        balls.append((x, y, radius, area))

    if balls:
        return True, balls
    else:
        return False, None

class Camera(Node):

    def __init__(self):
        super().__init__('camera')
        self.subscription_image = self.create_subscription(Image, "/zenith_camera/image_raw", self.image_callback, 10)
        #Image
        self.image = []
        self.get_logger().info(self.get_name() + " is launched")

    def image_callback(self, msg):
        # self.get_logger().info('here')
        try:
            self.image = bridge.imgmsg_to_cv2(msg, "bgr8")
            # self.image = cv2.imread('/home/am/Desktop/yellow_ball.jpg')
            detected, balls = detect_yellow_balls(self.image)
            if detected:
                for x, y, radius, area in balls:
                    self.get_logger().info(f"Yellow balls detected at: (x,y) = ({x},{y}) with r = {radius}")
                    cv2.circle(self.image, (x, y), radius, (0, 0, 255), -1)
                    cv2.rectangle(self.image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                cv2.imshow("Image window", self.image)
                cv2.waitKey(1)  # it will prevent the window from freezing
        except CvBridgeError as e:
            self.get_logger().error(self.get_name() + ": Cannot convert message " + str(e))

    # def detect_yellow_balls(self):
    #         ## Params
    #         lower_yellow = np.array([22, 93, 0])
    #         upper_yellow = np.array([55, 255, 255])
    #         r = 1
    #
    #         ## detection
    #         #rgb to hsv
    #         im_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    #         #mask with lower/upper values
    #         mask = cv2.inRange(im_hsv, lower_yellow, upper_yellow)
    #
    #         ## Yellow ball detection (1/2)
    #         # create a disk mask
    #         kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (r ,r))
    #         # apply an opening to the mask
    #         mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    #         # get contours in mask image
    #         contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #
    #         ## localization
    #         if len(contours):
    #             # get area values
    #             areas = [cv2.contourArea(c) for c in contours]
    #             # get maximum area
    #             c = max(contours, key=cv2.contourArea)
    #             # get coordinates and radius of largest area
    #             ((_, _), radius) = cv2.minEnclosingCircle(c)
    #             M = cv2.moments(c)
    #             x = int(M["m10"] / M["m00"])
    #             y = int(M["m01"] / M["m00"])
    #
    #             cv2.circle(self.image, (int(x), int(y)), int(radius), (0, 0, 255), -1)
    #
    #             return True, x, y, radius
    #
    #         return False, None, None, None







def main(args=None):
    rclpy.init(args=args)

    camera = Camera()
    print("Starting the program..")

    while rclpy.ok():
        #Reading nodes
        rclpy.spin_once(camera)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # it will prevent the window from remaining open after the program ends
    print("~~ End of the program ~~")


if __name__ == '__main__':
    main()

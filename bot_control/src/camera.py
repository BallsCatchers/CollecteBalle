#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
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
        balls.append((x, y, radius, area))

    if balls:
        return True, balls
    else:
        return False, None


def detect_base(img_RGB):
    ## Params
    lower_orange = np.array([5, 50, 50])
    upper_orange = np.array([15, 255, 255])

    ## detection
    #rgb to hsv
    im_hsv = cv2.cvtColor(img_RGB, cv2.COLOR_BGR2HSV)
    #mask with lower/upper values
    mask = cv2.inRange(im_hsv, lower_orange, upper_orange)

    ## Orange base detection (1/2)
    # get contours in mask image
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    base = []
    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        # Append the rectangle's information to the list
        base.append((x, y, w, h))

    if base:
        return True, base
    else:
        return False, None


class Camera(Node):

    def __init__(self):
        super().__init__('camera')
        self.subscription_image = self.create_subscription(Image, "/zenith_camera/image_raw", self.image_callback, 10)
        self.publisher_balls = self.create_publisher(Float64MultiArray, '/tennis_balls', 10)
        self.publisher_base = self.create_publisher(Float64MultiArray, '/bases', 10)

        #Image
        self.image = []
        self.get_logger().info(self.get_name() + " is launched")

    def image_callback(self, msg):
        # self.get_logger().info('here')
        try:
            self.image = bridge.imgmsg_to_cv2(msg, "bgr8")
            balls_detected, balls = detect_yellow_balls(self.image)
            if balls_detected:
                ball_positions = []
                for x, y, radius, area in balls:
                    self.get_logger().info(f"Yellow balls detected at: (x,y) = ({x},{y}) with r = {radius}")

                    cv2.circle(self.image, (int(x), int(y)), int(radius), (0, 0, 255), -1)
                    cv2.rectangle(self.image, (int(x) - 5, int(y) - 5), (int(x) + 5, int(y) + 5), (0, 128, 255), -1)

                     # Add the x and y coordinates of each ball to the ball_positions list
                    ball_positions.append(x)
                    ball_positions.append(y)

            base_detected, base = detect_base(self.image)
            if base_detected:
                base_positions = []
                for x, y, w, h in base:
                    self.get_logger().info(f"\nBase detected at: (x,y) = ({x},{y}) with size (w,h) = ({w},{h})\n")

                    cv2.rectangle(self.image,(int(x),int(y)),(int(x)+int(w),int(y)+int(h)),(255,0,0),2)

                     # Add the x and y coordinates of each ball to the ball_positions list
                    base_positions.append(x)
                    base_positions.append(y)

                self.image_publisher(ball_positions, base_positions)

            cv2.imshow("Image window", self.image)
            cv2.waitKey(1)  # it will prevent the window from freezing

        except CvBridgeError as e:
            self.get_logger().error(self.get_name() + ": Cannot convert message " + str(e))

    def image_publisher(self, ball_positions, base_positions):
        if ball_positions != []:
            # Create the Float64MultiArray message for the balls
            msg_balls = Float64MultiArray()
            msg_balls.data = ball_positions
            # Publish the message
            self.publisher_balls.publish(msg_balls)

        elif base_positions != []:
            # Create the Float64MultiArray message for the balls
            msg_base = Float64MultiArray()
            msg_base.data = ball_positions
            # Publish the message
            self.publisher_base.publish(msg_base)

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

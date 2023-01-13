#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
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
    r = 5
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

<<<<<<< HEAD
def detect_marker(img_RGB):
    # Define the range of colors for red and green markers
    lower_red = np.array([0, 0, 200])
    upper_red = np.array([50, 50, 255])
    lower_green = np.array([30, 100, 30])
    upper_green = np.array([90, 255, 90])

    # Convert image from BGR to HSV color space
    img_HSV = cv2.cvtColor(img_RGB, cv2.COLOR_BGR2HSV)

    # Create masks for red and green markers
    red_mask = cv2.inRange(img_HSV, lower_red, upper_red)
    green_mask = cv2.inRange(img_HSV, lower_green, upper_green)

    # Get the contours of the markers
    
import math

def detect_marker(img_RGB):
    r = 5
    im_hsv = cv2.cvtColor(img_RGB, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (r ,r))

    # Define the range of colors for red and green markers
    # Create a mask for the red and green pixels
    lower_red = np.array([0, 0, 200])
    upper_red = np.array([50, 50, 255])
    red_mask = cv2.inRange(im_hsv, lower_red, upper_red)

    lower_green = np.array([30, 100, 30])
    upper_green = np.array([90, 255, 90])
    green_mask = cv2.inRange(im_hsv, lower_green, upper_green)

    # Apply morphological operations to remove noise
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

    # Get contours in mask image
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables to store the position of the markers
    red_x, red_y, green_x, green_y = None, None, None, None

    # Iterate over the contours and find the center of each marker
    for c in red_contours:
        M = cv2.moments(c)
        if M["m00"] != 0:
            red_x = int(M["m10"] / M["m00"])
            red_y = int(M["m01"] / M["m00"])
    for c in green_contours:
        M = cv2.moments(c)
        if M["m00"] != 0:
            green_x = int(M["m10"] / M["m00"])
            green_y = int(M["m01"] / M["m00"])

    # If both markers are found, calculate the orientation of the robot
    if red_x is not None and green_x is not None and red_y is not None and green_y is not None:
<<<<<<< HEAD
=======
        print(red_x, red_y, green_x, green_y)
>>>>>>> devel
        position_x = (green_x + red_x)/2
        position_y = (green_y + red_y)/2
        orientation = math.atan2(green_y - red_y, green_x - red_x)
        return True, position_x, position_y, orientation

    return False, None, None, None

<<<<<<< HEAD
=======

>>>>>>> devel
class Camera(Node):

    def __init__(self):
        super().__init__('camera')
        self.subscription_image = self.create_subscription(Image, "/zenith_camera/image_raw", self.image_callback, 10)
        self.publisher_balls = self.create_publisher(Float64MultiArray, '/tennis_balls', 10)
        self.publisher_base = self.create_publisher(Float64MultiArray, '/bases', 10)
        self.publisher_robot = self.create_publisher(Vector3, '/bot_pos', 10)

        #Image
        self.image = []
        self.get_logger().info(self.get_name() + " is launched")

    def image_callback(self, msg):
        # self.get_logger().info('here')
        try:
            self.image = bridge.imgmsg_to_cv2(msg, "bgr8")
            balls_detected, balls = detect_yellow_balls(self.image)
            ball_positions = []
            if balls_detected:
                for x, y, radius, area in balls:
                    # self.get_logger().info(f"Yellow balls detected at: (x,y) = ({x},{y}) with r = {radius}")

                    # cv2.circle(self.image, (int(x), int(y)), int(radius), (0, 0, 255), -1)
                    # cv2.rectangle(self.image, (int(x) - 5, int(y) - 5), (int(x) + 5, int(y) + 5), (0, 128, 255), -1)

                     # Add the x and y coordinates of each ball to the ball_positions list
                    ball_positions.append(x)
                    ball_positions.append(y)

            base_detected, base = detect_base(self.image)
            base_positions = []
            if base_detected:
                for x, y, w, h in base:
                    # self.get_logger().info(f"\nBase detected at: (x,y) = ({x},{y}) with size (w,h) = ({w},{h})\n")

                    # cv2.rectangle(self.image,(int(x),int(y)),(int(x)+int(w),int(y)+int(h)),(255,0,0),2)

                     # Add the x and y coordinates of each ball to the ball_positions list
                    base_positions.append(x)
                    base_positions.append(y)


            marker_detected, position_x, position_y, orientation = detect_marker(self.image)
            robot_position = []
            if marker_detected:
<<<<<<< HEAD
=======
                self.get_logger().info(f"\Robot detected at: (x,y, theta) = ({position_x},{position_y},{orientation}")
>>>>>>> devel
                self.get_logger().info(f"\nOrientation : {position_x}\n")
                self.get_logger().info(f"\nOrientation : {position_y}\n")
                self.get_logger().info(f"\nOrientation : {orientation}\n")
                robot_position = [position_x, position_y, orientation]
<<<<<<< HEAD
=======
                cv2.rectangle(self.image,(int(position_x),int(position_y)),(5,5),(255,0,0),2)
                cv2.rectangle(self.image,(int(position_x + math.cos(orientation)*10),\
                              int(position_y + math.sin(orientation)*10)),(5,5),(255,0,0),2)
>>>>>>> devel

            self.image_publisher(ball_positions, base_positions, robot_position)

            cv2.imshow("Image window", self.image)
            cv2.waitKey(1)  # it will prevent the window from freezing

        except CvBridgeError as e:
            self.get_logger().error(self.get_name() + ": Cannot convert message " + str(e))

    def image_publisher(self, ball_positions, base_positions, robot_position):
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

        elif robot_position != []:
            msg_orientation = Vector3()
            msg_orientation.x = robot_position[0]
            msg_orientation.y = robot_position[1]
            msg_orientation.z = robot_position[2]
            self.publisher_base.publish(msg_orientation)

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

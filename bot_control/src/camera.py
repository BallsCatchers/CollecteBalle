#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
import time

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
        return balls
    else:
        return []

def detect_base(img_RGB):
    ## Params
    lower_orange = np.array([10, 200, 130])
    upper_orange = np.array([40, 230, 150])

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
        if w > 10: # Avoid unwanted detection
            base.append((x, y, w, h))

    if base:
        return True, base
    else:
        return False, None

def detect_marker(img_RGB):
    r = 5
    im_hsv = cv2.cvtColor(img_RGB, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (r, r))

    # Define the range of colors for red and green markers
    # Create a mask for the red and green pixels
    lower_red = np.array([0, 215, 120])
    upper_red = np.array([25, 255, 150])
    red_mask = cv2.inRange(im_hsv, lower_red, upper_red)

    lower_green = np.array([50, 190, 120])
    upper_green = np.array([90, 255, 160])
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
        position_x = (green_x + red_x)/2
        position_y = (green_y + red_y)/2
        orientation = math.atan2(green_y - red_y, green_x - red_x)
        orientation = (- orientation + math.pi) % (2 * math.pi)
        return True, position_x, position_y, orientation

    return False, None, None, None

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

        #Balls
        self.balls = []

    def dist(self,x,y,ball):
        return np.sqrt((ball[1]-x)**2 + (ball[2]-y)**2)

    def image_callback(self, msg):
        try:
            self.image = bridge.imgmsg_to_cv2(msg, "bgr8")
            balls = detect_yellow_balls(self.image)
            ball_positions = []
            new_balls = []
            idx_to_remove = []
            to_display = []

            #Evite les doublons dans self.balls
            unique_balls = set()
            for i in range(len(self.balls)):
                if self.balls[i][0] not in unique_balls:
                    unique_balls.add(self.balls[i][0])
                    self.balls[i] = self.balls[i]
                else:
                    self.balls.pop(i)

            #On cherche les nouvelles balles et on les ajoute
            for i in range(len(balls)):
                x, y, radius, area = balls[i][0], balls[i][1], balls[i][2], balls[i][3]

                #Affichage
                ball_positions.append(x)
                ball_positions.append(y)

                # Check if this is a new ball
                if len(self.balls) < len(ball_positions)//2:
                    print("adding new ball..")
                    d_min = np.inf
                    for j in range(len(self.balls)):
                        d = np.sqrt((self.balls[j][1]-x)**2 + (self.balls[j][2]-y)**2)
                        if d < d_min:
                            d_min = d
                    if d_min > 1:
                        print("new ball added !")
                        self.balls.append([time.time(), x, y, radius])
                    else:
                        print("fail to add the ball, d_min ", d_min)


            # Find closest ball in self.balls
            if len(balls) == len(self.balls): #Si il a bien détecter toutes les balles
                for i in range(len(self.balls)-1,-1,-1):
                    d_min = np.inf
                    xmin = 0; ymin = 0; rmin = 0;
                    for j in range(len(balls)):
                        x, y, radius, area = balls[j][0], balls[j][1], balls[j][2], balls[j][3]
                        d = np.sqrt((self.balls[i][1]-x)**2 + (self.balls[i][2]-y)**2)
                        if d < d_min:
                            d_min = d
                            xmin = x; ymin = y; rmin = radius
                    # if d_min < 50:
                    to_display.append([i, xmin, ymin, rmin])
                    # else:
                    #     idx_to_remove.append(i)

            else:
                print("List doesn't have the same lenghts")


            for i in range(len(to_display)):
                idx_i = to_display[i][0]; x_i = to_display[i][1]; y_i = to_display[i][2]
                for j in range(i+1,len(to_display)):
                    idx_j = to_display[j][0]; x_j = to_display[j][1]; y_j = to_display[j][2]
                    if x_i == x_j and y_i == y_j:
                        print("superposition")
                        if self.balls[idx_i][0] > self.balls[idx_j][0]:
                            if idx_j not in idx_to_remove:
                                idx_to_remove.append(idx_j)
                        else:
                            if idx_i not in idx_to_remove:
                                idx_to_remove.append(idx_i)

            for i in range(len(to_display)):
                if i not in idx_to_remove:
                    idx_to_display = to_display[i][0]
                    xmin = to_display[i][1]
                    ymin = to_display[i][2]
                    rmin = to_display[i][3]
                    self.balls[idx_to_display] = [self.balls[idx_to_display][0], xmin, ymin, rmin]

            # Remove balls that were not updated
            for i in idx_to_remove:
                self.balls.pop(i)
                print("indice to remove: ",i)


            #Affichage
            for j in range(len(balls)):
                x, y, radius, area = balls[j][0], balls[j][1], balls[j][2], balls[j][3]
                cv2.circle(self.image, (int(x), int(y)), int(radius), (0, 0, 255), -1)
                cv2.rectangle(self.image, (int(x) - 5, int(y) - 5), (int(x) + 5, int(y) + 5), (0, 128, 255), -1)

            for i in range(len(self.balls)):
                t = self.balls[i][0]
                x = self.balls[i][1]
                y = self.balls[i][2]
                radius = self.balls[i][3]

                cv2.putText(self.image, "Ball " + str(i), (int(x-radius), int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(self.image, "Time: " + str(time.time()-self.balls[i][0]), (int(x-radius), int(y-radius+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

            base_detected, base = detect_base(self.image)
            base_positions = []
            if base_detected:
                for x, y, w, h in base:
                    # self.get_logger().info(f"\nBase detected at: (x,y) = ({x},{y}) with size (w,h) = ({w},{h})\n")

                    cv2.rectangle(self.image,(int(x),int(y)),(int(x)+int(w),int(y)+int(h)),(255,0,0),2)

                     # Add the x and y coordinates of each ball to the ball_positions list
                    base_positions.append(x * 1.0)
                    base_positions.append(y * 1.0)


            marker_detected, position_x, position_y, orientation = detect_marker(self.image)
            robot_position = []
            if marker_detected:
                # self.get_logger().info(f"\Robot detected at: (x,y, theta) = ({position_x},{position_y},{orientation}")
                # self.get_logger().info(f"\nx : {position_x}\n")
                # self.get_logger().info(f"\ny : {position_y}\n")
                # self.get_logger().info(f"\nOrientation : {orientation*180./math.pi}\n")
                robot_position = [position_x, position_y, orientation]
                cv2.rectangle(self.image,(int(position_x)-2,int(position_y)-2),(int(position_x)+2,int(position_y)+2),(255,0,0),2)
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

        if base_positions != []:
            # Create the Float64MultiArray message for the base
            msg_base = Float64MultiArray()
            msg_base.data = base_positions
            # Publish the message
            self.publisher_base.publish(msg_base)

        if robot_position != []:
            msg_orientation = Vector3()
            msg_orientation.x = robot_position[0]
            msg_orientation.y = robot_position[1]
            msg_orientation.z = robot_position[2]
            self.publisher_robot.publish(msg_orientation)

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

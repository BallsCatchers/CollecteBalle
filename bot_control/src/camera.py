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
        # orientation = (- orientation + math.pi) % (2 * math.pi)
        return True, position_x, position_y, orientation

    return False, None, None, None

def detect_net(img_RGB):

    # Obtenir la taille de l'image
    hauteur, largeur, _ = img_RGB.shape

    print(f"hauteur = {hauteur}; largeur = {largeur}")

    # Définir les coordonnées du rectangle du filet
    x_min = int(largeur / 2 - largeur * 0.005)
    x_max = int(largeur / 2 + largeur * 0.005)
    y_min = int(hauteur * 0.14)
    y_max = int(hauteur * 0.86)

    net = [x_min, y_min, x_max, y_max]

    doors_up_right = (int(largeur / 2 - largeur * 0.08), int(hauteur * 0.2)//2)
    doors_up_left = (int(largeur / 2 + largeur * 0.08), int(hauteur * 0.2)//2)
    doors_up = [doors_up_left, doors_up_right]

    doors_down_right = (int(largeur / 2 - largeur * 0.08) , (int(hauteur) + int(hauteur * 0.8))//2)
    doors_down_left = (int(largeur / 2 + largeur * 0.08) , (int(hauteur) + int(hauteur * 0.8))//2)
    doors_down = [doors_down_left, doors_down_right]

    return(net, doors_up, doors_down)



class Camera(Node):

    def __init__(self):
        super().__init__('camera')
        self.subscription_image = self.create_subscription(Image, "/zenith_camera/image_raw", self.image_callback, 10)
        self.publisher_balls = self.create_publisher(Float64MultiArray, '/tennis_balls', 10)
        self.publisher_base = self.create_publisher(Float64MultiArray, '/bases', 10)
        self.publisher_robot = self.create_publisher(Vector3, '/bot_pos', 10)

        self.publisher_goal = self.create_publisher(Vector3, '/ball_goal', 10)


        #Image
        self.input_received = False
        self.image = []
        #Balls
        self.balls = []



        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")


    def dist(self,x,y,ball):
        return np.sqrt((ball[1]-x)**2 + (ball[2]-y)**2)


    def image_callback(self, msg):
        try:
            self.image = bridge.imgmsg_to_cv2(msg, "bgr8")
            self.input_received = True
            # self.get_logger().info(self.get_name() + " got img")

        except CvBridgeError:
            self.get_logger().info(self.get_name() + " CvBridgeError !! Cannot convert img from ros msg to CV2 !")
            pass

    def go_to_target(self, robot_position):
        # self.get_logger().info(f"\Robot detected at: (x,y, theta) = ({position_x},{position_y},{orientation*180./math.pi}")
        position_x, position_y, orientation = [i for i in robot_position]

        cv2.rectangle(self.image,(int(position_x)-2,int(position_y)-2),(int(position_x)+2,int(position_y)+2),(255,0,0),2)


        # Dessiner le rectangle du filet sur l'image et les portes
        net, doors_up, doors_down = detect_net(self.image)
        x_min, y_min, x_max, y_max = [i for i in net]
        doors_up_right, doors_up_left = [i for i in doors_up]
        doors_down_right, doors_down_left = [i for i in doors_down]

        #Filet
        cv2.rectangle(self.image, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

        #Portes
        cv2.circle(self.image, (int(doors_up_right[0]), int(doors_up_right[1])), int(10), (255, 0, 0), -1)
        cv2.putText(self.image, "doors_up_right", (int(doors_up_right[0]), int(doors_up_right[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.circle(self.image, (int(doors_up_left[0]), int(doors_up_left[1])), int(10), (255, 0, 0), -1)
        cv2.putText(self.image, "doors_up_left", (int(doors_up_left[0]), int(doors_up_left[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.circle(self.image, (int(doors_down_right[0]), int(doors_down_right[1])), int(10), (255, 0, 0), -1)
        cv2.putText(self.image, "doors_down_right", (int(doors_down_right[0]), int(doors_down_right[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.circle(self.image, (int(doors_down_left[0]), int(doors_down_left[1])), int(10), (255, 0, 0), -1)
        cv2.putText(self.image, "doors_down_left", (int(doors_down_left[0]), int(doors_down_left[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        if len(self.balls) > 0:
            xg, yg = self.balls[0][1], self.balls[0][2]
            hauteur = (y_min + y_max); largeur = (x_min + x_max)
            print(xg,yg)

            if largeur/2 - position_x > 0: #Le robot est à gauche
                if largeur/2 - xg > 0: #La target est aussi à gauche
                    goal = (int(xg), int(yg))
                    cv2.arrowedLine(self.image, (int(position_x), int(position_y)), goal, (255, 0, 0), 2)
                else: #C'est pas du même côté
                    if hauteur/2 - yg > 0: #En haut à gauche
                        if hauteur*0.1 - position_y > 0: #Très en haut à gauche
                            goal = (int(doors_up_right[0]), int(doors_up_right[1]))
                            cv2.arrowedLine(self.image, (int(position_x), int(position_y)), goal, (255, 0, 0), 2)
                        else:
                            goal = (int(doors_up_left[0]), int(doors_up_left[1]))
                            cv2.arrowedLine(self.image, (int(position_x), int(position_y)), goal, (255, 0, 0), 2)
                    else: #En bas à gauche
                        if hauteur*0.9 - position_y > 0: #Très en bas à gauche
                            goal = (int(doors_down_left[0]), int(doors_down_left[1]))
                            cv2.arrowedLine(self.image, (int(position_x), int(position_y)), goal, (255, 0, 0), 2)
                        else:
                            goal = (int(doors_down_right[0]), int(doors_down_right[1]))
                            cv2.arrowedLine(self.image, (int(position_x), int(position_y)), goal, (255, 0, 0), 2)

            else: #Le robot est à droite
                if largeur/2 - xg <= 0: #La target est aussi à droite
                    goal = (int(xg), int(yg))
                    cv2.arrowedLine(self.image, (int(position_x), int(position_y)), goal, (255, 0, 0), 2)
                else: #Pas du même côté
                    if hauteur/2 - yg > 0: #En haut à droite
                        if hauteur*0.1 - position_y > 0: #Très en haut à droite
                            goal = (int(doors_up_left[0]), int(doors_up_left[1]))
                            cv2.arrowedLine(self.image, (int(position_x), int(position_y)), goal, (255, 0, 0), 2)
                        else:
                            goal = (int(doors_up_right[0]), int(doors_up_right[1]))
                            cv2.arrowedLine(self.image, (int(position_x), int(position_y)), goal, (255, 0, 0), 2)
                    else:
                        if hauteur*0.9 - position_y > 0: #Très en bas à gauche
                            goal = (int(doors_down_right[0]), int(doors_down_right[1]))
                            cv2.arrowedLine(self.image, (int(position_x), int(position_y)), goal, (255, 0, 0), 2)
                        else:
                            goal = (int(doors_down_left[0]), int(doors_down_left[1]))
                            cv2.arrowedLine(self.image, (int(position_x), int(position_y)), goal, (255, 0, 0), 2)

            return(goal)
        return []


    def process(self):
        if self.input_received:
            balls = detect_yellow_balls(self.image)
            marker_detected, position_x, position_y, orientation = detect_marker(self.image)
            base_detected, base = detect_base(self.image)

            ball_positions, base_positions, robot_position = [], [], []

            # self.get_logger().info("\n=========================\n")
            # self.get_logger().info("Found " + str(len(balls)) + " balls in img")

            # self.balls = n_balls * [detection_time t0, pos_x, pos_y, updated (bool)]


            # Reset the update state of each balls
            for i in self.balls:
                i[3] = False

            # For each detected ball:
            for i in range(len(balls)):
                x, y, radius, area = balls[i][0], balls[i][1], balls[i][2], balls[i][3]

                #Publication
                ball_positions.append(x)
                ball_positions.append(y)

                # Reset of min values
                d_min = np.inf
                index_j = None

                for j in range(len(self.balls)):
                    d = np.sqrt((self.balls[j][1]-x)**2 + (self.balls[j][2]-y)**2)
                    if d < d_min:
                        d_min = d # Finding the minimal distance for matching
                        index_j = j # Getting the corresponding index in the already found list if it exist
                # self.get_logger().info("Params : d_min :" + str(d_min) + ", index of closest : " + str(index_j))
                if d_min > 3:
                    # self.get_logger().info("new ball added ! Pos : " + str(x) + ", " + str(y))
                    self.balls.append([time.time(), x, y, True])
                else:
                    # Update the corresponding ball in the self.balls list
                    # self.get_logger().info("Updated ball nb " + str(index_j))
                    self.balls[index_j][1] = x
                    self.balls[index_j][2] = y
                    self.balls[index_j][3] = True
                # else:
                #     print("fail to add the ball, d_min  : ", d_min, ', and pos : ', x, y)

            # Removing all the unupdated balls
            n_balls = [self.balls[i] for i in range(len(self.balls)) if self.balls[i][3]]
            self.balls = n_balls
            # self.get_logger().info(" length of new balls : " + str(len(n_balls)))


            # #Affichage
            for i in range(len(self.balls)):
                t = self.balls[i][0]
                x = self.balls[i][1]
                y = self.balls[i][2]
                radius = 4.

                cv2.circle(self.image, (int(x), int(y)), int(radius), (0, 0, 255), -1)
                cv2.putText(self.image, "Ball " + str(i+1), (int(x-radius-20), int(y-radius-20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(self.image, "Time: " + time.strftime('%Mmin %Ss', time.gmtime(time.time()-self.balls[i][0])), (int(x-radius-20), int(y-radius-20+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            if base_detected:
                for x, y, w, h in base:
                    # self.get_logger().info(f"\nBase detected at: (x,y) = ({x},{y}) with size (w,h) = ({w},{h})\n")
                    cv2.rectangle(self.image,(int(x),int(y)),(int(x)+int(w),int(y)+int(h)),(255,0,0),2)
                     # Add the x and y coordinates of each ball to the ball_positions list
                    base_positions.append(x * 1.0)
                    base_positions.append(y * 1.0)


            if marker_detected:
                # self.get_logger().info(f"\Robot detected at: (x,y, theta) = ({position_x},{position_y},{orientation*180./math.pi}")
                robot_position = [position_x, position_y, orientation]
                cv2.rectangle(self.image,(int(position_x)-2,int(position_y)-2),(int(position_x)+2,int(position_y)+2),(255,0,0),2)

                goal = self.go_to_target(robot_position)

                self.image_publisher(ball_positions, base_positions, robot_position, goal)

            cv2.imshow("Image window", self.image)
            cv2.waitKey(1)  # it will prevent the window from freezing
            self.input_received = False


    def image_publisher(self, ball_positions, base_positions, robot_position, goal):
        if ball_positions != []:
            # Create the Float64MultiArray message for the balls
            msg_balls = Float64MultiArray()
            msg_balls.data = ball_positions
            # Publish the message
            self.publisher_balls.publish(msg_balls)

            # goal_ball = Vector3()
            # oldest_ball = self.balls[0]
            # goal_ball.x = oldest_ball[1]
            # goal_ball.y = oldest_ball[2]
            # self.publisher_goal.publish(goal_ball)

            if goal != []:
                print(goal)
                print(goal[0])
                print(goal[1],'\n')
                goal_ball = Vector3()
                goal_ball.x = float(goal[0])
                goal_ball.y = float(goal[1])
                self.publisher_goal.publish(goal_ball)

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

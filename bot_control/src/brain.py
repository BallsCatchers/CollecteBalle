#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
import numpy as np

# from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

import math

def sawtooth(theta):
    return 2 * math.atan(math.tan(theta / 2))

class Main(Node):
    def __init__(self):
        super().__init__("MainBallCatcher")

        # ===============================
        # Subscriptions 
        # ===============================
        self.input_goal = self.create_subscription(Vector3, "/ball_goal", self.sub_goal, 10)
        self.__goal = []
        self.__is_ball = True

        self.__bases = self.create_subscription(Float64MultiArray, "/bases", self.sub_bases, 10)
        self.__bases = []

        self.__pose = self.create_subscription(Vector3, "/bot_pos", self.get_pose, 10)
        self.__last_pose = []
        # ===============================
        # Publishers
        # ===============================
        self.trigger_pub = self.create_publisher(Bool, "/catch_trigger", 10)
        self.__trigger = Bool()
        self.__trigger.data = False


        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.__cmd_twist = Twist()


        self.state_pub = self.create_publisher(String, "/state", 10)
        # ===============================
        # Main
        # ===============================

        # Initialisation
        self.__state = "get_balls"
        self.__last_state = "get_balls"
        self.__chrono = time.time()

        # Unstuck function variables 
        self.__cooldown = time.time() # Cooldown timer between two unstuck procedures
        self.__unstuck_timer = time.time()  # Timer for the time to go in reverse
        self.__stuck_timer = time.time() # Timer for false stuck (ie: if stuck for > 5s, unstuck procedure)
        self.__wait_lock = False # Check for the stuck timer to unstuck procedure

        # Ball counting variables
        self.__t_last_detect = None
        self.__nb_balls = 0
        self.__catched_ball = True

        # Changing states variables
        self.__home = False
        self.__done = False

        # Initial (arbitrary) pos of the bot
        self.x = 0.
        self.y = 0.
        self.theta = 0.

        # Proportional coefs for movements 
        self.K1 = 0.00004 # Go front
        self.K2 = 2.6 # Rotation

        # Ros Node spinning
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")

    def sub_bases(self, msg):
        # self.get_logger().info(self.get_name() + " got bases")

        # Getting the bases values only if we detecte 2 rectangles
        if len(msg.data) == 8:
            self.__bases = msg.data
        # self.get_logger().info("Going home : " + str(len(self.__bases)) + ", first element : " + str(self.__bases[0]))



    def sub_goal(self, msg):
        # Getting the goal information
        self.__goal=[msg.x, msg.y] # Goal pos x,y in pixels

        # Getting the info if it's a ball, a door (intermediate goal) or if no goal
        if msg.z == 1:
            self.__is_ball = True
        elif msg.z == 0:
            self.__is_ball = False
        elif msg.z == 2:
            self.__state = "come_back"

        # self.get_logger().info(self.get_name() + " got goal !")


    def get_pose(self, msg):
        # self.get_logger().info(self.get_name() + " got pose")
        self.__last_pose = [self.x, self.y, self.theta]
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.z
    
    def move(self, dx, dy):
        err_x = (dx - self.x)
        err_y = (dy - self.y)
        d = err_x ** 2 + err_y ** 2
        # Calculating new speed, reducing it while turning and adding min/max thresholds
        v_x = float(max(min(self.K1 * d - 0.6*abs(self.__cmd_twist.angular.z), 2.), 0.7))

        # Closing the doors if too fast
        if v_x > 0.7 or not(self.__is_ball):
            self.__trigger.data = True
        else:
            self.__trigger.data = False

        # Prioritise turning
        if abs(self.__cmd_twist.angular.z) > 1. :
            v_x = 0.
        self.__cmd_twist.linear.x = v_x

    def move_no_balls(self, dx, dy):
        err_x = (dx - self.x)
        err_y = (dy - self.y)
        d = err_x ** 2 + err_y ** 2
        # Calculating new speed, reducing it while turning and adding min/max thresholds
        v_x = float(max(min(self.K1 * d - 0.6*abs(self.__cmd_twist.angular.z), 2.), 1.))

        # Closing the doors
        self.__trigger.data = True

        # Prioritise turning
        if abs(self.__cmd_twist.angular.z) > 1. :
            v_x = 0.
        self.__cmd_twist.linear.x = v_x

    def turn(self, dtheta):
        err_theta = sawtooth(self.theta - dtheta)
        cmd_angle = self.K2 * err_theta
        if err_theta > 0.4 or not(self.__is_ball):
            self.__trigger.data = True

        self.__cmd_twist.angular.z = cmd_angle

    def check_stuck(self):
        if self.__last_pose: # Check if we have a prior position
            v = np.sqrt((self.__last_pose[0] - self.x)**2 + (self.__last_pose[1] - self.y)**2)/ 0.1
            dtheta = self.__last_pose[2] - self.theta
            v += dtheta
            # self.get_logger().info(self.get_name() + " Current speed is : " + str(v) + " and cmd is " + str(self.__cmd_twist.linear.x))

            # Checking for cmd, actual speed, and cooldown
            if v == 0. and self.__cmd_twist.linear.x > 0 and (time.time() - self.__cooldown) > 10.:
                return True



    def process(self):

        chrono = time.time() - self.__chrono # For a timing switch of states
        x_filet = 640. # middle of the img, (can be setup auto if needed)

        stuck = self.check_stuck()

        # stuck check function
        if stuck and self.__state != "stuck" :
            self.__stuck_timer = time.time()
            self.__wait_lock = True
        else:
            self.__wait_lock = False

        # getting the param of the bases
        if len(self.__bases) == 8:
            if self.__bases[0] < x_filet:
                x_base_left = self.__bases[0]
                y_base_left = self.__bases[1] 
                x_base_right =self.__bases[6]
                y_base_right = self.__bases[7]
                w_left, h_left = self.__bases[2], self.__bases[3]
                w_right, h_right = self.__bases[4], self.__bases[5]
            else:
                x_base_right = self.__bases[0]
                y_base_right = self.__bases[1]
                x_base_left = self.__bases[4]
                y_base_left = self.__bases[5]
                w_right, h_right = self.__bases[2], self.__bases[3]
                w_left, h_left = self.__bases[6], self.__bases[7]


        # ===========================
        # State changing functions
        # ===========================

        # Unstuck procedure if stuck for > 5. s    
        if self.__state != "stuck" and time.time() - self.__stuck_timer > 5.  and self.__wait_lock:
            self.__cooldown = time.time()
            self.__unstuck_timer = time.time()
            self.__last_state = self.__state    
            self.__state = "stuck"
            self.__wait_lock = False
            self.get_logger().info(self.get_name() + " switched state to : " + self.__state)


        # getting the balls function, timer controled 
        if chrono >= 60.*2 and self.__state == "get_balls":
            self.__last_state = self.__state
            self.__state = "come_back"
            self.get_logger().info(self.get_name() + " switched state to : " + self.__state)

        # Going back to the base
        if self.__state == "come_back" and self.__home :
            self.__last_state = self.__state
            self.__state = "home"
            self.get_logger().info(self.get_name() + " switched state to : " + self.__state)
            self.__home = False

        # When home, release balls and reverse
        if self.__state == "home" and self.__done :
            self.__last_state = self.__state
            self.__state = "get_balls"
            self.__chrono = time.time()
            self.__done = False
            self.get_logger().info(self.get_name() + " switched state to : " + self.__state)

        msg_state = String()
        msg_state.data = self.__state
        self.trigger_pub.publish(self.__trigger)
        self.state_pub.publish(msg_state)

        # ===========================
        # Control function according to the current state
        # ===========================

        if self.__state == "get_balls":
            if len(self.__goal) != 0:
                x_target, y_target = self.__goal[0], self.__goal[1]
                angle = math.atan2(self.y - y_target, self.x - x_target)
                # print("Angle : ", angle*180./np.pi)
                # print("Bot : ", self.theta*180./np.pi)
                d = np.sqrt((x_target - self.x)**2 + (y_target - self.y)**2)
                if d < 13 :
                    self.__catched_ball = False
                    self.__t_last_detect = time.time()
                    self.get_logger().info(self.get_name() + " bot close to goal : " + str(d))

                if self.__t_last_detect != None :
                    if time.time() - self.__t_last_detect > 1.5 and not(self.__catched_ball): 
                        self.__nb_balls += 1
                        self.__catched_ball = True
                        self.get_logger().info(self.get_name() + " got 1 ball !")
                        self.get_logger().info(self.get_name() + " Nb of collected balls : " + str(self.__nb_balls))
                self.move(x_target, y_target)
                self.turn(angle)
        
        if self.__state == "come_back":
            self.__trigger.data = True

            if self.x < x_filet:
                # left
                # self.get_logger().info("Going Left : " + str(x_base_left < self.x <  w_left) + ", " + str(y_base_left < self.y <  h_left))
                mid_x, mid_y = (x_base_left + w_left)/2, (y_base_left + h_left)//2
                angle = math.atan2(self.y - mid_y, self.x - mid_x)
                self.turn(angle)
                self.move_no_balls(mid_x, mid_y)
                if x_base_left < self.x <  w_left and y_base_left < self.y <  h_left:
                    self.__home = True

            else:
                # right
                # self.get_logger().info("Going Right " + str(x_base_right < self.x < w_right and y_base_right < self.y < h_right))
                mid_x, mid_y = (x_base_right + w_right)/2, (y_base_right + h_right)//2
                angle = math.atan2(self.y - mid_y, self.x - mid_x)
                self.turn(angle)
                self.move_no_balls(mid_x, mid_y)
                if x_base_right < self.x < w_right and y_base_right < self.y < h_right:
                    self.__home = True


        if self.__state == "home":
            self.__trigger.data = False
            self.__cmd_twist.linear.x = -3.0

            if self.x < x_filet:
                if x_base_left < self.x <  w_left and y_base_left < self.y <  h_left:
                    self.__done = False
                    # self.get_logger().info(self.get_name() + " is inside the left base !")
                else :
                    self.__done = True

            else:
                if x_base_right < self.x < w_right and y_base_right < self.y < h_right:
                    self.__done = False
                    # self.get_logger().info(self.get_name() + " is inside the right base !")
                else :
                    self.__done = True


        if self.__state == "stuck":
            if time.time() - self.__unstuck_timer < 2.:
                self.__trigger.data = True
                self.__cmd_twist.linear.x = -3.
                self.__cmd_twist.angular.z = 0.
            else:
                self.__state = self.__last_state
                self.get_logger().info(self.get_name() + " now unstuck and ste is " + self.__state)




            
        self.twist_pub.publish(self.__cmd_twist)
        # self.get_logger().info("Twist published : " + str(self.__cmd_twist.linear.x) + ", " + str(self.__cmd_twist.linear.y) + ", " + str(self.__cmd_twist.angular.z))
        # self.get_logger().info(self.get_name() + " Running")
        # self.get_logger().info(self.get_name() + " : " + str(len(self.__bases)))






def main(args=None):    
    rclpy.init()
    my_py_node = Main()
    rclpy.spin(my_py_node)
    my_py_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
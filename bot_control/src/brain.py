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

        self.__bases = self.create_subscription(Float64MultiArray, "/bases", self.sub_bases, 10)
        self.__bases = []

        self.__pose = self.create_subscription(Vector3, "/bot_pos", self.get_pose, 10)

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
        self.__state = "get_balls"
        self.__chrono = time.time()


        self.__t_last_detect = None
        self.__nb_balls = 0
        self.__home = False
        self.__done = False

        self.x = 0.
        self.y = 0.
        self.theta = 0.

        self.K1 = 0.00005
        self.K2 = 2.4

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")

    def sub_bases(self, msg):
        # self.get_logger().info(self.get_name() + " got bases")
        self.__bases = msg.data
        # self.get_logger().info("Going home : " + str(len(self.__bases)) + ", first element : " + str(self.__bases[0]))



    def sub_goal(self, msg):
        self.__goal=[msg.x, msg.y]
        # self.get_logger().info(self.get_name() + " got goal !")


    def get_pose(self, msg):
        # self.get_logger().info(self.get_name() + " got pose")
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.z
    
    def move(self, dx, dy):
        err_x = (dx - self.x)
        err_y = (dy - self.y)
        d = err_x ** 2 + err_y ** 2
        v_x = float(max(min(self.K1 * d, 1.6), 0.6))
        self.__cmd_twist.linear.x = v_x
        if v_x > 0.75:
            self.__trigger.data = True
        else:
            self.__trigger.data = False
        # self.__cmd_twist.linear.y = self.K1 * d


    def turn(self, dtheta):
        err_theta = sawtooth(self.theta - dtheta)
        self.__cmd_twist.angular.z = self.K2 * err_theta

    def process(self):

        chrono = time.time() - self.__chrono


        if chrono >= 45.*1 and self.__state == "get_balls":
            self.__state = "come_back"
            self.get_logger().info(self.get_name() + " switched state to : " + self.__state)

        if self.__state == "come_back" and self.__home :
            self.__state = "home"
            self.get_logger().info(self.get_name() + " switched state to : " + self.__state)
            self.__home = False

        if self.__state == "home" and self.__done :
            self.get_logger().info(self.get_name() + " switched state to : " + self.__state)
            self.__state = "get_balls"
            self.__chrono = time.time()
            self.__done = False


        msg_state = String()
        msg_state.data = self.__state
        self.trigger_pub.publish(self.__trigger)
        self.state_pub.publish(msg_state)

        if self.__state == "get_balls":
            if len(self.__goal) != 0:
                x_target, y_target = self.__goal[0], self.__goal[1]
                angle = math.atan2(self.y - y_target, self.x - x_target)
                # print("Angle : ", angle*180./np.pi)
                # print("Bot : ", self.theta*180./np.pi)
                # d = np.sqrt((x_target - self.x)**2 + (y_target - self.y)**2)
                # if d < 12 :
                #     self.__t_last_detect = time.time()
                #     self.get_logger().info(self.get_name() + " bot close to goal : " + str(d))

                # if self.__t_last_detect != None:
                #     if time.time() - self.__t_last_detect > 1.:
                #         self.__nb_balls += 1
                #         self.get_logger().info(self.get_name() + " got 1 ball !")
                #         self.get_logger().info(self.get_name() + " Nb of collected balls : " + str(self.__nb_balls))
                self.move(x_target, y_target)
                self.turn(angle)
        
        if self.__state == "come_back":
            x_filet = 640.
            self.__trigger.data = True
            if len(self.__bases) == 8:
                if self.__bases[0] < x_filet:
                    x_base_left = self.__bases[0]
                    y_base_left = self.__bases[1] 
                    x_base_right =self.__bases[6]
                    y_base_right = self.__bases[7]
                    w_left, h_left = self.__bases[2], self.__bases[3]
                    w_right, h_right = self.__bases[4], self.__bases[5]
                else:
                    x_base_right = self.__bases[2]
                    y_base_right = self.__bases[3]
                    x_base_left = self.__bases[4]
                    y_base_left = self.__bases[5]
                    w_right, h_right = self.__bases[0], self.__bases[1]
                    w_left, h_left = self.__bases[6], self.__bases[7]

                if self.x < x_filet:
                    # left
                    # self.get_logger().info("Going Left : " + str(x_base_left < self.x <  w_left) + ", " + str(y_base_left < self.y <  h_left))
                    mid_x, mid_y = (x_base_left + w_left)/2, (y_base_left + h_left)//2
                    angle = math.atan2(self.y - mid_y, self.x - mid_x)
                    self.move(mid_x, mid_y)
                    self.turn(angle)
                    if x_base_left < self.x <  w_left and y_base_left < self.y <  h_left:
                        self.__home = True

                else:
                    # right
                    # self.get_logger().info("Going Right")
                    mid_x, mid_y = (x_base_right + w_right)/2, (y_base_right + h_right)//2
                    angle = math.atan2(self.y - mid_y, self.x - mid_x)
                    self.move(mid_x, mid_y)
                    self.turn(angle)
                    if x_base_right < self.x < x_base_right and y_base_right < self.y < y_base_right:
                        self.__home = True


        if self.__state == "home":
            self.__trigger.data = False
            self.__cmd_twist.linear.x = -3.0
            if len(self.__bases) == 8:
                for i in range(0, len(self.__bases), 4):
                    x, y, w, h = self.__bases[i], self.__bases[i+1],self.__bases[i+2], self.__bases[i+3]
                    if x < self.x <  w and y< self.y < h:
                        self.__done = False
                    else :
                        self.__done = True

            
        self.twist_pub.publish(self.__cmd_twist)
        # self.get_logger().info("Twist published : " + str(self.__cmd_twist.linear.x) + ", " + str(self.__cmd_twist.linear.y) + ", " + str(self.__cmd_twist.angular.z))
        # self.get_logger().info(self.get_name() + " Running")
        # self.get_logger().info(self.get_name() + " : " + str(len(self.__bases)))






def main(args=None):    
    rclpy.init()
    my_py_node = Main()
    rclpy.spin(my_py_node)
    myPyNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
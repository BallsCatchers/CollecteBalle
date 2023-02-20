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


        self.__home = False

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
        self.__balls = msg.data


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
        v_x = float(max(min(self.K1 * d, 1.2), 0.6))
        self.__cmd_twist.linear.x = v_x
        if v_x > 0.8:
            self.__trigger.data = True
        else:
            self.__trigger.data = False
        # self.__cmd_twist.linear.y = self.K1 * d


    def turn(self, dtheta):
        err_theta = sawtooth(self.theta - dtheta)
        self.__cmd_twist.angular.z = self.K2 * err_theta

    def process(self):

        chrono = time.time() - self.__chrono
        if chrono >= 60.*4 and self.__state == "get_balls":
            self.__state = "come_back"
            self.get_logger().info(self.get_name() + " switched state to : " + self.__state)

        if self.__state == "come_back" and self.__home :
            self.__state = "get_balls"
            self.get_logger().info(self.get_name() + " switched state to : " + self.__state)
            self.__chrono = time.time()

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
                if ((x_target - self.x)**2 + (y_target - self.y)**2)
                self.move(x_target, y_target)
                self.turn(angle)
        
        if self.__state == "come_back":
            x_filet = 500.
            self.__trigger.data = True
            # if self.x < x_filet:
            #     # left
            #     self.move(x_base_left, y_base_left)
            #     self.turn(math.atan2(y_base_left, x_base_left))

            # else:
            #     # right
            #     self.move(x_base_right, y_base_right)
            #     self.turn(math.atan2(y_base_right, x_base_right))
        
        self.twist_pub.publish(self.__cmd_twist)
        # self.get_logger().info("Twist published : " + str(self.__cmd_twist.linear.x) + ", " + str(self.__cmd_twist.linear.y) + ", " + str(self.__cmd_twist.angular.z))
        # self.get_logger().info(self.get_name() + " Running")
        # self.get_logger().info(self.get_name() + " : " + str(len(self.__balls)))






def main(args=None):    
    rclpy.init()
    my_py_node = Main()
    rclpy.spin(my_py_node)
    myPyNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
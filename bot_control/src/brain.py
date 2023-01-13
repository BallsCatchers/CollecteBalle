#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node

# from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String

import math

def sawtooth(theta):
    return 2 * math.atan(math.tan(theta / 2))

class Robot:
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.theta = 0.

        self.K1 = 0.02
        self.K2 = 0.02

    def move(self, dx, dy):
        err_x = (dx - self.x)
        err_y = (dy - self.y)
        self.x += self.K1 * err_x
        self.y += self.K1 * err_y


    def turn(self, dtheta):
        err_theta = sawtooth(dtheta - self.theta)
        self.theta += self.K2 * err_theta


class Main(Node):
    def __init__(self):
        super().__init__("MainBallCatcher")

        # ===============================
        # Subscriptions 
        # ===============================
        self.input_balls = self.create_subscription(Float64MultiArray, "/tennis_balls", self.sub_balls, 10)
        self.__balls = []

        # ===============================
        # Publishers
        # ===============================
        self.trigger_pub = self.create_publisher(Bool, "/catch_trigger", 10)
        self.__trigger = Bool()
        self.__trigger.data = False


        self.twist_pub = self.create_publisher(Twist, "cmd_twist", 10)
        self.__cmd_twist = Twist()


        self.state_pub = self.create_publisher(String, "state", 10)
        # ===============================
        # Main
        # ===============================
        self.__state = "get_balls"
        self.__chrono = time.time()


        self.__home = False



        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")


    def sub_balls(self, msg):
        self.get_logger().info(self.get_name() + " got tennis balls")
        self.__balls = msg.data




    def process(self):

        chrono = time.time() - self.__chrono
        if chrono >= 30 and self.__state == "get_balls":
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

        self.__cmd_twist.linear.x = 0.
        self.__cmd_twist.linear.y = 0.
        self.__cmd_twist.linear.z = 0.

        self.__cmd_twist.angular.x = 0.
        self.__cmd_twist.angular.y = 0.
        self.__cmd_twist.angular.z = 0.

        self.twist_pub.publish(self.__cmd_twist)

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
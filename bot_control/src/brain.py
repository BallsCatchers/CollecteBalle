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

class Main(Node):
    def __init__(self):
        super().__init__("MainBallCatcher")

        # ===============================
        # Subscriptions 
        # ===============================
        self.input_balls = self.create_subscription(Float64MultiArray, "/tennis_balls", self.sub_balls, 10)
        self.__balls = []

        self.__bases = self.create_subscription(Float64MultiArray, "/bases", self.sub_bases, 10)
        self.__bases = []

        self.__pose = self.create_subscription(Vector3, "/pose_bot", self.get_pose, 10)

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

        self.K1 = 0.02
        self.K2 = 0.02

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")

    def sub_balls(self, msg):
        self.get_logger().info(self.get_name() + " got tennis balls")
        self.__balls = msg.data

    def sub_bases(self, msg):
        self.get_logger().info(self.get_name() + " got bases")
        self.__balls = msg.data
    
    def get_pose(self, msg):
        self.get_logger().info(self.get_name() + " got pose")
        self.x = msg.data.x
        self.y = msg.data.y
        self.theta = msg.data.theta
    
    def move(self, dx, dy):
        err_x = (dx - self.x)
        err_y = (dy - self.y)
        d = err_x ** 2 + err_y ** 2
        self.__cmd_twist.linear.x = self.K1 * d
        self.__cmd_twist.linear.y = self.K1 * d


    def turn(self, dtheta):
        err_theta = sawtooth(dtheta - self.theta)
        self.__cmd_twist.angular.z = self.K2 * err_theta

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

        if self.__state == "get_balls":
            distance_min = 1e10
            n_b = 0
            closest_b = 0
            for xb, yb in self.__balls:
                distance_b = (x - xb) ** 2 + (y - yb) ** 2
                if distance_b < distance_min:
                    distance_min = distance_b
                    closest_b = n_b
                n_b += 1
            x_target, y_target = self.__balls[closest_b], self.__balls[closest_b + 1]
        
            self.move(x_target, y_target)
            self.turn(math.atan2(y_target, x_target))
        
        if self.__state == "come_back":
            if self.x < x_filet:
                # left
                self.move(x_base_left, y_base_left)
                self.turn(math.atan2(y_base_left, x_base_left))

            else:
                # right
                self.move(x_base_right, y_base_right)
                self.turn(math.atan2(y_base_right, x_base_right))
        
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
#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Main(Node):
    def __init__(self):
        super().__init__("MainBallCatcher")

        self.twist_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.__cmd_twist = Twist()

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")

    def process(self):
        self.twist_pub.publish(self.__cmd_twist)


def main(args=None):    
    rclpy.init()
    my_py_node = Main()
    rclpy.spin(my_py_node)
    myPyNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
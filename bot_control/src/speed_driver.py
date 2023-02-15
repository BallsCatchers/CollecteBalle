#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node


from geometry_msgs.msg import Twist




class Main(Node):
    def __init__(self):
        super().__init__("SpeedDriver")

        # ===============================
        # Subscriptions 
        # ===============================
        self.input_twist = self.create_subscription(Twist, "/cmd_twist", self.sub_twist, 10)
        self.__cmd = Twist()



        # ===============================
        # Publishers
        # ===============================
        self.front_pub = self.create_publisher(Twist, "/gzbo/frontDrive/cmd_vel", 10)
        self.__front = Twist()


        # self.back_pub = self.create_publisher(Twist, "/gzbo/backDrive/cmd_vel", 10)
        # self.__back = Twist()

        # ===============================
        # Main
        # ===============================

        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")



    def sub_twist(self, msg):
        # self.get_logger().info(self.get_name() + " got tennis balls")
        self.__cmd = msg


    def process(self):
    	self.__front.linear.x = self.__cmd.linear.x
    	self.__front.linear.y = self.__cmd.linear.y
    	self.__front.linear.z = self.__cmd.linear.z



    	self.__back.linear.x = self.__cmd.linear.x
    	self.__back.linear.y = self.__cmd.linear.y
    	self.__back.linear.z = self.__cmd.linear.z


    	




#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool



class Main(Node):
    def __init__(self):
        super().__init__("MainBallCatcher")
        # self.result_pub = self.create_publisher(LaserScan, "scan", 10)
        # self.msg_result = Float64MultiArray()

        self.input_balls = self.create_subscription(Float64MultiArray, "/tennis_balls", self.sub_balls, 10)
        self.__balls = []


        self.trigger_pub = self.create_publisher(Bool, "/catch_trigger", 10)
        self.__trigger = Bool()
        self.__trigger.data = False

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")


    def sub_balls(self, msg):
        self.get_logger().info(self.get_name() + " got lidar info")
        self._dists = msg.intensities()
        self._angl = msg.ranges()



    def process(self):
        self.trigger_pub.publish(self.__trigger)
        self.get_logger().info(self.get_name() + " Running")
        self.get_logger().info(self.get_name() + " : " + str(len(self.__balls)))






def main(args=None):    
    rclpy.init()
    my_py_node = Main()
    rclpy.spin(my_py_node)
    myPyNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
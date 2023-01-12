#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


class Camera(Node):

    def __init__(self):
        super().__init__('camera')
        self.subscription_image = self.create_subscription(Image, "/zenith_camera/image_raw", self.image_callback, 10)

        #Image
        self.image = []
        self.get_logger().info(self.get_name() + " is launched")

    def image_callback(self, msg):
        try:
            self.image = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Image window", self.image)
        except CvBridgeError:
            self.get_logger().info(self.get_name() + ": Cannot convert message")

def main(args=None):
    rclpy.init(args=args)

    camera = Camera()
    print("Starting the program..")

    while rclpy.ok():
        #Reading nodes
        rclpy.spin_once(camera)


        # #Algorithm
        # data = []
        # for elem in camera.data:
        #     data.append(elem)
        #
        # data_array = np.zeros((camera.step, len(data)/camera.step))
        # for i in range(data_array.shape[0]):
        #     for j in range(data_array.shape[1]):
        #         data_array[i,j] = data[i+j]
        # data = np.array(data)
        # print(data.shape)
        # cv2.imread(data, np.uint8)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera.destroy_node()
    rclpy.shutdown()
    print("~~ End of the program ~~")


if __name__ == '__main__':
    main()

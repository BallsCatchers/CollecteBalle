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
import pygame
import os

def sawtooth(theta):
    return 2 * math.atan(math.tan(theta / 2))


class Main(Node):
    def __init__(self):
        super().__init__("MainBallCatcher")

        # ===============================
        # Subscriptions
        # ===============================
        self.input_gotball = self.create_subscription(String, "/gotball", self.sub_gotball, 10)
        self.get_ball = False

        # Initialize Pygame mixer
        pygame.mixer.init()

        # Load and play mario.mp3 on loop
        mario_mp3_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../../bot_control/src/", "mario.mp3")
        pygame.mixer.music.load(mario_mp3_path)
        pygame.mixer.music.play(-1)

    def sub_gotball(self, msg):
        # Getting the goal information
        # self.get_logger().info(self.get_name() + " Yoshi !" + msg.data)
        if msg.data == "GOTBALL":
            if not self.get_ball:
                self.get_ball = True
                self.process_music("aoe.mp3", volume=1.0)
        else:
            if self.get_ball:
                self.get_ball = False
                # Resume playing mario.mp3 on loop
                pygame.mixer.music.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../../bot_control/src/", "mario.mp3"))
                pygame.mixer.music.play(-1)

    def process_music(self, music_file, volume):
        # Construct the path to the music file
        mp3_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../../bot_control/src/", music_file)

        # Set the volume
        pygame.mixer.music.set_volume(volume)

        # Load and play the music file
        pygame.mixer.music.load(mp3_path)
        pygame.mixer.music.play()

        # Wait for the duration of the music and then resume playing mario.mp3 on loop
        pygame.time.wait(int(pygame.mixer.Sound(mp3_path).get_length() * 1000))
        pygame.mixer.music.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../../bot_control/src/", "mario.mp3"))
        pygame.mixer.music.play(-1)


def main(args=None):
    print(folder_path,"\n","\n")

    rclpy.init()
    my_py_node = Main()
    rclpy.spin(my_py_node)
    my_py_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    folder_path = os.path.dirname(os.path.abspath(__file__))
    main()

# if __name__ == '__main__':
#     folder_path = os.path.dirname(os.path.abspath(__file__))
#     mp3_path_noise = os.path.join(folder_path, "./Yoshis_Mlem_Sound_Effect.mp3")
#     if True:
#         pygame.init()
#         pygame.mixer.music.load(mp3_path_noise)
#         pygame.mixer.music.play()
#         while pygame.mixer.music.get_busy():
#             pass

import rclpy
from geometry_msgs.msg import Twist

def publisher_node():
    rclpy.init()
    node = rclpy.create_node('twist_publisher')
    pub = node.create_publisher(Twist, 'cmd_vel')

    msg = Twist()
    msg.linear.x = 1.0
    msg.angular.z = 0.5

    rate = rclpy.Rate(10) # 10Hz
    while rclpy.ok():
        pub.publish(msg)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    publisher_node()
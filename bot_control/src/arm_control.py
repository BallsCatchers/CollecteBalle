#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool



class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("publisher_joint_trajectory_position_controller")

        # Read parameters
        controller_name = "joint_trajectory_position_controller"
        timer = 1.
        self.joints = ["arm_l_joint", "arm_r_joint"]

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')




        # Read all positions from parameters
        self.goals = [[0., 0.], [0., 0.]]

        # print("Check of goals : ", self.goals)
        publish_topic = "/" + "joint_trajectory_controller" + "/" + "joint_trajectory"

        self.get_logger().info(
            'Publishing {} goals on topic "{}" every {} s'.format(
                len(self.joints), publish_topic, timer
            )
        )



        self.trigger_sub = self.create_subscription(Bool, "/catch_trigger", self.bool_callback,10)
        self.__trig = Bool()
        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 10)
        self.timer = self.create_timer(timer, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goals#[self.i]
        point.time_from_start = Duration(sec=1)

        traj.points.append(point)
        # print("Traj : ", traj)
        self.publisher_.publish(traj)

        # self.i += 1
        # self.i %= len(self.goals)


    def bool_callback(self, msg):

    	# self.get_logger().info('Received trigger ! ' + str(msg.data))
    	self.__trig.data = msg.data
    	if self.__trig.data:
    		self.goals = [-1.7, 1.7]
    	else:
    		self.goals = [0.2, -0.2]
    	# self.get_logger().info('Send goals : ' + str(self.goals[0]) + ', ' + str(self.goals[1]))

def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
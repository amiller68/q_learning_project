#!/usr/bin/env python3

import rospy
import numpy as np
import os

from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag


'''
This class handles acting on actions to published to the action topic
'''

class RobotManipulation(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("robot_manipulation")

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.accept_action)

        # ROS publishers
        self.reward_pub = rospy.Publisher("/q_learning/reward", QLearningReward, queue_size=10)
        self.run()

    def accept_action(self):
        pass

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = RobotManipulation()
#!/usr/bin/env python3

import rospy
import numpy as np
import os

'''
This class handles publishing action to subscribers based on Q learning data
'''

class RobotAction(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("robot_action")

        self.run()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = RobotAction()

#!/usr/bin/env python3
import csv
import sys

import rospy
import numpy as np
import os

from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag

'''
This class handles publishing action to subscribers based on Q learning data
'''

path_prefix = os.path.dirname(__file__) + "/action_states/"
q_matrix_path = os.path.dirname(__file__) + "/q_matrix.csv"

class RobotAction(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("robot_action")

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.accept_reward)

        # ROS publishers
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

        # Extract our actions, as done in q_learning.py
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        self.q_matrix = None
        self.state = 0
        self.reward = 0
        self.last_action = None

        with open(q_matrix_path, 'r') as q_csv:
            self.q_matrix = list(csv.reader(q_csv))

        self.running = True
        self.run()

    # Our manipulator should return a reward, even an empty one to signal that its done moving
    def accept_reward(self, reward):
        print("[R-ACTION] accepting a new reward")
        if self.reward_maximized():
            # Cause the node to exit if we're done moving objects
            self.running = False
            return
        next_action_id = self.get_next_action()
        self.send_action(next_action_id)

    def reward_maximized(self):
        # TODO: Check if we've reached an optimal state
        return True

    def send_action(self, action_id):
        # Get our action by index
        action = self.actions[action_id]
        self.last_action_id = action_id

        # Initialize a new message
        ret = RobotMoveObjectToTag()
        ret.robot_object = action['object']
        ret.tag_id = action['tag']

        # Publish the next action
        self.action_pub.publish(ret)

    # the id of the next action to take based on the state matrix
    def get_next_action(self):
        # TODO: Determine the next action
        ind = 0
        return ind

    def run(self):
        # Send a first action to init training
        try:
            # print("[R-ACTION] Sending first action...")
            self.send_action(self.get_next_action())
            # print("[R-ACTION] Listening for rewards...")
            while self.running:
                rospy.spin()
            return
        except KeyboardInterrupt:
            print("[R-ACTION] Q learner exiting...")
            sys.exit()


if __name__ == "__main__":
    node = RobotAction()

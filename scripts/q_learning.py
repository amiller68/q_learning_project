#!/usr/bin/env python3
import csv
import sys

import rospy
import numpy as np
import os
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag

from std_msgs.msg import Header

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"
q_matrix_path = os.path.dirname(__file__) + "/q_matrix.csv"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.accept_reward)

        # ROS publishers
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # Initialize our Q Matrix
        self.q_matrix = [
            [0] * len(self.actions)
            for _ in self.states
        ]

        # Run our learner
        self.running = True
        self.run()

    # Update Q matrix and then calculate the next action
    def accept_reward(self, reward):
        self.save_q_matrix()

    # return a Robot move to object msg based on action index
    def get_action(self, ind):
        # Get our action by index
        action = self.actions[ind]
        # Initialize a new message
        ret = RobotMoveObjectToTag()
        ret.robot_object = action['object']
        ret.tag_id = action['tag']
        return ret

    def save_q_matrix(self):
        with open(q_matrix_path, "w+") as q_csv:
            writer = csv.writer(q_csv, delimiter=',')
            writer.writerows(self.q_matrix)
        print("Q Matrix saved. exiting...")
        self.running = False

    def run(self):
        while self.running:
            rospy.spin()
        print("Q learner exiting...")
        return


if __name__ == "__main__":
    node = QLearning()
    sys.exit()

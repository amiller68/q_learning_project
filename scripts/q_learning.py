#!/usr/bin/env python3
import csv
import math
import sys
import threading
import time

import rospy
import numpy as np
import os
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag
from q_learning_project.msg import QMatrix

from std_msgs.msg import Header

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"
q_matrix_path = os.path.dirname(__file__) + "/q_matrix.csv"

# TODO: I know the communication works, byt the sending of the first action only gets heard inconsistently

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")
        self.rateLimit = rospy.Rate(1)  # How often we publish messages (2 Hz),
        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.accept_reward)

        # ROS publishers
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

        self.matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)

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

        self.q_matrix = None

        self.init_q_matrix()

        # Variables to hold our current state
        self.state_id = 0
        self.last_action_id = None

        # Run our learner
        self.exit = threading.Event()

    def init_q_matrix(self):
        self.q_matrix = [
            [0] * len(self.actions)
        ] * len(self.states)

        # Set any impossible actions in a given state to have a reward of -1

    # Update Q matrix and then calculate the next action
    def accept_reward(self, reward):
        if not self.exit:
            print("[QLEARNER ERROR] Received a reward before started run!")
            return

        print("[QLEARNER] accepting a new reward")
        self.update_q_matrix(reward)
        if self.matrix_converged():
            self.save_q_matrix()
            return
        next_action_id = self.get_next_action()
        self.send_action(next_action_id)

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

    def update_q_matrix(self, reward):
        pass

    def matrix_converged(self):
        return True

    # the id of the next action to take based on the state matrix
    def get_next_action(self):
        # TODO: Determine the next action
        ind = 0
        return ind

    def save_q_matrix(self):
        with open(q_matrix_path, "w+") as q_csv:
            writer = csv.writer(q_csv, delimiter=',')
            writer.writerows(self.q_matrix)
        print("Q Matrix saved.")
        self.exit.set()
        ret = QMatrix()
        ret.header = '' # TODO Figure out how to set this header
        ret.q_matrix = self.q_matrix
        self.matrix_pub.publish(ret)

    def start_q_learning(self):
        time.sleep(1)
        print("[QLEARNER] Sending first action...")
        self.send_action(self.get_next_action())

        # Run our learner
        self.run()

    def run(self):
        # Send a first action to init training
        try:
            print("[QLEARNER] Listening for rewards...")
            while not self.exit.is_set():
                time.sleep(1)
                print("hmm")
            print("[QLEARNER] Q learner exiting...")
            return
        except KeyboardInterrupt:
            print("[QLEARNER] Q learner exiting...")
            sys.exit()
        finally:
            rospy.signal_shutdown("Done processing Q-Matrix")


if __name__ == "__main__":
    node = QLearning()
    node.start_q_learning()

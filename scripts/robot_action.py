#!/usr/bin/env python3
import csv
import sys
import threading
import time

import rospy
import numpy as np
import os

from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag
from q_learning_project.msg import QMatrix


'''
This class handles publishing action to subscribers based on Q learning data
'''

path_prefix = os.path.dirname(__file__) + "/action_states/"
q_matrix_path = os.path.dirname(__file__) + "/q_matrix.csv"

class RobotAction(object):
    def __init__(self):
        # Initialize this node
        self.last_reward_msg = None
        rospy.init_node("robot_action")
        self.rateLimit = rospy.Rate(1)

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.accept_reward)
        # TODO: Implement getting q_matrix from a topic
        # rospy.Subscriber("/q_learning/q_matrix", QMatrix, self.get_matrix)
        # ROS publishers
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

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
        self.state_id = 0
        self.last_action = None

        with open(q_matrix_path, 'r') as q_csv:
            self.q_matrix = list(csv.reader(q_csv))

        self.steps = 0

        self.exit = threading.Event()
        self.processing = threading.Event()

    def get_matrix(self, data):
        self.q_matrix = data.q_matrix

    # Our manipulator should return a reward, even an empty one to signal that its done moving
    def accept_reward(self, reward):
        if not self.exit:
            print("[R-ACTION ERROR] Received a reward before started run!")
            return
        if self.processing.is_set():
            print("[QLEARNER ERROR] Received duplicate reward!")
            return
        self.processing.set()
        self.last_reward_msg = reward

        print("[R-ACTION] accepting a new reward")
        if self.reward_maximized():
            # Cause the node to exit if we're done moving objects
            self.exit.set()
            return
        next_action_id = self.get_next_action()
        print("[R-ACTION] Sending next action: ", next_action_id)
        self.send_action(next_action_id)
        self.processing.clear()

    # Only take three steps
    def reward_maximized(self):
        self.state_id = self.action_matrix[self.state_id].tolist().index(self.last_action_id)
        print("[R-ACTION] Next state: ", self.state_id)
        self.steps += 1
        return self.steps % 3 == 0

    def send_action(self, action_id):
        # Get our action by index
        print("Sending ACTION ID: ", action_id)

        action = self.actions[action_id]
        self.last_action_id = action_id

        # Initialize a new message
        ret = RobotMoveObjectToTag()
        ret.robot_object = action['object']
        ret.tag_id = action['tag']

        # Publish the next action
        self.action_pub.publish(ret)

    # the id of the best action to take based on the state
    def get_next_action(self):
        # Pick a random valid action based on our current state
        next_action_id = self.q_matrix[self.state_id].index(max(self.q_matrix[self.state_id]))
        return next_action_id

    # Once we have everything initialized, start sending actions
    def start_action_sequence(self):
        time.sleep(1)
        print("[R-ACTION] Sending first action from state 0")
        self.send_action(self.get_next_action())

        # Run our learner
        self.run()

    def run(self):
        # Send a first action to init training
        try:
            print("[R-ACTION] Listening for rewards...")
            while not self.exit.is_set():
                time.sleep(1)
            return
        except KeyboardInterrupt:
            print("[R-ACTION] exiting...")
            self.exit.set()
            sys.exit()
        # finally:
        #     rospy.signal_shutdown("Done processing Q-Matrix")


if __name__ == "__main__":
    node = RobotAction()
    node.start_action_sequence()

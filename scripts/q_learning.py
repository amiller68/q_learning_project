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

from random import randint

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
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt").astype(int)

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt").astype(int)
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
        self.states = np.loadtxt(path_prefix + "states.txt").astype(int)
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        self.q_matrix = None

        self.init_q_matrix()

        # Variables to hold our current state
        self.state_id = 0
        self.last_reward_msg = None
        self.last_action_id = None

        # Our convergence params
        self.alpha = 1
        self.gamma = 0.3

        # Keep track of what step where at to know when to reset the world
        self.steps = 0
        # The lower this is the more converged the matrix will be
        self.convergence_bound = 0.05
        # Keep track of how many consecutive convergences we've observed
        self.seq_convergences = 0
        # How many seq convergences we need to observe to determine we're done
        self.seq_convergences_target = 800

        # Run our learner
        self.exit = threading.Event()

    def init_q_matrix(self):
        # Kenneth says make everything start at 0
        # Strategy: start with all impossible transitions
        # self.q_matrix = [
        #     [-1] * len(self.actions)
        #     for _ in self.states
        # ]
        self.q_matrix = [
            [0] * len(self.actions)
            for _ in self.states
        ]
        # # Set all valid actions to 0
        # for s in range(len(self.states)):
        #     # For a given state, extract all valid actions (> -1)
        #     available_actions = [a for a in self.action_matrix[s] if a != -1]
        #     for a in available_actions:
        #         self.q_matrix[s][a] = 0

    # Update Q matrix and then calculate the next action
    def accept_reward(self, reward):
        print("[QLEARNER] accepting a new reward: ", reward.reward)

        # Error check the reward
        if self.exit.is_set():
            print("[QLEARNER ERROR] Received a reward before started run!")
            return
        elif self.last_reward_msg == reward:
            print("[QLEARNER ERROR] Received duplicate reward!")
            return
        self.last_reward_msg = reward

        # Returns true if the matrix is converged
        if self.update_q_matrix(reward.reward):
            self.save_q_matrix()
            return
        next_action_id = self.get_next_action()
        print("[QLEARNER] Sending next action: ", next_action_id)
        self.send_action(next_action_id)

    def send_action(self, action_id):
        print("Sending ACTION ID: ", action_id)
        # Update our last action taken
        self.last_action_id = action_id
        # Get our action by index
        action = self.actions[action_id]

        # Initialize a new message
        ret = RobotMoveObjectToTag()
        ret.robot_object = action['object']
        ret.tag_id = action['tag']

        # Publish the next action
        self.action_pub.publish(ret)

    # Update our Q matrix and return True if its converged
    def update_q_matrix(self, reward):
        self.steps += 1

        # Determine the next state
        next_state_id = self.action_matrix[self.state_id].tolist().index(self.last_action_id)

        # Get the reward for our last action
        last_reward = self.q_matrix[self.state_id][self.last_action_id]
        # Get the maximum reward for the next state
        print(self.q_matrix[next_state_id])
        max_reward = max(self.q_matrix[next_state_id])
        print("MAX REWARD: ", max_reward)

        # Update our Q-Matrix based on the Q-Learning algorithm
        self.q_matrix[self.state_id][self.last_action_id] += \
            self.alpha * (reward + self.gamma * max_reward - last_reward)

        print("[QLEARNER] Last reward: ", last_reward)
        print("[QLEARNER] Updated reward: ", self.q_matrix[self.state_id][self.last_action_id])

        # But if we've moved all the blocks, reset the world.
        # Don't try and update because we'd be moving into an invalid state
        if self.steps % 3 == 0:
            # if [_ for _ in self.q_matrix[next_state_id] if _ != -1] == []:
            #     print("Entering an invalid state!")
            print("[QLEARNER] Resetting state to 0")
            next_state_id = 0
            # return False

        # Transition to the next state
        self.state_id = next_state_id

        print("[QLEARNER] next state: ", self.state_id)

        # Check if this is within the bounds of convergence
        if abs(last_reward - self.q_matrix[self.state_id][self.last_action_id]) <= self.convergence_bound:
            self.seq_convergences += 1
        # else:
        #     self.seq_convergences = 0

        print("[QLEARNER] Sequential convergences: ", self.seq_convergences)

        # If we haven't converged yet, return false
        return self.seq_convergences >= self.seq_convergences_target

    # the id of the next action to take based on the state matrix
    def get_next_action(self):
        # Pick a random valid action based on our current state
        available_actions = [a for a in self.action_matrix[self.state_id] if a != -1]
        next_action_id = available_actions[randint(0, len(available_actions) - 1)]
        return next_action_id

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
            while not self.exit.is_set() and not rospy.is_shutdown():
                time.sleep(1)
            print("[QLEARNER] Q learner exiting...")
            return
        except KeyboardInterrupt:
            print("[QLEARNER] Q learner exiting...")
            sys.exit()
        finally:
            rospy.loginfo("[QLEARNER] Exiting")
            rospy.signal_shutdown("Done processing Q-Matrix")


if __name__ == "__main__":
    node = QLearning()
    node.start_q_learning()

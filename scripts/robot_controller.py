#!/usr/bin/env python3
import math
import sys
import time

import rospy
import numpy as np
import os

import rospy, rospkg, cv2, cv_bridge, numpy
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
import moveit_commander
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag


# Return the CV ranges for an HSV color
def cv_hsv(h, s, v):
    return np.array([
        h / 360 * 180 - 1,
        s * 256 - 1,
        v * 256 - 1
    ])


'''
General class for perceiving objects around a robot
Works on objects identified by color or AR tags
'''


class RobotPerception(object):
    def __init__(self):
        self.class_initialized = False
        # HOW OUR LINE FOLLOWER WAS INITIALIZED
        self.bridge = cv_bridge.CvBridge()
        # # initalize the debugging window
        # cv2.namedWindow("window", 1)

        rp = rospkg.RosPack()
        # # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        # A suscriber to get LaserScan Readings
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.objects = {
            'blue': {
                # HSV: 210, 1, 1
                'lower': cv_hsv(175, .5, .5),
                # HSV: 210, 1/2, 3/4
                'upper': cv_hsv(225, 1, 1)
            },
            'pink': {
                # HSV: 330, 1, 1
                # 'lower': cv_hsv(300, .85, .75),
                'lower': cv_hsv(280, .6, .5),

                # 'lower': cv_hsv(0, 0, 0),

                # HSV: 330, 1/2, 3/4
                'upper': cv_hsv(330, 1, 1)
            },
            'green': {
                # HSV: 120, 1/2, 1
                'lower': cv_hsv(80, .5, .6),
                # HSV: 120, 1, 3/4
                'upper': cv_hsv(130, 1, 1)
            }
        }

        self.tags = {
            1: '',
            2: '',
            3: ''
        }

        # State to keep track of what object/tag we're targetting and our latest img data
        self.target = None  # What target we're trying to locate at a given time
        self.target_err = None  # The orientation error of our bot from target
        self.target_dist = None  # How far the object directly in front of our robot is
        self.image = None  # The image the robot is currently processing
        self.scan_data = None  # The scan data the robot is currently processing

        # Use these to debug perception
        # self.test_color_perception()
        # self.test_tag_perception()

        self.class_initialized = True

    def image_callback(self, data):
        # Bind the img to our state variable
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        # If our target specifies an object
        if self.target in self.objects:
            # Look for the object, update err accordingly
            self.locate_object()
        elif self.target in self.tags:
            self.locate_tag()
        cv2.imshow("window", self.image)
        cv2.waitKey(3)

    def scan_callback(self, msg):
        self.scan_data = msg
        # Bind our target dist to the range of the object in front of us
        self.target_dist = msg.ranges[0]

    # check if the perceptor is initialized
    def initialized(self):
        return self.image is not None and self.scan_data is not None and self.class_initialized

    # Get the perceptors scan parameters. Used by the manipulator to determine max scan range
    def get_scan_data(self):
        if self.initialized():
            return self.scan_data
        return None

    # Set what target our perceptor is looking for at any given time
    def set_target(self, target):
        # If this is a good target
        if target in self.objects or target in self.tags:
            self.target_err = None
            self.target = target
            return True
        print("[R-PERCEPTION ERROR] Unknown target: ", target)
        return False

    # Get the perceptor's goal target based on its image and scan state
    def get_target(self):
        # If we have an idea of where to look for an object
        if self.target_err:
            return self.target_err, self.target_dist
        return None

    # A debugging script to test how well our node can perceive objects
    def test_color_perception(self):
        for color, ranges in self.objects.items():
            self.set_target(color)
            print("Looking for: ", color)
            while self.get_target():
                pass
            print("Found: ", color)
            time.sleep(3)
        print("Found all the colors!")

    # A debugging script to test how well our node can perceive tags
    def test_tag_perception(self):
        for tag, ar in self.tags.items():
            self.set_target(tag)
            print("Looking for: ", tag)
            while not self.get_target():
                pass
            print("Found: ", tag)
            time.sleep(3)
        print("Found all the tags!")

    # Locate an object and set orientation correction to target_err
    # If the object is found then set this value to None
    def locate_object(self):
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        lower_color = self.objects[self.target]['lower']
        upper_color = self.objects[self.target]['upper']

        # # this erases all pixels that aren't yellow
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # # this limits our search scope to only view a slice of the image near the ground
        h, w, d = self.image.shape

        # # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)
        # if there are any yellow pixels found
        ret = None
        if M['m00'] > 0:
            # center of the yellow pixels in the image
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # # a red circle is visualized in the debugging window to indicate
            # # the center point of the yellow pixels
            # # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
            cv2.circle(self.image, (cx, cy), 20, (0, 0, 255), -1)
            err = -float((cx - w / 2)) / w
            ret = err  # math.pow(err, 2) if err > 0 else -1 * math.pow(err, 2)
        # Set our reported target err
        self.target_err = ret

    # Locate an AR tag and set an orientation correction to target_err
    # If the location is unknown set this value to None
    def locate_tag(self):
        ret = None
        # This should look similar to the above, but we need to use the cv2 library
        self.target_err = ret


def joint_angs_to_radians(angs):
    return map(math.radians, angs)

class RobotManipulator(object):
    def __init__(self, scan_range_max=4.5):
        # Initialize a publisher to the velocity cmd topic
        self.move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Interfaces for moving the robot arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        self.arm_positions = {
            'start': joint_angs_to_radians([0, 17, 16, -33]),
            'lift': joint_angs_to_radians([0, -95, 75, -70]),
            'drop': joint_angs_to_radians([1, 16, 17, -32]),

        }

        self.grips = {
            'close': [0.0, 0.0],
            'open': [.01, .01]
        }

        # Set our velocity constants
        self.linear_speed = .05  # m/s
        self.angular_speed = -.75  # rad/s
        self.scan_range_max = scan_range_max

        # Set a goal distance, how close we should get to a target
        self.target_dist = .175  # m

        # while True:
        #     print("Testing arm...")
        #     self.return_arm_to_start()
        #     self.pickup_object()
        #     self.put_down_object()

        self.return_arm_to_start()

        self.arm_ready = True

    # Stop the robot where it is
    def stop(self):
        self.move.publish(Twist())

    # Backup a little bit
    def backup(self, duration):
        end_time = time.time() + duration
        movement = Twist()
        movement.linear.x = -self.linear_speed
        while time.time() < end_time:
            self.move.publish(movement)
        self.stop()

    def initialized(self):
        return self.arm_ready

    # Tell the robot to follow some object based on estimated orientation error and how far it is, stored in tuple
    # Publishes a movement command that follows the specified target
    def follow_target(self, target):

        # Extract from target
        follow = Twist()
        # If we don't have a target
        if not target:
            # print("No path! Looking around...")
            # Try Looking around for it
            follow.angular.z = self.angular_speed
            self.move.publish(follow)
            return True
        else:
            print("Found a path: ", target)

            # err is a float describing a rotation pid
            # dist describes the distance observed directly in front of the robot
            err, dist = target

            # Set our linear velocity based on the distance

            # If we can't see anything in from of us
            if math.isinf(dist) or dist == 0:
                # Gotta go fast
                follow.linear.x = self.linear_speed
            # Else if we're closed in on it
            elif dist > self.target_dist:
                follow.linear.x = \
                    self.linear_speed * (dist - self.target_dist) / (self.scan_range_max - self.target_dist)

            follow.linear.x = min(
                self.linear_speed,
                (dist - self.target_dist) / self.target_dist
            )
            follow.angular.z = err

            print("Converging on target: ", follow.linear.x, "m/s @", follow.angular.z, "rad/s")
            self.move.publish(follow)

            # Publish our follow command
            self.move.publish(follow)

            # Return false if we ever come in range of our target
            return dist > self.target_dist

    def return_arm_to_start(self):
        print("[MANIPULATOR] Returning arm to start...")
        self.move_group_arm.go(self.arm_positions['start'], wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(self.grips['open'], wait=True)
        self.move_group_gripper.stop()
        time.sleep(3)

    # Pick up an object
    def pickup_object(self):
        print("[MANIPULATOR] Picking object up...")
        self.move_group_gripper.go(self.grips['close'], wait=True)
        self.move_group_gripper.stop()
        self.move_group_arm.go(self.arm_positions['lift'], wait=True)
        self.move_group_arm.stop()
        time.sleep(3)

    # Put down an object
    def put_down_object(self):
        print("[MANIPULATOR] Putting object down...")
        self.move_group_arm.go(self.arm_positions['drop'], wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(self.grips['open'], wait=True)
        self.move_group_gripper.stop()
        time.sleep(3)


'''
This class handles acting on actions to published to the action topic
It uses our perception and 
'''


class RobotController(object):
    def __init__(self):
        self.initialized = False

        # Initialize this node
        rospy.init_node("robot_controller")

        # subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.accept_action)

        # publish to our learning reward topic
        self.reward_pub = rospy.Publisher("/q_learning/reward", QLearningReward, queue_size=10)

        self.rate = rospy.Rate(10)
        print("Waiting for perception to be initialized")

        # Initialize our perception
        self.perception = RobotPerception()
        while not self.perception.initialized():
            print("...")
            time.sleep(1)

        print("Waiting for manipulator to be initialized")

        # Initialize our manipulation class, make sure to provide it with a max scan value
        self.manipulator = RobotManipulator(scan_range_max=self.perception.get_scan_data().range_max)

        while not self.manipulator.initialized():
            print("...")
            time.sleep(1)

        # ROS:

        print("Controller initialized.")
        self.initialized = True

    def accept_action(self, action):
        print("Received action: ", action)

        print("Moving towards object: ", action.robot_object)

        # Set our perceptor to look for our object
        if not self.perception.set_target(action.robot_object):
            print("[R-Controller ERROR] Can't locate this object: ", action.robot_object)
            sys.exit()

        target = self.perception.get_target()
        while self.manipulator.follow_target(target):
            target = self.perception.get_target()

        # Stop the robot
        self.manipulator.stop()

        # Pickup the object
        self.manipulator.pickup_object()

        print("Moving object towards tag: ", action.tag_id)

        # Set our perceptor to look for our object
        if not self.perception.set_target(action.tag_id):
            print("[R-Controller ERROR] Can't locate this tag: ", action.tag_id)
            sys.exit()

        # target = self.perception.get_target()
        target = 0, 0  # FOR testing put an object down, make sure to remove this!
        while self.manipulator.follow_target(target):
            target = self.perception.get_target()

        # Stop the robot
        self.manipulator.stop()

        # Put down the object
        self.manipulator.put_down_object()

        # Back away from the object
        self.manipulator.backup(duration=5)

        # Return the arm to the start position
        self.manipulator.return_arm_to_start()

        # Publish an empty reward for now
        self.reward_pub.publish(QLearningReward())

    def start_controller(self):
        # Publish an empty reward to let the action handler know to start
        time.sleep(3)
        self.reward_pub.publish(QLearningReward())
        self.run()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = RobotController()
    node.start_controller()
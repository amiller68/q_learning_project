#!/usr/bin/env python3

import rospy
import numpy as np
import os


from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag


'''
General class for perceiving objects around a robot
Works on objects identified by color or AR tags
'''

class RobotPerception(object):
    def __init__(self):
        # HOW OUR LINE FOLLOWER WAS INITIALIZED
        # self.bridge = cv_bridge.CvBridge()
        # # initalize the debugging window
        # cv2.namedWindow("window", 1)
        # # subscribe to the robot's RGB camera data stream
        # self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        # # self.image_sub = rospy.Subscriber('raspicam_node/image',
        # #                                   Image, self.image_callback)
        #
        # # Rospy Params
        # self.queue_size = 20  # The size of our message queues
        # self.rateLimit = rospy.Rate(10)  # How often we publish messages (2 Hz), utilize with self.rateLimit.sleep()
        # # State to hold our img
        self.img_msg = None

        # Keep track of latest angular velocity (either L or R)
        # True if moving left
        self.lastDir = None

    def image_callback(self, msg):
        self.img_msg = msg

    # Locate an object and return an orientation correction and approximate distance
    # If the object is found then return None. Otherwise, return a tuple (err, dist).
    # If the location is unknown return (None, None)
    def locate_object(self, object_string):
        ret = None
        # CODE FROM OUR LINE FOLLOWER
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        # image = self.bridge.imgmsg_to_cv2(self.img_msg, desired_encoding='bgr8')
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #
        # lower_yellow = numpy.array([10, 50, 150])
        # upper_yellow = numpy.array([20, 255, 255])
        #
        # # this erases all pixels that aren't yellow
        # mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        #
        # # this limits our search scope to only view a slice of the image near the ground
        # h, w, d = image.shape
        # search_top = int(3 * h / 4)
        # search_bot = int(3 * h / 4 + 20)
        # mask[0:search_top, 0:w] = 0
        # mask[search_bot:h, 0:w] = 0
        #
        # # using moments() function, the center of the yellow pixels is determined
        # M = cv2.moments(mask)
        # # if there are any yellow pixels found
        # ret = None
        # if M['m00'] > 0:
        #     # center of the yellow pixels in the image
        #     cx = int(M['m10'] / M['m00'])
        #     cy = int(M['m01'] / M['m00'])
        #
        #     # # a red circle is visualized in the debugging window to indicate
        #     # # the center point of the yellow pixels
        #     # # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
        #     cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
        #     err = -float((cx - w / 2)) / w
        #     ret = err
        # # disable this to reduce lag
        # cv2.imshow("window", image)
        # cv2.waitKey(3)
        return ret

    # Locate an AR tag and return an orientation correction and approximate distance
    # If the object is found then return None. Otherwise, return a tuple (err, dist).
    # If the location is unknown return (None, None)
    def locate_tag(self, tag_string):
        ret = None
        # This should look similar to the above, but we need to use the cv2 library
        return ret


class RobotManipulator(object):
    def __init__(self):
        self.move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Maybe add odometry?

        self.linear_speed = .5  # m/s
        self.angular_speed = .3  # rad/s

        # Our goal distance, how close we should get to a target
        self.target_dist = .1  # m
        pass

    # Stop the robot where it is
    def stop(self):
        self.move.publish(Twist())

    # Tell the robot to follow some object based on estimated orientation error and how far it is, stored in tuple
    # Publishes a movement command that follows the specified target
    def follow_target(self, target):
        err, dist = target

        # If we've received a code for not knowing an objects location...
        if not err and not dist:
            # Try Looking around
            return

        print("Found a path: ", target)

        # TODO: test this

        # Extract from target
        follow = Twist()

        # Implement linear proportional control
        follow.linear.x = min(
            self.linear_speed,
            (dist - self.target_dist) / self.target_dist
        )
        follow.angular.z = err
        print("Converging on target: ", follow.linear.x, "m/s @", follow.angular.z, "rad/s")
        self.move.publish(follow)

    # Pick up an object
    def pickup_object(self):
        pass

    # Put down an object
    def put_down_object(self):
        pass


'''
This class handles acting on actions to published to the action topic
It uses our perception and 
'''

class RobotController(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("robot_controller")

        self.manipulator = RobotManipulator()
        self.perception = RobotPerception()

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.accept_action)

        # ROS publishers
        self.reward_pub = rospy.Publisher("/q_learning/reward", QLearningReward, queue_size=10)
        self.run()

    def accept_action(self, action):
        print("Received action: ", action)

        print("Moving towards object: ", action.robot_object)
        while target := self.perception.locate_object(action.robot_object):
            self.manipulator.follow_target(target)

        # Stop the robot
        self.manipulator.stop()

        # Pickup the object
        self.manipulator.pickup_object()

        print("Moving object towards tag: ", action.tag_id)
        while target := self.perception.locate_object(action.tag_id):
            self.manipulator.follow_target(target)

        # Stop the robot
        self.manipulator.stop()

        # Pickup the object
        self.manipulator.pickup_object()

        # Publish an empty reward for now
        self.reward_pub.publish(QLearningReward())

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = RobotController()
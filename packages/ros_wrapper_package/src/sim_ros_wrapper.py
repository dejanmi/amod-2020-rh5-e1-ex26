#!/usr/bin/env python3

import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import gym_duckietown
from gym_duckietown.simulator import Simulator
import cv2

class SimROSWrapper(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(SimROSWrapper, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('fakebot/camera_node/image/compressed', CompressedImage, queue_size=10)
        self.sub = rospy.Subscriber('fakebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.callback)
        self.env = Simulator(
            seed=123,  # random seed
            map_name="loop_empty",
            max_steps=500001,  # we don't want the gym to reset itself
            domain_rand=0,
            camera_width=640,
            camera_height=480,
            accept_start_angle_deg=4,  # start close to straight
            full_transparency=True,
            distortion=True,
        )
        self.action = np.array([0.0, 0.0])

    def callback(self, data):
        self.action = np.array([data.vel_left, data.vel_right])

    def step(self):
        while not rospy.is_shutdown():
            observation, reward, done, misc = node.env.step(self.action)
            image_np = cv2.cvtColor(np.ascontiguousarray(observation), cv2.COLOR_BGR2RGB)
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
            self.env.render()
            self.pub.publish(msg)


if __name__ == '__main__':
    # create the node
    node = SimROSWrapper(node_name='sim_ros_wrapper')
    node.step()
    # keep spinning
    rospy.spin()

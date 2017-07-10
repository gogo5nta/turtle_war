#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from gazebo_msgs.msg import ModelStates 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class AbstractBot(object):
    __metaclass__ = ABCMeta

    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name

        # bumper state
        self.bumper = BumperEvent()
        self.center_bumper = False
        self.left_bumper = False
        self.right_bumper = False

        # for convert image topic to opencv obj
        self.bridge = CvBridge()

        # velocity publisher
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,queue_size=1)

        # bumper subscrivre
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumperCallback)

        # camera subscriver
        # please uncoment out if you use camera
        # reffer http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',   Image, self.imageCallback)
        # reffer http://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depthCallback)        

    # bumper topic call back sample
    # update bumper state
    def bumperCallback(self, data):
        if data.bumper == 0:
            if data.state == 1:
                self.left_bumper = True
            else:
                self.left_bumper = False
                
        if data.bumper == 1:
            if data.state == 1:
                self.center_bumper = True
            else:
                self.center_bumper = False

        if data.bumper == 2:
            if data.state == 1:
                self.right_bumper = True
            else:
                self.right_bumper = False

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def depthCallback(self, data):
        try:
            #depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")            
        except CvBridgeError as e:
            print(e)

        depth_array = np.array(depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        cv2.imshow("Depth window", depth_array)
        cv2.waitKey(3)        

    @abstractmethod
    def strategy(self):
        pass


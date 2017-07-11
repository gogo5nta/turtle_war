#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from gazebo_msgs.msg import ModelStates 
from sensor_msgs.msg import Image
import message_filters

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
        # get rgb and depth image  http://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/
        self.image_sub = message_filters.Subscriber("camera/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("camera/depth/image_raw", Image)

        # ApproximateTimeSynchronizer http://docs.ros.org/api/message_filters/html/python/
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 1, 0.5)
        self.ts.registerCallback(self.imageDepthcallback)

        # view gui flag
        self.cv_view = True

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
    def imageDepthcallback(self, rgb_data, depth_data):
        try:
            rgb_image   = self.bridge.imgmsg_to_cv2(rgb_data,   "bgr8")

            # resize  http://tatabox.hatenablog.com/entry/2013/07/15/164015
            h = rgb_image.shape[0]
            w = rgb_image.shape[1]            
            rgb_image_half = cv2.resize(rgb_image,(w/2,h/2))

            # reffer http://answers.ros.org/question/58902/how-to-store-the-depth-data-from-kinectcameradepth_registreredimage_raw-as-gray-scale-image/
            #depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")            
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")

            # resize  http://tatabox.hatenablog.com/entry/2013/07/15/164015            
            h = depth_image.shape[0]
            w = depth_image.shape[1]
            depth_image_half = cv2.resize(depth_image,(w/2,h/2))            
        except CvBridgeError, e:
            print e

        #depth_array = np.array(depth_image, dtype=np.float32)
        depth_array = np.array(depth_image_half, dtype=np.float32)        
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        # --- test Red Region ---
        # https://www.blog.umentu.work/python3-opencv3%E3%81%A7%E6%8C%87%E5%AE%9A%E3%81%97%E3%81%9F%E8%89%B2%E3%81%AE%E3%81%BF%E3%82%92%E6%8A%BD%E5%87%BA%E3%81%97%E3%81%A6%E8%A1%A8%E7%A4%BA%E3%81%99%E3%82%8B%E3%80%90%E5%8B%95%E7%94%BB/
        # BGR
        lower_red = np.array([0,    0,    0])
        upper_red = np.array([8,    8,  255])       
        red_mask = cv2.inRange(rgb_image_half, lower_red, upper_red)
        red_image = cv2.bitwise_and(rgb_image_half, rgb_image_half, mask=red_mask) 

        if self.cv_view == True:
            cv2.imshow("Image window", rgb_image_half)            
            cv2.imshow("Depth window", depth_array)
            cv2.imshow("Red Image window", red_image)
            cv2.waitKey(1)

    @abstractmethod
    def strategy(self):
        pass


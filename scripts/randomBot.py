#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import random
import time

from abstractBot import *
from geometry_msgs.msg import Twist

class RandomBot(AbstractBot):
    
    def strategy(self):
        r = rospy.Rate(100)

        #target        
        target_speed = 0
        target_turn  = 0

        #control
        control_speed = 0
        control_turn = 0

        #max
        max_control_speed = 2

        #time
        UPDATE_FREQUENCY = 1
        update_time = 0

        #flag
        bum_flag = False
        yel_flag = False        
        red_flag = False
        rot_flag = False

        th = 0.0
        sp = 0.0

        while not rospy.is_shutdown():
            if self.center_bumper or self.left_bumper or self.right_bumper:
                update_time = time.time()
                rospy.loginfo('bumper hit!!')

                control_speed = -1
                target_speed  = -1

                control_turn = 3
                target_turn  = 3

                # init
                th_cnt = 0.0
                bum_flag = True                
                yel_flag = False
                red_flag = False
                rot_flag = False

            elif time.time() - update_time > UPDATE_FREQUENCY:
                update_time = time.time()
                
                # --- use rgb and depth data ---
                # step1: extract yellow (yellow)
                # 
                if self.yel_loc[0] != -1 and self.yel_loc[1] != -1:
                    self.step = 1

                    # init
                    th_cnt = 0.0
                    if bum_flag:
                        target_turn  = 0
                        control_turn = 0
                        #target_speed  = max_control_speed
                        #control_speed = max_control_speed
                        bum_flag = False                        
                    red_flag = False
                    yel_flag = True
                    if rot_flag:
                        target_turn  = 0
                        control_turn = 0
                        #target_speed  = max_control_speed
                        #control_speed = max_control_speed
                        rot_flag = False

                    loc_x = self.yel_loc[0]
                    cen_x = self.loc_centor_x
                    val   = self.red_val

                    th = np.arctan((1.0*(loc_x - cen_x)/cen_x))
                    target_turn  = -th

                    if   0.05 < val:
                        sp = 2
                    elif 0.04 < val:
                        sp = 1.8
                    elif 0.03 < val:
                        sp = 1.6
                    elif 0.02 < val:
                        sp = 1.4
                    elif 0.01 < val:
                        sp = 1.2
                    else:
                        sp = 0.8 
                    target_speed = sp
                    
                # step2: extract red (red)
                elif self.red_loc[0] != -1 and self.red_loc[1] != -1:
                    self.step = 2
                    
                    # init
                    th_cnt = 0.0
                    if bum_flag:
                        target_turn  = 0
                        control_turn = 0
                        #target_speed  = max_control_speed
                        #control_speed = max_control_speed
                        bum_flag = False                                            
                    yel_flag = False
                    red_flag = True
                    if rot_flag:
                        target_turn  = 0
                        control_turn = 0
                        #target_speed  = max_control_speed
                        #control_speed = max_control_speed
                        rot_flag = False

                    loc_x = self.red_loc[0]
                    cen_x = self.loc_centor_x
                    val   = self.red_val

                    th = np.arctan((1.0*(loc_x - cen_x)/cen_x))
                    target_turn  = -th

                    if   0.05 < val:
                        sp = 2
                    elif 0.04 < val:
                        sp = 1.8
                    elif 0.03 < val:
                        sp = 1.6
                    elif 0.02 < val:
                        sp = 1.4
                    elif 0.01 < val:
                        sp = 1.2
                    else:
                        sp = 0.8 
                    target_speed = sp


                # step3: check total cnt of rotation th (green)
                elif th_cnt < 10 and (yel_flag or red_flag):
                    self.step = 3

                    #init
                    rot_flag = True

                    control_speed = 0
                    target_speed  = 0

                    #control_turn = 1.5
                    target_turn  = 2.0
                    th_cnt = th_cnt +1

                # step0: random (while)
                else:
                    self.step = 0
                    
                    # init
                    th_cnt =   100
                    yel_flag = False
                    red_flag = False
                    rot_flag = False                    

                    # random  http://www.python-izm.com/contents/application/random.shtml
                    target_speed  = random.uniform(-1, 1)
                    target_turn   = random.uniform(-1.5, 1.5)                    

            #print http://programming-study.com/technology/python-print/
            print('speed(%f, %f)_turn(%f, %f)_th(%f)_sp(%f)' %(target_speed, control_speed, target_turn, control_turn, th, sp))

            # def 0.02
            del_speed  = 0.04
            # def 0.1
            del_turn   = 0.1

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + del_speed )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - del_speed )
            else:
                control_speed = target_speed

            #max speed check
            control_speed = min (control_speed, max_control_speed)

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + del_turn )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - del_turn)
            else:
                control_turn = target_turn  

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            #print(twist)
        
            self.vel_pub.publish(twist)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('random_bot')
    bot = RandomBot('Random')
    bot.strategy()

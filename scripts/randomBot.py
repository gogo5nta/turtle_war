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
        
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        surplus = 0

        UPDATE_FREQUENCY = 1
        update_time = 0

        #define
        x  = 1.0
        th = 0.0
        rot_flag = False

        while not rospy.is_shutdown():
            if self.center_bumper or self.left_bumper or self.right_bumper:
                update_time = time.time()
                rospy.loginfo('bumper hit!!')

                # go speed
                x = 0
                # turn speed
                th = 3
                control_speed = -1
                control_turn = 0

                # init
                th_cnt = 0.0
                x_cnt  = 0.0
            
            elif time.time() - update_time > UPDATE_FREQUENCY:
                update_time = time.time()
                
                # --- use rgb and depth data ---
                # step1: extract yellow (yellow)
                if self.yel_loc[0] != -1 and self.yel_loc[1] != -1:
                    # init
                    th_cnt = 0.0
                    x_cnt  = 0.0
                    rot_flag = False
                    self.step = 1

                    loc_x = self.yel_loc[0]
                    cen_x = self.loc_centor_x
                    val   = self.red_val                    

                    th = (1.0*(cen_x - loc_x)) / cen_x
                    x  = 0.5 + (1.0 - val)*(1.0 - val)
                    # print('yel_th%f_x%f_val%f',th, x, val)

                # step2: extract red (red)
                elif self.red_loc[0] != -1 and self.red_loc[1] != -1:
                    # init
                    th_cnt = 0.0
                    x_cnt  = 0.0
                    rot_flag = False
                    self.step = 2

                    loc_x = self.red_loc[0]
                    cen_x = self.loc_centor_x
                    val   = self.red_val

                    th = (1.0*(cen_x - loc_x)) / cen_x
                    x  = 0.5 + (1.0 - val)*(1.0 - val)
                    # print('red_th%f_x%f_val%f',th, x, val)

                # step3: check total cnt of rotation th (green)
                elif th_cnt < 3.14:
                    self.step = 3
                    rot_flag = True

                    th = th + 0.25
                    th_cnt = th_cnt +0.25
                    x = random.uniform(0.1, 0.5)

                # step0: random (while)
                else:
                    # init
                    th_cnt = 0.0
                    x_cnt  = 0.0
                    rot_flag = False
                    self.step = 0

                    x  = random.randint(-1, 1)
                    th = random.randint(-3, 3)                    
                """
                # --- use random ---
                value = random.randint(1,1000)
                if value < 500:
                    #x = 1
                    x = 1                    
                    th = 0

                elif value < 750:
                    x = 0
                    th = 3

                elif value < 1000:
                    x = 0
                    th = -3
                else:
                    x = 0
                    th = 0
                """

            target_speed = x
            target_turn = th

            # def 0.02
            del_speed  = 0.04
            # def 0.1
            del_turn   = 0.2

            if rot_flag:
                del_speed = 0.01
                del_turn  = 0.4

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + del_speed )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - del_speed )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + del_turn )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - del_turn )
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

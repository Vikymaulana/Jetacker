#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import math
import rospy
from std_srvs.srv import Trigger
from hiwonder_sdk.ackermann import AckermannChassis
from geometry_msgs.msg import Twist, TransformStamped
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

class Controller:
    def __init__(self):
        rospy.init_node('hiwonder_controller', anonymous=False)
        
        self.last_linear_x = 0
        self.last_linear_y = 0
        self.last_angular_z = 0
        joint_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制
        self.ackermann = AckermannChassis(joint_pub, track=0.222, wheelbase=0.213, wheel_diameter=101, pulse_per_cycle=4*11*90)  #44.0 * 90.0

        self.go_factor = rospy.get_param('~go_factor', 1.00)
        self.turn_factor = rospy.get_param('~turn_factor', 1.00)
        
        rospy.Subscriber('hiwonder_controller/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('cmd_vel', Twist, self.app_cmd_vel_callback)
        rospy.Service('hiwonder_controller/load_calibrate_param', Trigger, self.load_calibrate_param)
          
    def load_calibrate_param(self, msg):
        self.go_factor = rospy.get_param('~go_factor', 1.00)
        self.turn_factor = rospy.get_param('~turn_factor', 1.00)
        rospy.loginfo('load_calibrate_param')
        
        return [True, 'load_calibrate_param']
    
    def app_cmd_vel_callback(self, msg):
        if msg.linear.x > 0.2:
            msg.linear.x = 0.2
        if msg.linear.x < -0.2:
            msg.linear.x = -0.2
        if msg.linear.y > 0.2:
            msg.linear.y = 0.2
        if msg.linear.y < -0.2:
            msg.linear.y = -0.2
        if msg.angular.z > 0.5:
            msg.angular.z = 0.5
        if msg.angular.z < -0.5:
            msg.angular.z = -0.5
        self.cmd_vel_callback(msg)

    def cmd_vel_callback(self, msg):
        if abs(msg.linear.y) > 1e-8:
            linear_x = 0
        else:
            linear_x = self.go_factor*msg.linear.x
        linear_y = 0
        if msg.angular.z != 0:
            r = msg.linear.x/msg.angular.z
            if r != 0:
                angular_z = linear_x/r
            else:
                angular_z = 0
        else:
            angular_z = 0
        
        self.last_linear_x = linear_x
        self.last_linear_y = linear_y
        self.last_angular_z = angular_z
        if abs(msg.linear.z) > 1e-8 and abs(msg.linear.x) < 1e-8 and abs(msg.angular.z) < 1e-8:
            self.ackermann.set_velocity(linear_x, angular_z, reset_servo=False)
        else:
            self.ackermann.set_velocity(linear_x, angular_z)

if __name__ == "__main__":
    try:
        node = Controller()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        node.ackermann.reset_motors()
        sys.exit()

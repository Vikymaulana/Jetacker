#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/08
import os
import math
import rospy
from hiwonder_servo_msgs.msg import CommandDuration, JointState

class InitPose:
    def __init__(self):
        rospy.init_node('init_pose')

        joint1_angle = rospy.get_param('~joint1', 0)
        jointw_angle = rospy.get_param('~jointw', 0)
        namespace = rospy.get_namespace()
        
        joint1 = rospy.Publisher(namespace + 'joint1_controller/command_duration', CommandDuration, queue_size=1)
        jointw = rospy.Publisher(namespace + 'w_joint_controller/command_duration', CommandDuration, queue_size=1)

        while not rospy.is_shutdown():
            try:
                if rospy.get_param(namespace + 'hiwonder_servo_manager/init_finish') and rospy.get_param(namespace + 'joint_states_publisher/init_finish'):
                    print('******init_pose******')
                    break
            except:
                rospy.sleep(0.1)
        
        rospy.sleep(0.2)
        joint1.publish(CommandDuration(data=math.radians(joint1_angle), duration=2000))
        jointw.publish(CommandDuration(data=math.radians(jointw_angle), duration=2000))

if __name__ == '__main__':
    InitPose()

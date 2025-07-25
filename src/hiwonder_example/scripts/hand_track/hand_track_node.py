#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 手跟随
import rospy
import signal
import hiwonder_sdk.pid as pid
from geometry_msgs.msg import Twist
from hiwonder_kinematics import transform
from hiwonder_interfaces.msg import Point2D
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos

class HandTrackNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.image = None
        self.center = None
        self.running = True
        self.y_dis = 500

        self.pid_y = pid.PID(0.04, 0.0, 0.0)

        signal.signal(signal.SIGINT, self.shutdown)

        self.mecanum_pub = rospy.Publisher('/hiwonder_controller/cmd_vel', Twist, queue_size=1)  # 底盘控制
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制

        rospy.Subscriber('/hand_detect/center', Point2D, self.get_hand_callback)

        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/hiwonder_servo_manager/init_finish') and rospy.get_param('/joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)

        rospy.sleep(0.2)
        self.init_action()
        rospy.set_param('~init_finish', True)

        self.hand_track() 

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def init_action(self):
        set_servos(self.joints_pub, 1500, ((6, 500),))

        self.mecanum_pub.publish(Twist())

    def get_hand_callback(self, msg):
        if msg.width != 0:
            self.center = msg
        else:
            self.center = None

    def hand_track(self):
        while self.running:
            if self.center is not None:
                self.pid_y.SetPoint = self.center.width / 2
                self.pid_y.update(self.center.width - self.center.x)
                self.y_dis += self.pid_y.output
                if self.y_dis < 200:
                    self.y_dis = 200
                if self.y_dis > 800:
                    self.y_dis = 800

                set_servos(self.joints_pub, 20, ((6, self.y_dis), ))
                rospy.sleep(0.02)
            else:
                rospy.sleep(0.01)

        self.init_action() 
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    HandTrackNode('hand_track')

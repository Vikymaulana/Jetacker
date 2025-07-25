#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/23
# @author:aiden
# 颜色跟踪
import rospy
import signal
import hiwonder_sdk.pid as pid
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_interfaces.msg import ColorsInfo, ColorDetect
from hiwonder_interfaces.srv import SetColorDetectParam, SetString
from hiwonder_servo_controllers.bus_servo_control import set_servos

class ColorTrackNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.y_dis = 500
        self.center = None
        self.running = True
        self.start = False
        self.name = name

        self.pid_y = pid.PID(0.04, 0.0, 0.0)

        signal.signal(signal.SIGINT, self.shutdown)

        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制
        self.mecanum_pub = rospy.Publisher('/hiwonder_controller/cmd_vel', Twist, queue_size=1)  # 底盘控制

        rospy.Subscriber('/color_detect/color_info', ColorsInfo, self.get_color_callback)

        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色

        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/hiwonder_servo_manager/init_finish') and rospy.get_param('/joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        rospy.sleep(0.2)
        self.init_action()
        
        if rospy.get_param('~start', True):
            self.start_srv_callback(None)
            self.set_color_srv_callback(String('red'))

        rospy.set_param('~init_finish', True)

        self.color_track()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def init_action(self):
        set_servos(self.joints_pub, 1500, ((6, 500), ))
        self.mecanum_pub.publish(Twist())

    def set_color_srv_callback(self, msg):
        rospy.loginfo("set_color")

        msg_red = ColorDetect()
        msg_red.color_name = msg.data
        msg_red.detect_type = 'circle'
        res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([msg_red])
        if res.success and msg.data != '':
            print('start_track_' + msg_red.color_name)
        else:
            print('track_fail')

        return [True, 'set_color']

    def start_srv_callback(self, msg):
        rospy.loginfo("start color track")

        self.start = True

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop color track')

        self.start = False
        res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)()
        if res.success:
            print('set color success')
        else:
            print('set color fail')

        return TriggerResponse(success=True)

    def get_color_callback(self, msg):
        if msg.data != []:
            if msg.data[0].radius > 10:
                self.center = msg.data[0]
            else:
                self.center = None 
        else:
            self.center = None

    def color_track(self):
        while self.running:
            if self.center is not None and self.start:
                self.pid_y.SetPoint = self.center.width/2 
                self.pid_y.update(self.center.x)
                self.y_dis += self.pid_y.output
                if self.y_dis < 200:
                    self.y_dis = 200
                if self.y_dis > 800:
                    self.y_dis = 800
                set_servos(self.joints_pub, 20, ((6, self.y_dis), ))
                rospy.sleep(0.02)
            else:
                rospy.sleep(0.01)
        
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ColorTrackNode('color_track')

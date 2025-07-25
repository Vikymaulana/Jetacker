#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 语音控制移动
import os
import json
import math
import rospy
import signal
import numpy as np
import hiwonder_sdk.pid as pid
import hiwonder_sdk.misc as misc
from hiwonder_sdk import buzzer
import sensor_msgs.msg as sensor_msg
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from xf_mic_asr_offline import voice_play

MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree
CAR_WIDTH = 0.4  # meter


class VoiceControlNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.angle = None
        self.words = None
        self.running = True
        self.haved_stop = False
        self.lidar_follow = False
        self.start_follow = False
        self.last_status = Twist()
        self.threshold = 3
        self.speed = 0.3
        self.stop_dist = 0.4
        self.count = 0
        self.scan_angle = math.radians(45)

        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)

        self.language = os.environ['LANGUAGE']
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        self.mecanum_pub = rospy.Publisher('/hiwonder_controller/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/voice_words', String, self.words_callback)
        rospy.Subscriber('/mic/awake/angle', Int32, self.angle_callback)
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/xf_asr_offline_node/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        self.play('running')
        self.mecanum_pub.publish(Twist())
        signal.signal(signal.SIGINT, self.shutdown)

        rospy.loginfo('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo(
            '控制指令: 左转 右转 前进 后退 过来(Voice command: turn left/turn right/go forward/go backward/come here)')

        self.time_stamp = rospy.get_time()
        self.current_time_stamp = rospy.get_time()
        self.run()

    def play(self, name):
        voice_play.play(name, language=self.language)

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    def words_callback(self, msg):
        self.words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            self.words = self.words.replace(' ', '')
        print('words:', self.words)
        if self.words is not None and self.words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                                         '失败10次(Fail-10-times']:
            pass
        elif self.words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif self.words == '休眠(Sleep)':
            buzzer.on()
            rospy.sleep(0.05)
            buzzer.off()

    def angle_callback(self, msg):
        self.angle = msg.data
        print('angle:', self.angle)
        self.start_follow = False
        # buzzer.on()
        # rospy.sleep(0.1)
        # buzzer.off()
        self.mecanum_pub.publish(Twist())

    def run(self):
        while not rospy.is_shutdown() and self.running:
            if self.words is not None:
                twist = Twist()
                if self.words == '前进' or self.words == 'go forward':
                    self.play('go')
                    self.time_stamp = rospy.get_time() + 2
                    twist.linear.x = 0.2
                elif self.words == '后退' or self.words == 'go backward':
                    self.play('back')
                    self.time_stamp = rospy.get_time() + 2
                    twist.linear.x = -0.2
                elif self.words == '左转' or self.words == 'turn left':
                    self.play('turn_left')
                    self.time_stamp = rospy.get_time() + 2
                    twist.linear.x = 0.15
                    twist.angular.z = twist.linear.x*math.tan(0.6)/0.213
                elif self.words == '右转' or self.words == 'turn right':
                    self.play('turn_right')
                    self.time_stamp = rospy.get_time() + 2
                    twist.linear.x = 0.15
                    twist.angular.z = -twist.linear.x*math.tan(0.6)/0.213
                elif self.words == '休眠(Sleep)':
                    rospy.sleep(0.01)
                self.words = None
                self.haved_stop = False
                self.mecanum_pub.publish(twist)
            else:
                rospy.sleep(0.01)
            self.current_time_stamp = rospy.get_time()
            if self.time_stamp < self.current_time_stamp and not self.haved_stop:
                self.mecanum_pub.publish(Twist())
                self.haved_stop = True

if __name__ == "__main__":
    VoiceControlNode('voice_control')

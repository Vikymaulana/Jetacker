#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 物体跟踪
import cv2
import math
import rospy
import numpy as np
import hiwonder_sdk.pid as pid
import hiwonder_sdk.fps as fps
import hiwonder_sdk.misc as misc
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos

class KCFNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.y_dis = 500
        self.pid_y = pid.PID(0.04, 0.0, 0.0)

        self.fps = fps.FPS()
        
        self.mouse_click = False
        self.selection = None   # 实时跟踪鼠标的跟踪区域
        self.track_window = None   # 要检测的物体所在区域
        self.drag_start = None   # 标记，是否开始拖动鼠标
        self.start_circle = True
        self.start_click = False

        self.tracker = cv2.legacy.TrackerMedianFlow_create()
        fs = cv2.FileStorage("custom_csrt.xml", cv2.FILE_STORAGE_READ)
        fn = fs.getFirstTopLevelNode()
        #self.tracker.save('default_csrt.xml')
        self.tracker.read(fn)

        cv2.namedWindow(name, 1)
        cv2.setMouseCallback(name, self.onmouse)
        camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制
        self.image_sub = rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, self.image_callback, queue_size=1)  # 订阅目标检测节点
        self.mecanum_pub = rospy.Publisher('/hiwonder_controller/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.2)
        self.mecanum_pub.publish(Twist())
        rospy.set_param('~init_finish', True)

    def image_callback(self, ros_image: Image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
        
        try:
            result_image = self.image_proc(np.copy(rgb_image))
        except BaseException as e:
            print(e)
            result_image = rgb_image
        cv2.imshow(self.name, cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        key = cv2.waitKey(1)
        if key != -1:
            self.mecanum_pub.publish(Twist())
            rospy.signal_shutdown('shutdown')
     
    #鼠标点击事件回调函数
    def onmouse(self, event, x, y, flags, param):   
        if event == cv2.EVENT_LBUTTONDOWN:       #鼠标左键按下
            self.mouse_click = True
            self.drag_start = (x, y)       #鼠标起始位置
            self.track_window = None
        if self.drag_start:       #是否开始拖动鼠标，记录鼠标位置
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:        #鼠标左键松开
            self.mouse_click = False
            self.drag_start = None
            self.track_window = self.selection
            self.selection = None
        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse_click = False
            self.selection = None   # 实时跟踪鼠标的跟踪区域
            self.track_window = None   # 要检测的物体所在区域
            self.drag_start = None   # 标记，是否开始拖动鼠标
            self.start_circle = True
            self.start_click = False
            self.mecanum_pub.publish(Twist())
            self.tracker = cv2.legacy.TrackerMedianFlow_create()
            fs = cv2.FileStorage("custom_csrt.xml", cv2.FILE_STORAGE_READ)
            fn = fs.getFirstTopLevelNode()
            #tracker.save('default_csrt.xml')
            self.tracker.read(fn)
            self.y_dis = 500
            set_servos(self.joints_pub, 100, ((6, self.y_dis), ))

    def image_proc(self, image):
        if self.start_circle:
            # 用鼠标拖拽一个框来指定区域             
            if self.track_window:      #跟踪目标的窗口画出后，实时标出跟踪目标
                cv2.rectangle(image, (self.track_window[0], self.track_window[1]), (self.track_window[2], self.track_window[3]), (0, 0, 255), 2)
            elif self.selection:      #跟踪目标的窗口随鼠标拖动实时显示
                cv2.rectangle(image, (self.selection[0], self.selection[1]), (self.selection[2], self.selection[3]), (0, 0, 255), 2)
            if self.mouse_click:
                self.start_click = True
            if self.start_click:
                if not self.mouse_click:
                    self.start_circle = False
            if not self.start_circle:
                print('start tracking')
                bbox = (self.track_window[0], self.track_window[1], self.track_window[2] - self.track_window[0], self.track_window[3] - self.track_window[1])
                self.tracker.init(image, bbox)
        else:
            twist = Twist()
            ok, bbox = self.tracker.update(image)
            if ok and min(bbox) > 0:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
                center = [(p1[0] + p2[0])/2, (p1[1] + p2[1])/2]
                self.pid_y.SetPoint = 320 
                self.pid_y.update(center[0])
                self.y_dis += self.pid_y.output
                if self.y_dis < 200:
                    self.y_dis = 200
                if self.y_dis > 800:
                    self.y_dis = 800
                set_servos(self.joints_pub, 20, ((6, self.y_dis), ))
                rospy.sleep(0.02)
            else:
                # Tracking failure
                cv2.putText(image, "Tracking failure detected !", (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
        self.fps.update()
        result_image = self.fps.show_fps(image)
        
        return result_image

if __name__ == '__main__':
    kcf_node = KCFNode('kcf_tracking')
    try:
        rospy.spin()
    except Exception as e:
        kcf_node.mecanum_pub.publish(Twist())
        rospy.logerr(str(e))

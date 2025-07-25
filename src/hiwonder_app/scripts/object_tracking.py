#!/usr/bin/env python3
# encoding: utf-8
# 颜色跟踪
import cv2
import math
import rospy
import signal
import threading
import numpy as np
import hiwonder_sdk.pid as pid
import hiwonder_sdk.misc as misc
import geometry_msgs.msg as geo_msg
import hiwonder_sdk.common as common
from hiwonder_app.common import ColorPicker, Heart
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from hiwonder_servo_controllers.bus_servo_control import set_servos
from sensor_msgs.msg import CameraInfo, Image, CompressedImage, LaserScan
from hiwonder_interfaces.srv import SetPoint, SetPointRequest, SetPointResponse
from hiwonder_interfaces.srv import SetFloat64, SetFloat64Request, SetFloat64Response

class ObjectTracker:
    def __init__(self, color, node):
        self.node = node
        self.pid_yaw = pid.PID(0.006, 0.0, 0.0)
        self.pid_dist = pid.PID(0.002, 0.0, 0.00)
        self.pid_servo = pid.PID(0.02, 0.0, 0.0)
        self.last_color_circle = None
        self.lost_target_count = 0
        self.target_lab, self.target_rgb = color
        self.weight_sum = 1.0
        self.servo = 500

    def __call__(self, image, result_image, threshold):
        twist = geo_msg.Twist()
        image = cv2.resize(image, (320, 240))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # RGB转LAB空间
        image = cv2.GaussianBlur(image, (5, 5), 5)

        target_color = [self.target_lab, ]
        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        mask = cv2.inRange(image, tuple(target_color[1]), tuple(target_color[2]))  # 二值化
        # cv2.imshow('mask', cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))
        # cv2.waitKey(1)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # 计算各个轮廓的面积
        contour_area = list(filter(lambda c: c[1] > 40, contour_area))  # 剔除>面积过小的轮廓
        circle = None
        if len(contour_area) > 0:
            if self.last_color_circle is None:
                contour, area = max(contour_area, key=lambda c_a: c_a[1])
                circle = cv2.minEnclosingCircle(contour)
            else:
                (last_x, last_y), last_r = self.last_color_circle
                circles = map(lambda c: cv2.minEnclosingCircle(c[0]), contour_area)
                circle_dist = list(map(lambda c: (c, math.sqrt(((c[0][0] - last_x) ** 2) + ((c[0][1] - last_y) ** 2))),
                                       circles))
                circle, dist = min(circle_dist, key=lambda c: c[1])
                if dist < 100:
                    circle = circle
        if circle is not None:
            self.lost_target_count = 0
            (x, y), r = circle
            x = x / 320 * 640
            y = y / 240 * 480
            r = r / 320 * 640

            cv2.circle(result_image, (320, 340), 5, (255, 255, 0), -1)
            result_image = cv2.circle(result_image, (int(x), int(y)), int(r), (255 - self.target_rgb[0],
                                                                               255 - self.target_rgb[1],
                                                                               255 - self.target_rgb[2]), 2)
            vx = 0
            vw = 0
            if abs(x - 320) > 10:
                self.pid_servo.SetPoint = 320
                self.pid_servo.update(x)
                #print(self.servo, x, self.pid_servo.output)
                self.servo += self.pid_servo.output
                if self.servo > 650:
                    self.servo = 650
                if self.servo < 350:
                    self.servo = 350
            if abs(y - 340) > 20:
                self.pid_dist.update(y - 340)
                twist.linear.x = misc.set_range(self.pid_dist.output, -0.35, 0.35)
            else:
                self.pid_dist.clear()
            if abs(self.servo - 500) > 20:
                self.pid_yaw.update(self.servo - 500)
                twist.angular.z = -twist.linear.x*math.tan(misc.set_range(self.pid_yaw.output, -0.6, 0.6))/0.213
            else:
                self.pid_yaw.clear()

        return result_image, twist, self.servo


class OjbectTrackingNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.image = None
        self.start = False
        self.color_picker = None
        self.tracker = None
        self.is_running = False
        self.threshold = 0.2
        self.dist_threshold = 0.3
        self.lock = threading.RLock()
        self.image_sub = None
        self.running = True
        signal.signal(signal.SIGINT, self.shutdown)
        self.mecanum_pub = rospy.Publisher('/hiwonder_controller/cmd_vel', geo_msg.Twist, queue_size=1)
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)
        self.enter_srv = rospy.Service('~enter', Trigger, self.enter_srv_callback)
        self.exit_srv = rospy.Service('~exit', Trigger, self.exit_srv_callback)
        self.set_running_srv = rospy.Service('~set_running', SetBool, self.set_running_srv_callback)
        self.set_target_color_srv = rospy.Service('~set_target_color', SetPoint,
                                                  self.set_target_color_srv_callback)
        self.get_target_color_srv = rospy.Service('~get_target_color', Trigger,
                                                  self.get_target_color_srv_callback)
        self.set_threshold_srv = rospy.Service('~set_threshold', SetFloat64,
                                               self.set_threshold_srv_callback)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.image_proc()

    def shutdown(self, signum, frame):
        self.running = False

    def enter_srv_callback(self, _):
        rospy.loginfo("object tracking enter")
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()
            except Exception as e:
                rospy.logerr(str(e))
            self.start = False
            self.threshold = 0.2
            self.tracker = None
            self.color_picker = None
            self.dist_threshold = 0.3
            depth_camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')
            self.image_sub = rospy.Subscriber('/%s/rgb/image_raw' % depth_camera, Image, self.image_callback)
            set_servos(self.joints_pub, 1000, ((1, 300), (2, 500), (3, 210), (4, 40), (5, 665), (6, 500)))
            self.mecanum_pub.publish(geo_msg.Twist())
            self.is_running = True
        return TriggerResponse(success=True)

    def exit_srv_callback(self, _):
        rospy.loginfo("object tracking exit")
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()
            except Exception as e:
                rospy.logerr(str(e))
            self.image = None
            self.start = False
            self.is_running = False
            self.color_picker = None
            self.tracker = None
            self.dist_threshold = 0.3
            self.mecanum_pub.publish(geo_msg.Twist())
        return TriggerResponse(success=True)

    def set_target_color_srv_callback(self, req: SetPointRequest):
        rospy.loginfo("set_target_color")
        with self.lock:
            x, y = req.data.x, req.data.y
            if x == -1 and y == -1:
                self.color_picker = None
                self.tracker = None
            else:
                self.tracker = None
                self.color_picker = ColorPicker(req.data, 20)
            self.mecanum_pub.publish(geo_msg.Twist())
        return SetPointResponse(success=True)

    def get_target_color_srv_callback(self, _):
        rospy.loginfo("get_target_color")
        rsp = TriggerResponse(success=False, message="")
        with self.lock:
            if self.tracker is not None:
                rsp.success = True
                rgb = self.tracker.target_rgb
                rsp.message = "{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return rsp

    def set_running_srv_callback(self, req: SetBoolRequest):
        rospy.loginfo("set_running")
        with self.lock:
            self.start = req.data
            if not self.start:
                self.mecanum_pub.publish(geo_msg.Twist())
        return SetBoolResponse(success=req.data)

    def set_threshold_srv_callback(self, req: SetFloat64Request):
        rospy.loginfo("threshold")
        with self.lock:
            self.threshold = req.data
            return SetFloat64Response(success=True)

    def image_proc(self):
        while self.running:
            if self.is_running and self.image is not None:
                result_image = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)  # 显示结果用的画面
                with self.lock:
                    # 颜色拾取器和识别追踪互斥, 如果拾取器存在就开始拾取
                    if self.color_picker is not None:  # 拾取器存在
                        target_color, result_image = self.color_picker(self.image, result_image)
                        if target_color is not None:
                            self.color_picker = None
                            self.tracker = ObjectTracker(target_color, self)
                    else:
                        if self.tracker is not None:
                            try:
                                result_image, twist, servo = self.tracker(self.image, result_image, self.threshold)
                                if self.start:
                                    set_servos(self.joints_pub, 20, ((6, servo), ))
                                    self.mecanum_pub.publish(twist)
                                else:
                                    self.tracker.pid_dist.clear()
                                    self.tracker.pid_yaw.clear()
                            except Exception as e:
                                rospy.logerr(str(e))
                cv2.line(result_image, (310, 340), (330, 340), (0, 255, 255), 2)
                cv2.line(result_image, (320, 330), (320, 350), (0, 255, 255), 2)
                self.result_publisher.publish(common.cv2_image2ros(result_image, self.name))
            else:
                rospy.sleep(0.01)

    def image_callback(self, ros_image):
        self.image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面

if __name__ == "__main__":
    OjbectTrackingNode('object_tracking')

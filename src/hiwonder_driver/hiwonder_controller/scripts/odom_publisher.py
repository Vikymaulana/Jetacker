#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import tf2_ros
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseWithCovarianceStamped

ODOM_POSE_COVARIANCE = [1e-9, 0, 0, 0, 0, 0, 
                        0, 1e-3, 1e-9, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]
 
ODOM_TWIST_COVARIANCE = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 0.1]

def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

def qua2rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
  
    return roll, pitch, yaw

class Controller:
    def __init__(self):
        rospy.init_node('hiwonder_odom_publisher', anonymous=False)
        
        self.freq = rospy.get_param('~freq', 50)
        self.pub_odom_topic = rospy.get_param('~pub_odom_topic','false')
        self.odom_topic = rospy.get_param('~odom_topic','odom_raw')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_footprint')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.pub_odom_transform = rospy.get_param('~pub_odom_transform','true')

        self.linear_factor = rospy.get_param('~linear_correction_factor', 1.00)
        self.angular_factor = rospy.get_param('~angular_correction_factor', 1.00)
        self.cmd_vel = rospy.get_param('~cmd_vel', '/hiwonder_controller/cmd_vel')
        
        self.lock = threading.RLock()
        
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0
        self.pose_yaw = 0

        if self.pub_odom_topic:
            self.odom = Odometry()
            self.odom.header.frame_id = self.odom_frame_id
            self.odom.child_frame_id = self.base_frame_id
            
            self.odom.pose.covariance = ODOM_POSE_COVARIANCE
            self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
            
            self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
            self.dt = 1.0/self.freq
           
            self.odom_broadcaster = tf2_ros.TransformBroadcaster()  # 定义TF变换广播者
            self.odom_trans = TransformStamped()
            self.odom_trans.header.frame_id = self.odom_frame_id
            self.odom_trans.child_frame_id = self.base_frame_id

            rospy.Timer(rospy.Duration(self.dt), self.cal_odom_fun)
        
        self.pose_pub = rospy.Publisher('set_pose', PoseWithCovarianceStamped, queue_size=1)
        rospy.Subscriber('set_odom', Pose2D, self.set_odom)
        rospy.Subscriber(self.cmd_vel, Twist, self.cmd_vel_callback)
        rospy.Subscriber('cmd_vel', Twist, self.app_cmd_vel_callback)
          
    def set_odom(self, msg):
        self.odom = Odometry()
        self.odom.header.frame_id = self.odom_frame_id
        self.odom.child_frame_id = self.base_frame_id
        
        self.odom.pose.covariance = ODOM_POSE_COVARIANCE
        self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom.pose.pose.position.x = msg.x
        self.odom.pose.pose.position.y = msg.y
        self.pose_yaw = msg.theta
        self.odom.pose.pose.orientation = rpy2qua(0, 0, self.pose_yaw)
        
        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0
        
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.odom_frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.pose = self.odom.pose.pose
        pose.pose.covariance = ODOM_POSE_COVARIANCE
        self.pose_pub.publish(pose)

    def load_calibrate_param(self, msg):
        self.linear_factor = rospy.get_param('~linear_correction_factor', 1.00)
        self.angular_factor = rospy.get_param('~angular_correction_factor', 1.00)
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
            self.linear_x = 0
        else:
            self.linear_x = msg.linear.x 
        self.linear_y = 0
        self.angular_z = msg.angular.z
         
    def cal_odom_fun(self, event):
        self.odom.header.stamp = rospy.Time.now()
        
        self.x += math.cos(self.pose_yaw)*self.linear_x*self.dt - math.sin(self.pose_yaw)*self.linear_y*self.dt
        self.y += math.sin(self.pose_yaw)*self.linear_x*self.dt + math.cos(self.pose_yaw)*self.linear_y*self.dt 

        self.odom.pose.pose.position.x = self.linear_factor*self.x 
        self.odom.pose.pose.position.y = self.linear_factor*self.y
        self.odom.pose.pose.position.z = 0
       
        self.yaw += self.angular_z*self.dt
        self.pose_yaw = self.yaw

        self.odom.pose.pose.orientation = rpy2qua(0, 0, self.pose_yaw)
        self.odom.twist.twist.linear.x = self.linear_x
        self.odom.twist.twist.linear.y = self.linear_y
        self.odom.twist.twist.angular.z = self.angular_z

        self.odom_trans.header.stamp = rospy.Time.now()
        self.odom_trans.transform.translation.x = self.odom.pose.pose.position.x
        self.odom_trans.transform.translation.y = self.odom.pose.pose.position.y
        self.odom_trans.transform.translation.z = 0
        self.odom_trans.transform.rotation = self.odom.pose.pose.orientation

        #self.odom_broadcaster.sendTransform(self.odom_trans)

        self.odom_pub.publish(self.odom)

if __name__ == "__main__":
    try:
        node = Controller()
        rospy.spin()
    except Exception as e:
        print(e)

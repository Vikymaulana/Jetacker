#!/usr/bin/env python

""" calibrate_angular.py - Version 1.1 2013-12-20

    Rotate the robot 360 degrees to check the odometry parameters of the base controller.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import tf
import rospy
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, Quaternion
from math import radians, copysign, pi, atan2, asin
from hiwonder_calibration.cfg import CalibrateAngularConfig

def qua2rpy(quat):
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = asin(2 * (w * y - x * z))
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    
    return roll, pitch, yaw

def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
        
    return res

class CalibrateAngular():
    def __init__(self):
        # Give the node a name
        rospy.init_node('calibrate_angular', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        
        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)
        
        # The test angle is 360 degrees
        self.test_angle = radians(rospy.get_param('~test_angle', 360.0))

        self.speed = rospy.get_param('~speed', 0.5) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', 1)) # degrees converted to radians
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.turn_scale_correction = rospy.get_param('~motor_turn_scale_correction', 1.0)
        self.start_test_angular = rospy.get_param('~start_test_angular', True)
        self.start_test_turn = rospy.get_param('~start_test_turn', False)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/hiwonder_controller/cmd_vel', Twist, queue_size=5)
        
        # Fire up the dynamic_reconfigure server
        dyn_server = Server(CalibrateAngularConfig, self.dynamic_reconfigure_callback)
        
        # Connect to the dynamic_reconfigure server
        dyn_client = dynamic_reconfigure.client.Client("calibrate_angular", timeout=60)
        
        # The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
        
        reverse = 1
        start = True
        use_time = 0
        start_time = 0
        while not rospy.is_shutdown():
            if self.start_test_angular:
                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()
                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle
                                
                # Alternate directions between tests
                reverse = -reverse
                
                while abs(error) > self.tolerance and self.start_test_angular:
                    if rospy.is_shutdown():
                        return
                    
                    # Rotate the robot to reduce the error
                    move_cmd = Twist()
                    move_cmd.linear.x = 0.2
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                 
                    # Get the current rotation angle from tf                   
                    self.odom_angle = self.get_odom_angle()
                    
                    # Compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * normalize_angle(self.odom_angle - last_angle)
                    
                    # Add to our total angle so far
                    turn_angle += delta_angle

                    # Compute the new error
                    error = self.test_angle - turn_angle

                    # Store the current angle for the next comparison
                    last_angle = self.odom_angle
                                    
                # Stop the robot
                self.cmd_vel.publish(Twist())
                
                # Update the status flag
                self.start_test_angular = False
                params = {'start_test_angular': False}
                dyn_client.update_configuration(params)
            elif self.start_test_turn:
                if start:
                    use_time = self.test_angle/self.speed
                    start = False
                    start_time = rospy.get_time()
                else:
                    # Are we close enough?
                    if not self.start_test_turn or abs(rospy.get_time() - start_time) > use_time:
                        start = True
                        self.cmd_vel.publish(Twist())
                        self.start_test_turn = False
                        params = {'start_test_turn': False}
                        rospy.loginfo(params)
                        dyn_client.update_configuration(params)
                    else:
                        # If not, move in the appropriate direction
                        move_cmd = Twist()
                        move_cmd.linear.x = 0.2
                        move_cmd.angular.z = self.turn_scale_correction*self.speed
                        self.cmd_vel.publish(move_cmd)
            else:
                self.cmd_vel.publish(Twist())
            rospy.sleep(0.5)
                    
        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        # Convert the rotation from a quaternion to an Euler angle
        return qua2rpy(Quaternion(*rot))[2]
            
    def dynamic_reconfigure_callback(self, config, level):
        self.test_angle =  radians(config['test_angle'])
        self.speed = config['speed']
        self.tolerance = radians(config['tolerance'])
        self.odom_angular_scale_correction = config['odom_angular_scale_correction']
        self.turn_scale_correction = config['motor_turn_scale_correction']
        self.start_test_turn = config['start_test_turn']
        self.start_test_angular = config['start_test_angular']
        if self.start_test_turn:
            self.star_test_angular = False
        return config
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        CalibrateAngular()
    except BaseException as e:
        print(e)
        rospy.loginfo("Calibration terminated.")

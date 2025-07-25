#!/usr/bin/env python

""" calibrate_linear.py - Version 1.1 2013-12-20

    Move the robot 1.0 meter to check on the PID parameters of the base controller.

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
from math import copysign, sqrt, pow
from geometry_msgs.msg import Twist, Point
from dynamic_reconfigure.server import Server
from hiwonder_calibration.cfg import CalibrateLinearConfig

class CalibrateLinear():
    def __init__(self):
        # Give the node a name
        rospy.init_node('calibrate_linear', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)
        
        # Set the distance to travel
        self.test_distance = rospy.get_param('~test_distance', 1.0) # meters
        self.speed = rospy.get_param('~speed', 0.15) # meters per second
        self.tolerance = rospy.get_param('~tolerance', 0.01) # meters
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.go_scale_correction = rospy.get_param('~motor_go_scale_correction', 1.0)
        self.start_test_linear = rospy.get_param('~start_test_linear', True)
        self.start_test_go = rospy.get_param('~start_test_go', False)
        
        cmd_vel = rospy.get_param('~cmd_vel', "/hiwonder_controller/cmd_vel")
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher(cmd_vel, Twist, queue_size=5)
        
        # Fire up the dynamic_reconfigure server
        dyn_server = Server(CalibrateLinearConfig, self.dynamic_reconfigure_callback)
        
        # Connect to the dynamic_reconfigure server
        dyn_client = dynamic_reconfigure.client.Client("calibrate_linear", timeout=60)
 
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
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
  
        self.position = Point()
        
        # Get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()
        
        x_start = self.position.x
        y_start = self.position.y
            
        move_cmd = Twist()
        start = True
        use_time = 0
        start_time = 0
        while not rospy.is_shutdown():
            # Stop the robot by default
            move_cmd = Twist()
            if self.start_test_linear:
                # Get the current position from the tf transform between the odom and base frames
                self.position = self.get_position()
                
                # Compute the Euclidean distance from the target point
                distance = sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2))
                                
                # Correct the estimated distance by the correction factor
                distance *= self.odom_linear_scale_correction

                # How close are we?
                error =  distance - self.test_distance
                
                # Are we close enough?
                if not self.start_test_linear or abs(error) <  self.tolerance:
                    self.start_test_linear = False
                    params = {'start_test_linear': False}
                    rospy.loginfo(params)
                    dyn_client.update_configuration(params)
                else:
                    # If not, move in the appropriate direction
                    move_cmd.linear.x = copysign(self.speed, -1 * error)
            elif self.start_test_go:
                if start:
                    use_time = self.test_distance/self.speed
                    #print(use_time, self.speed, self.go_scale_correction*self.speed)
                    start = False
                    start_time = rospy.get_time()
                else:
                    # Are we close enough?
                    if not self.start_test_go or abs(rospy.get_time() - start_time) > use_time:
                        start = True
                        self.start_test_go = False
                        params = {'start_test_go': False}
                        rospy.loginfo(params)
                        dyn_client.update_configuration(params)
                    else:
                        # If not, move in the appropriate direction
                        move_cmd.linear.x = self.go_scale_correction*self.speed
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
            
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def dynamic_reconfigure_callback(self, config, level):
        self.test_distance = config['test_distance']
        self.speed = config['speed']
        self.tolerance = config['tolerance']
        self.odom_linear_scale_correction = config['odom_linear_scale_correction']
        self.go_scale_correction = config['motor_go_scale_correction']
        self.start_test_linear = config['start_test_linear']
        self.start_test_go = config['start_test_go']
        if self.start_test_go:
            self.star_test_linear = False
        return config
        
    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans)
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        CalibrateLinear()
        rospy.spin()
    except:
        rospy.loginfo("Calibration terminated.")

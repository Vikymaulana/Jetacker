#!/usr/bin/python3
# coding=utf8
#
# 阿克曼底盘控制(Ackermann wheel chassis control)
#
import os
import math
import time
import rospy
import numpy as np
import hiwonder_sdk.encoder_motor as motor
from hiwonder_servo_controllers.bus_servo_control import set_servos

class AckermannChassis:
    # track = 0.222  # mm (left right wheel distance)
    # wheelbase = 0.213  # mm front rear wheel distance
    # WHEEL_DIAMETER = 101  # mm
    # PULSE_PER_CYCLE = 44

    def __init__(self, joint_pub, track=0.222, wheelbase=0.213, wheel_diameter=101, pulse_per_cycle=44 * 90):
        self.motor_controller = motor.EncoderMotorController(1)
        self.track = track
        self.wheelbase = wheelbase
        self.wheel_diameter = wheel_diameter
        self.pulse_per_cycle = pulse_per_cycle
        self.linear_speed = 0
        self.angular_speed = 0
        self.joints_pub = joint_pub

    def speed_covert(self, speed):
        """
        covert speed mm/s to pulse/10ms
        :param speed:
        :return:
        """
        return speed / (math.pi * self.wheel_diameter) * self.pulse_per_cycle * 0.01  # pulse/10ms

    def reset_motors(self):
        self.motor_controller.set_speed((0, 0, 0, 0))
        self.linear_speed = 0
        self.angular_speed = 0

    def set_velocity(self, linear_speed, angular_speed, fake=False, reset_servo=True):
        if linear_speed != 0:
            if angular_speed != 0:
                theta = math.atan(self.wheelbase*angular_speed/linear_speed)
                steering_angle = theta
                if abs(steering_angle) > math.radians(37):
                    return None
                set_servos(self.joints_pub, 20, ((9, 1000*math.degrees(steering_angle)/240 + 500), ))
            else:
                set_servos(self.joints_pub, 20, ((9, 500), ))
        else:
            if reset_servo:
                set_servos(self.joints_pub, 20, ((9, 500), ))
        vr = 1000*(linear_speed + angular_speed*self.track/2)
        vl = 1000*(linear_speed - angular_speed*self.track/2)
        v_s = [int(self.speed_covert(v)) for v in [0, -vl, 0, vr]]
        if fake:
            return v_s
        self.motor_controller.set_speed(v_s)
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

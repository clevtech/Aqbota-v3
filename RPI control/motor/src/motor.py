#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import RPi.GPIO as io
io.setmode(io.BCM)

from adafruit_servokit import ServoKit
pwm = ServoKit(channels=16)

import time

import roslib; 
# roslib.load_manifest('teleop_twist_keyboard')
import rospy

import math

from geometry_msgs.msg import Twist

WHEEL_DIAMETER = 0.54
COUNTS_PER_REV = 8
MAX_RPM = 16
LR_WHEELS_DISTANCE = 0.25
WHEEL_CIRCUS = WHEEL_DIAMETER * math.pi
FREQUENCY = 5000
pwm.set_pwm_freq(FREQUENCY)

last_command = 0


# Motor leftforward - 0, leftbackward - 1, rightforward - 2, rightbackward - 3
def motor_spin(motor, rpm):
    if abs(rpm) > 16:
        if rpm > 16:
            rpm = 16
        else:
            rpm = -16
    if motor == "L":
        end = abs(rpm)*(4096/16) - 1
        if rpm > 0: 
            pwm.set_pwm(1, 0, 0)
            pwm.set_pwm(0, 0, end)
        else:
            pwm.set_pwm(0, 0, 0)
            pwm.set_pwm(1, 0, end)
    else:
        end = abs(rpm)*(4096/16) - 1
        if rpm > 0: 
            pwm.set_pwm(3, 0, 0)
            pwm.set_pwm(2, 0, end)
        else:
            pwm.set_pwm(2, 0, 0)
            pwm.set_pwm(3, 0, end)

def motor_stop():
    pwm.set_pwm(0, 0, 0)
    pwm.set_pwm(1, 0, 0)
    pwm.set_pwm(2, 0, 0)
    pwm.set_pwm(3, 0, 0)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

def callback(data):

    vel_x_min = data.linear.x*MAX_RPM
    vel_z_min = data.angular.z*MAX_RPM
    tangential_vel = vel_z_min * (LR_WHEELS_DISTANCE / 2) * MAX_RPM

    x_rpm = vel_x_min/WHEEL_CIRCUS
    tan_rpm = vel_z_min/WHEEL_CIRCUS

    MotorL = x_rpm - tan_rpm
    MotorR = x_rpm + tan_rpm


if __name__ == '__main__':
    last_command = int(round(time.time() * 1000))
    listener()

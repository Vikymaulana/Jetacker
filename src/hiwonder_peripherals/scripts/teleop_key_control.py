#!/usr/bin/env python3
# encoding: utf-8
import math
import rospy
from geometry_msgs.msg import Twist

import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

LIN_VEL = 0.2
ANG_VEL = LIN_VEL/(0.213/math.tan(0.628))

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = rospy.get_time()
        while True:
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('hiwonder_teleop')
    mecanum_pub = rospy.Publisher('hiwonder_controller/cmd_vel', Twist, queue_size=10)

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0
    last_x = 0
    last_z = 0
    count = 0
    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w':
                count = 0
                control_linear_vel = LIN_VEL
            elif key == 'a':
                count = 0
                control_angular_vel = ANG_VEL
            elif key == 'd':
                count = 0
                control_angular_vel = -ANG_VEL
            elif key == 's':
                count = 0
                control_linear_vel = -LIN_VEL
            elif key == '':
                count += 1
                if count > 5:
                    count = 0
                    if control_angular_vel != 0:
                        control_angular_vel = 0
                        control_linear_vel = 0
            else:
                count = 0
                if (key == '\x03'):
                    break
            twist = Twist()
            twist.linear.x = control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel

            if last_x != control_linear_vel or last_z != control_angular_vel or control_angular_vel != 0:
                mecanum_pub.publish(twist)
            
            last_x = control_linear_vel
            last_z = control_angular_vel
    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        mecanum_pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

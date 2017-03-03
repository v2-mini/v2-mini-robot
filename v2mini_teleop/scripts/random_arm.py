#!/usr/bin/env python

"""robot arm routines"""

import rospy
from std_msgs.msg import Float64
from time import sleep


j1arm_pub = rospy.Publisher('arm_joint1_controller/command', Float64, queue_size=1)
j2arm_pub = rospy.Publisher('arm_joint2_controller/command', Float64, queue_size=1)
j3arm_pub = rospy.Publisher('arm_joint3_controller/command', Float64, queue_size=1)
j4arm_pub = rospy.Publisher('arm_joint4_controller/command', Float64, queue_size=1)
j5arm_pub = rospy.Publisher('arm_joint5_controller/command', Float64, queue_size=1)


def routine_1():
    """pickup object off table"""

    rospy.init_node('random_motion')
    # r = rospy.Rate(30)

    while not rospy.is_shutdown():

        # init arm position
        rest_pos()
        sleep(2)

        #

        # end routine
        break


def rest_pos():
    """arm resting position"""
    j1arm_pub.publish(0.11)
    j2arm_pub.publish(1.3)
    j3arm_pub.publish(-0.12)
    j4arm_pub.publish(-0.25)
    j5arm_pub.publish(0.16)

if __name__ == '__main__':
    routine_1()

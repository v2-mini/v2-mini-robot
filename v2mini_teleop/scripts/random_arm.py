#!/usr/bin/env python

"""very basic hardcoded robot arm routines for demo"""

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from time import sleep

arm_publishers = [
    rospy.Publisher('arm_joint1_controller/command', Float64, queue_size=1),
    rospy.Publisher('arm_joint2_controller/command', Float64, queue_size=1),
    rospy.Publisher('arm_joint3_controller/command', Float64, queue_size=1),
    rospy.Publisher('arm_joint4_controller/command', Float64, queue_size=1),
    rospy.Publisher('arm_joint5_controller/command', Float64, queue_size=1),
]

arm_states = {
    "side_rest": [0.11, 1.3, -0.12, -0.25, 0.16],
    "half_reach": [0.42, 1.35, -0.12, -1.4, 0.15],
    "full_reach": [1.55, 1.59, -0.38, -0.22, 0.24],
    "arm2chest": [0.48, 1.54, -1.04, -1.25, 0.23],
}


def main():
    rospy.init_node('random_motion')
    rospy.Subscriber("routines", String, sub_callback)
    rospy.spin()


def sub_callback(data):
    """"""
    if data.data == "grab":
        grab_routine()
    elif data.data == "pass":
        pass_routine()


def grab_routine():
    """pickup object off table"""
    move_arm(arm_states["side_rest"])
    sleep(8)

    move_arm(arm_states["half_reach"])
    sleep(7)

    move_arm(arm_states["full_reach"])
    sleep(4)

    move_arm(arm_states["arm2chest"])
    sleep(8)


def pass_routine():
    """pass-off object"""
    move_arm(arm_states["full_reach"])
    sleep(6)

    move_arm(arm_states["side_rest"])
    sleep(8)


def move_arm(state):
    """change the state of the arm"""
    for i in range(len(arm_publishers)):
        arm_publishers[i].publish(state[i])


if __name__ == '__main__':
    main()

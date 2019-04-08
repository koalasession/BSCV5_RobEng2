#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def move():

    # initialize a node
    rospy.init_node('move_twist', anonymous=False)

    # creates a publisher
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    
    linear_velocity = 0.2
    angular_velocity = 0.2

    # creates a Twist message with linear and angular values.
    msg = Twist()
    msg.linear.x = linear_velocity
    msg.angular.z = angular_velocity

    # save current time and publish rate at 10 Hz.
    start = rospy.Time.now()
    rate = rospy.Rate(10)

    # publish for 5 seconds.
    while rospy.Time.now() < start + rospy.Duration.from_sec(5):
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass

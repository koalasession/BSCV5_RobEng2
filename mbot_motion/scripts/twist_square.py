#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi


def twist_square():
    # Turning angle in degrees
    angle = 90
    # Turning speed degrees per second
    ang_vel = angle/5

    correction = 1.75

    # initialize a node
    rospy.init_node('move_twist', anonymous=False)

    # creates a publisher
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    # creates Twist messages.
    forward = Twist()
    forward.linear.x = 0.2
    forward.angular.z = 0.0

    turn = Twist()
    turn.linear.x = 0.0
    turn.angular.z = correction*ang_vel*2*pi/360

    for i in range(4):
        publish_msg(pub, forward)
        publish_msg(pub, turn)


def publish_msg(pub, msg):
    # save current time and publish rate at 10 Hz
    start = rospy.Time.now()
    rate = rospy.Rate(10)

    # publish for 5 seconds.
    while rospy.Time.now() < start + rospy.Duration.from_sec(5):
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        twist_square()
    except rospy.ROSInterruptException:
        pass

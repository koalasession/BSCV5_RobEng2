#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from mbot_motion.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi


def move_odom():

    # initialize a node
    rospy.init_node('nav_square', anonymous=False)

    # creates a publisher
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    # save current time and publish rate at 10 Hz.
    rate = rospy.Rate(10)

    goal_distance = 1.0
    goal_angle = radians(90)
    angular_velocity = 0.7
    linear_velocity = 0.2

    # initialize the tf listener
    tf_listener = tf.TransformListener()
    # buffer
    rospy.sleep(2)

    # set the odometry frame
    odometry_frame = '/odom'

    # find out if the robot uses /base_link or /base_footprint
    try:
        tf_listener.waitForTransform(
            odometry_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
        base_frame = '/base_footprint'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        try:
            tf_listener.waitForTransform(
                odometry_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
            base_frame = '/base_link'
        except(tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo(
                "Cannot find transform between /odom and /base_link or /base_footprint")
            rospy.signal_shutdown("tf Exception")

    # Initialize the position variable as a Point type
    position = Point()

    # Loop for 4 side of the square.
    for i in range(4):
        # creates a Twist message with linear and angular values.
        msg = Twist()
        msg.linear.x = linear_velocity

        # Get the starting position values
        (position, rotation) = get_odom(tf_listener, odometry_frame, base_frame)

        x_start = position.x
        y_start = position.y

        # Keep track of the distance traveled
        distance = 0

        # publish for 5 seconds.
        while distance < goal_distance:
            pub.publish(msg)
            rate.sleep()
            # Get the current position
            (position, rotation) = get_odom(
                tf_listener, odometry_frame, base_frame)

            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - x_start), 2) +
                            pow((position.y - y_start), 2))

        # Stopping the robot.
        msg = Twist()
        pub.publish(msg)
        rospy.sleep(1)

        # Set the movement command to a rotation
        msg.angular.z = angular_velocity

        # Track the last angle measured
        last_angle = rotation

        # Track how far the turtlebot has turned
        turn_angle = 0

        # publish for 5 seconds.
        while abs(turn_angle) < abs(goal_angle):
            pub.publish(msg)
            rate.sleep()
            print(turn_angle)
            # Get the current position
            (position, rotation) = get_odom(
                tf_listener, odometry_frame, base_frame)

            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(rotation - last_angle)

            # Add to the running total
            turn_angle += delta_angle
            print(delta_angle)
            last_angle = rotation

        # Stopping the robot.
        msg = Twist()
        pub.publish(msg)
        rospy.sleep(1)


def get_odom(tf_listener, odometry_frame, base_frame):
    # Get the current transform between the odom adn base frames
    try:
        (transformation, rotation) = tf_listener.lookupTransform(
            odometry_frame, base_frame, rospy.Time(0))
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*transformation), quat_to_angle(Quaternion(*rotation)))


if __name__ == '__main__':
    try:
        move_odom()
    except rospy.ROSInterruptException:
        pass

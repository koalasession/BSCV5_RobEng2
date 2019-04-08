import roslib
import rospy
import math
import random
import tf
from tf.transformations import euler_from_quaternion
# A set of message filters which take in messages and may output those messages at a later time, based on the conditions that filter needs met.
import message_filters
# The odometry message
from nav_msgs.msg import Odometry
# the velocity command message
from geometry_msgs.msg import Twist

# instantiate global variables "globalOdom"
globalOdom = Odometry()
goalX = None
goalY = None
success_percent = 0

arr_goalX = [0, -5, 0]
arr_goalY = [5, 0, -5]

# method to control the robot


def callback(odom):
    # the odometry parameter should be global
    global globalOdom
    globalOdom = odom
    at_goal = False

    global goalX, goalY, success_percent, arr_goalX, arr_goalY

    # make a new twist message
    command = Twist()

    # log the position to a file
    X_file = open('X_data.txt', 'a')
    Y_file = open('Y_data.txt', 'a')

    for index in range(1):
        # Fill in the fields.  Field values are unspecified
        # until they are actually assigned. The Twist message
        # holds linear and angular velocities.
        print "Success Percent = ", success_percent
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        # get goal x and y locations from the launch file
        # get parameter 'lab2/goalX' with a default value of 0
        # find current (x,y) position of robot based on odometry
        currentX = globalOdom.pose.pose.position.x
        currentY = globalOdom.pose.pose.position.y
        print type(currentX)
        if goalX is None:
            goalX = currentX + 5
        if goalY is None:
            goalY = currentY
        print "GoalX = ", goalX, "\tGoalY = ", goalY

        if abs(currentX - goalX) < 0.1 and abs(currentY - goalY) < 0.1:
            print "At goal!!!!"
            if success_percent == 3:
                rospy.signal_shutdown("Goal Reached.")
            else:
                goalX = goalX + arr_goalX[success_percent]
                goalY = goalY + arr_goalY[success_percent]
                success_percent = success_percent + 1

        # find current orientation of robot based on odometry (quaternion coordinates)
        xOr = globalOdom.pose.pose.orientation.x
        yOr = globalOdom.pose.pose.orientation.y
        zOr = globalOdom.pose.pose.orientation.z
        wOr = globalOdom.pose.pose.orientation.w

        # find orientation of robot (Euler coordinates)
        (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])

        # find currentAngle of robot (equivalent to yaw)
        # now that you have yaw, the robot's pose is completely defined by (currentX, currentY, currentAngle)
        currentAngle = yaw

        d_x = goalX - currentX
        d_y = goalY - currentY

        # calculate bearing difference with bot and goal
        bearing_diff = math.atan2(d_y, d_x)
        print "d_x = ", d_x
        print "d_y = ", d_y
        print "\n\nbearing diff: ", bearing_diff*180/math.pi

        # the code below (currently commented) shows how
        # you can print variables to the terminal (may
        # be useful for debugging)
        print 'x: {0}'.format(currentX)
        print 'y: {0}'.format(currentY)
        print yaw*180/math.pi

        # Line the bot up with the goal
        lined_up = False
        bearing = bearing_diff - currentAngle
        print "bearing ", bearing*180/math.pi

        # log the current position to a file
        X_file.write('%.4f' % currentX)
        X_file.write('\n')

        Y_file.write('%.4f' % currentY)
        Y_file.write('\n')

        if abs(bearing) < 0.05:
            lined_up = True

        if lined_up:
            # compute distance to goal position
            distToGoal = math.sqrt(math.pow(d_y, 2)+math.pow(d_x, 2))
            vel = 10

            if distToGoal < 5.0:  # change this based on need
                vel = vel*distToGoal/5.0

            if distToGoal < 0.01:  # change this based on need
                # Stop if you are within 1 unit of goal
                vel = 0.0
                print 'Arrived at goal!'
            # commanded velocities

            command.linear.x = 2.5 * (vel)
            print "Heading straight"

        elif not lined_up:
            command.angular.z = bearing/5
            print "Lining up heading"

        pub.publish(command)

    # other technique

    # command.angular.z = 1.0 * (bearing )
    # pub.publish(command)

    X_file.close()
    Y_file.close()

    # main function call


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('square_odom', log_level=rospy.DEBUG)

    # subscribe to odometry message
    sub2 = message_filters.Subscriber('odom', Odometry)

    # synchronize laser scan and odometry data
    ts = message_filters.TimeSynchronizer([sub2], 10)
    ts.registerCallback(callback)

    # publish twist message
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    # Turn control over to ROS
    rospy.spin()

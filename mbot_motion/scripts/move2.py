#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from math import pi


class Move():
    def __init__(self):
        # initialize the node.
        rospy.init_node('straight', anonymous=False)
        # initialize shutdown function
        rospy.on_shutdown(self.shutdown)
        # Twist messages publisher to the /cmd_vel topic
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

#    def callback(self,data):
        # display the data axes values for every received message
#        rospy.loginfo("axes values = " + '{}'.format(data.axes))
        # create a Twist message
#        msg = Twist()
        # set the axes values (1 and 0)(linear and angular velocities)
#        msg.linear.x = data.axes[1]
#        msg.angular.z = data.axes[0]
#        self.cmd_vel.publish(msg)

    def shutdown(self):
        # Always stop the robot when shutting down the node
        # display "Stopping the node.."
        rospy.loginfo("Stopping the straight node.")
        # publish and empty Twist message to stop the robot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        Move()
    except KeyboardInterrupt:
        break
        


//move forward
ros: : Time start = ros: : Time: : now()
while(ros:: Time: : now() - start < ros: : Duration(5.0))
{
    geometry_msgs: : Twist move
    // velocity controls
    move.linear.x = 0.1
    // speed value m/s
    move.angular.z = 0
    movement_pub.publish(move)

    ros: : spinOnce()
    rate.sleep()
}
//turn right
ros: : Time start_turn = ros: : Time: : now()
while(ros:: Time: : now() - start_turn < ros: : Duration(4.0))
{
    geometry_msgs: : Twist move
    // velocity controls
    move.linear.x = 0
    // speed value m/s
    move.angular.z = -2.25
    movement_pub.publish(move)

    ros: : spinOnce()
    rate.sleep()
}
//move forward again
ros: : Time start2 = ros: : Time: : now()
while(ros:: Time: : now() - start2 < ros: : Duration(5.0))
{
    geometry_msgs: : Twist move
    // velocity controls
    move.linear.x = 0.1
    // speed value m/s
    move.angular.z = 0
    movement_pub.publish(move)

    ros: : spinOnce()
    rate.sleep()
}

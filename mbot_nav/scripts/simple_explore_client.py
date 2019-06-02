#!/usr/bin/env python
from sys import argv

import rospy
import actionlib
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskGoal
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, Twist


class SimpleExplorationClient:
    """ A simple client class that submits a goal to the frontier_exploration node"""

    def __init__(self, args):
        # receive polygon dimenstion
        if len(args) == 1:
            max_x = int(argv[1])
            max_y = int(argv[1])
        elif len(argv) == 2:
            max_x = int(argv[1])
            max_y = int(argv[2])
        else:
            max_x = 5
            max_y = 5

        # Initialize node
        rospy.init_node('simple_explore', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Set Twist msg publisher
        self.pub = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)
        # Set polygon points
        self.points_x = [max_x, max_x, -max_x, -max_x, max_x+0.1]
        self.points_y = [max_y, -max_y, -max_y, max_y, max_y+0.1]

    def init_move(self):
        """ initial rotaion to explore initial surroundings """
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.7

        self.publish_msg(move_cmd)
        rospy.loginfo("move for init")

    def publish_msg(self, msg):
        # save current time and publish rate at 10 Hz
        start = rospy.Time.now()
        rate = rospy.Rate(10)

        # publish for 5 seconds.
        while rospy.Time.now() < start + rospy.Duration.from_sec(10):
            self.pub.publish(msg)
            rate.sleep()
        self.pub.publish(Twist())

    def request_exploration(self):
        # exploration client
        client = actionlib.SimpleActionClient(
            'explore_server',
            ExploreTaskAction)
        client.wait_for_server()

        # empty polygon
        polygonStamped = PolygonStamped()
        polygonStamped.header.frame_id = 'map'
        polygonStamped.header.stamp = rospy.Time.now()
        # starting point from turtlebot position
        initialGoal = PointStamped()
        initialGoal.header.frame_id = 'map'
        initialGoal.point = Point32(x=0, y=0, z=0)

        # unbounded exploration, uncomment for bounded
        # for x, y in zip(self.points_x, self.points_y):
        # polygonStamped.polygon.points.append(Point32(x=x, y=y, z=0))

        # setting exploration goal
        exploration_goal = ExploreTaskGoal()
        exploration_goal.explore_boundary = polygonStamped
        exploration_goal.explore_center = initialGoal

        # starting exploration
        client.send_goal(exploration_goal)

        rospy.loginfo("Exploration Started")
        client.wait_for_result()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Stop the robot
        self.pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    simpleExplorationClient = SimpleExplorationClient(argv[1:])
    simpleExplorationClient.init_move()
    simpleExplorationClient.request_exploration()

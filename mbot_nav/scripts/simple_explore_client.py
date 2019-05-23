#!/usr/bin/env python
# https://github.com/borisoft/circlerecognize/blob/2d87a9c5184c0d241a1e44508cc6e65ee30b2268/src/fist_robotics_labs/src/makemap.py?fbclid=IwAR1i6LeQ14mU4yKE0MUALadHptBZOYIryOcbWqiVO4x5oc8-gm6CeW1O2nY
# for inclusion in launch file
# https://github.com/katie0809/HaniumProject2017/blob/ef9868e5a7ef5e47d30d06d609e134ef0cc49c54/src/frontier_explore_b.py?fbclid=IwAR2fzcP0kRivAkOWpfPFGe_Wf_KqjxD_tz5rQ1zCugZZO6VujIplNZznArA
from sys import argv

import rospy
from frontier_exploration.msg import ExploreTaskAction
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
import actionlib

class SimpleExplorationClient:
    """ A simple client class that submits a goal to the frontier_exploration node"""

    def __init__(self, args):
        if len(args) == 1:
            max_x = int(argv[1])
            max_y = int(argv[1])
        elif len(argv) == 2:
            max_x = int(argv[1])
            max_y = int(argv[2])
        else:
            max_x = 5
            max_y = 5

        rospy.init_node('simple_explore', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.points_x = [max_x, max_x, -max_x, -max_x, max_x+0.1]
        self.points_y = [max_y, -max_y, -max_y, max_y, max_y+0.1]

    def init_move(self):
        move_cmd = Twist()
        move_cmd.linear.x = 1
        move_cmd.linear.y = 0
        move_cmd.angular.z = 2

        self.cmd_vel.publish(move_cmd)
        rospy.loginfo("move for init")

    def request_exploration(self):
        client = actionlib.SimpleActionClient(
            'explore_server',
            ExploreTaskAction)
        client.wait_for_server()

        #  exploration_goal.explore_boundary.header.frame_id = 'map'
        polygonStamped = PolygonStamped()
        polygonStamped.header.frame_id = 'map'
        polygonStamped.header.stamp = rospy.Time.now()
        point = Point32()
        initialGoal = PointStamped()
        initialGoal.header.frame_id = 'map'
        initialGoal.point = Point32(x=0, y=0, z=0)

        for x,y in zip(self.points_x, self.points_y):
            polygonStamped.polygon.points.append(Point32(x=x, y=y, z=0))

        exploration_goal = ExploreTaskGoal()
        # unbounded exploration, uncomment for bounded
        # exploration_goal.explore_boundary = polygonStamped
        exploration_goal.explore_center = initialGoal

        client.send_goal(exploration_goal)

        rospy.loginfo("Exploration Started")
        client.wait_for_result()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Stop the robot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('exploration_simple')

    simpleExplorationClient = SimpleExplorationClient(argv[1:])
    simpleExplorationClient.init_move()
    simpleExplorationClient.request_exploration()

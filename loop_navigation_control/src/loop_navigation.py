#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from time import sleep
from std_msgs.msg import Bool


class MoveNode:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.is_charging_flag = False
        self.counter = 0

    def go_to(self, x, y, z):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if wait:
            sleep(2)

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

        return self.client.get_result()

    def home_position(self, loc):
        if loc == 0:
            rospy.loginfo("Heading to First Point")
            self.go_to(0.3327469671548591, -0.07669310109807612, -0.34303521623827754)
        elif loc == 1:
            rospy.loginfo("Heading to Second Point")
            self.go_to(0.0745602489172651, -2.7893164804901556, -2.65709415824142)
        elif loc == 2:
            rospy.loginfo("Heading to Third Point")
            self.go_to(2.57994547476032, -1.925609675033026, -1.5745951272044802)
        elif loc == 3:
            rospy.loginfo("Heading to Fourth Point")
            self.go_to(4.3941970305451905, -0.23586030441848677, -0.10364319482621749)

    def cancel_goal(self):
        self.client.cancel_goal()
        rospy.loginfo("Goal cancelled")
    
    def docking_station_navigation(self):
        self.cancel_goal()
        pass

    def loop_navigation(self):
        if self.counter == 0:
            self.home_position(0)
            self.counter = 1
        elif self.counter == 1:
            self.home_position(1)
            self.counter = 2
        elif self.counter == 2:
            self.home_position(2)
            self.counter = 3
        elif self.counter == 3:
            self.home_position(3)
            self.counter = 0

    def flag_check(self, msg):
        self.is_charging_flag = msg.data

    def run(self):
        rospy.init_node('navigation_logical_node')
        rospy.Subscriber('battery_low_power_status', Bool, self.flag_check)
        while not rospy.is_shutdown():
            while not self.is_charging_flag:
                self.loop_navigation()
            while self.is_charging_flag:
                self.docking_station_navigation()


if __name__ == '__main__':
    rospy.init_node('navigation_logical_node')  
    move_node = MoveNode()
    move_node.run()

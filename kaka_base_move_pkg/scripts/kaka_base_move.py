#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import std_msgs
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class KakaBaseMove:
    def __init__(self):
        self.__robot_control_sub = rospy.Subscriber("robot_control",std_msgs.msg.String,self.RobotControlCallBack,queue_size=50)
        self.__base_move_result_pub = rospy.Publisher("base_move_result", std_msgs.msg.Int32, queue_size=50)
        self.__move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        rospy.loginfo("KAKA_BASE_MOVE waiting for move_base action server...")
        #等待60秒
        self.__move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("KAKA_BASE_MOVE connected to move base server,wait for command")

        # 设定目标位姿
        self.__targets = []

        target_angle = Quaternion(0.000,0.000,-0.703,0.712)
        target_position = Point(9.397, -3.910, 0.0)
        target_pose = Pose(target_position,target_angle)

        self.__targets.append(target_pose)


    def Run(self):
        rospy.loginfo("KAKA_BASE_MOVE running!")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()


    def RobotControlCallBack(self,msg):
        if msg.data == "start_move":
            rospy.loginfo("KAKA_BASE_MOVE start move!")
            if len(self.__targets) != 0:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = self.__targets[0]
                rospy.loginfo(goal)
                self.__targets.pop(0)
                self.move(goal)
            else:
                rospy.loginfo("KAKA_BASE_MOVE no targets!")
        elif msg.data == "stop_move":
            rospy.loginfo("KAKA_BASE_MOVE stop move!")


    def move(self,goal):
        self.__move_base.send_goal(goal)
        #给定一分钟时间到达目标点
        finished_within_time = self.__move_base.wait_for_server(rospy.Duration(60))
        if not finished_within_time:
            self.__move_base.cancel_goal()
            rospy.loginfo("KAKA_BASE_MOVE time out achieving goal")
            self.__base_move_result_pub.publish(0)
        else:
            state = self.__move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("KAKA_BASE_MOVE goal succeeded!")
                self.__base_move_result_pub.publish(1)


if __name__ == '__main__':
    rospy.init_node('kaka_base_move', anonymous=True)
    kaka_base_move = KakaBaseMove()
    kaka_base_move.Run()


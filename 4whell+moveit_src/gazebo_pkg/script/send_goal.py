#!/usr/bin/env python
#-*- coding:utf-8   -*-
 
 
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
 

def goal_pose():
    goal_pose=MoveBaseGoal()
    goal_pose.target_pose.header.frame_id="map"
    goal_pose.target_pose.pose.position.x=-1.55
    goal_pose.target_pose.pose.position.y=-6.8
    goal_pose.target_pose.pose.orientation.w=1
    return goal_pose
 
 
if __name__ == "__main__": 
    #节点初始化
    rospy.init_node('send_goal')
 
    #创建MoveBaseAction client
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #等待MoveBaseAction server启动
    client.wait_for_server()
 
    while not rospy.is_shutdown():
	goal=goal_pose()
	print("Sending goal")
        client.send_goal(goal)
	client.wait_for_result()
	

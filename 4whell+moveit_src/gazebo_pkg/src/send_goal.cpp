#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
using namespace std;
float B_position_x;
float B_position_y;
float B_orientation_z;
float B_orientation_w;
float C_position_x;
float C_position_y;
float C_orientation_z;
float C_orientation_w;
float D1_position_x;
float D1_position_y;
float D1_orientation_z;
float D1_orientation_w;
float D2_position_x;
float D2_position_y;
float D2_orientation_z;
float D2_orientation_w;
float D3_position_x;
float D3_position_y;
float D3_orientation_z;
float D3_orientation_w;
bool Choose_D1;
bool Choose_D2;
bool Choose_D3;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient ac("move_base", true);
move_base_msgs::MoveBaseGoal goal_B;
move_base_msgs::MoveBaseGoal goal_C;
move_base_msgs::MoveBaseGoal goal_D1;
move_base_msgs::MoveBaseGoal goal_D2;
move_base_msgs::MoveBaseGoal goal_D3;
void init_pose()
{
  goal_B.target_pose.header.frame_id = "map";
  goal_B.target_pose.header.stamp = ros::Time::now();
  goal_C.target_pose.header.frame_id = "map";
  goal_C.target_pose.header.stamp = ros::Time::now();
  goal_D1.target_pose.header.frame_id = "map";
  goal_D1.target_pose.header.stamp = ros::Time::now();
  goal_D2.target_pose.header.frame_id = "map";
  goal_D2.target_pose.header.stamp = ros::Time::now();
  goal_D3.target_pose.header.frame_id = "map";
  goal_D3.target_pose.header.stamp = ros::Time::now();
  //Goal B
  goal_B.target_pose.pose.position.x = B_position_x; 
  goal_B.target_pose.pose.position.y = B_position_y;  
  goal_B.target_pose.pose.position.z = B_orientation_z;   
  goal_B.target_pose.pose.orientation.w = B_orientation_w; 
  //Goal C
  goal_C.target_pose.pose.position.x = C_position_x; 
  goal_C.target_pose.pose.position.y = C_position_y;  
  goal_C.target_pose.pose.position.z = C_orientation_z;   
  goal_C.target_pose.pose.orientation.w = C_orientation_w; 

  //Goal D1
  goal_D1.target_pose.pose.position.x = D1_position_x; 
  goal_D1.target_pose.pose.position.y = D1_position_y;  
  goal_D1.target_pose.pose.position.z = D1_orientation_z;   
  goal_D1.target_pose.pose.orientation.w = D1_orientation_w; 

  //Goal D2
  goal_D2.target_pose.pose.position.x = D2_position_x; 
  goal_D2.target_pose.pose.position.y = D2_position_y;  
  goal_D2.target_pose.pose.position.z = D2_orientation_z;   
  goal_D2.target_pose.pose.orientation.w = D2_orientation_w; 

  //Goal D3
  goal_D3.target_pose.pose.position.x = D3_position_x; 
  goal_D3.target_pose.pose.position.y = D3_position_y;  
  goal_D3.target_pose.pose.position.z = D3_orientation_z;   
  goal_D3.target_pose.pose.orientation.w = D3_orientation_w; 

}
void voice_words_callback(const std_msgs::String& msg)
{
	/***指令***/
	string str1 = msg.data.c_str();    //取传入数据
	string str2 = "小车去B区";

        if(str1 == str2)
	{
		cout<<"好的：小车自主导航至B区"<<endl;
		ac.sendGoal(goal_B);
  		ac.waitForResult();
  		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
    			ROS_INFO("小车到达B区!");
			//识别二维码
                        //语音播报
 			//发送目标点，到达c区
			//拍照，检测，将检测结果pub
		}
  		else
    			ROS_INFO("小车未到达B区！");
		if(Choose_D1)
		{
			cout<<"小车自主导航至D1"<<endl;
			ac.sendGoal(goal_D1);
  			ac.waitForResult();
  			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
    				ROS_INFO("小车到达终点D1!");
				//语音播报
			}
  			else
    				ROS_INFO("小车未到达终点D1！");
		}	
		if(Choose_D2)
		{
			cout<<"小车自主导航至D2"<<endl;
			ac.sendGoal(goal_D2);
  			ac.waitForResult();
  			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
    				ROS_INFO("小车到达终点D2!");
			}
  			else
    				ROS_INFO("小车未到达终点D2！");
		}	
 		if(Choose_D3)
		{
			cout<<"小车自主导航至D3"<<endl;
			ac.sendGoal(goal_D3);
  			ac.waitForResult();
  			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
    				ROS_INFO("小车到达终点D3!");
			}
  			else
    				ROS_INFO("小车未到达终点D3！");
		}	
	}
	

int main(int argc, char** argv){
  ros::init(argc, argv, "send_goal");
  ros::NodeHandle n;    //创建句柄
  
  /***创建离线命令词识别结果话题订阅者***/
  ros::Subscriber voice_words_sub = n.subscribe("voice_words",10,voice_words_callback);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  init_pose();
  n.param<float>("/B_position_x", B_position_x, 1);
  n.param<float>("/B_position_y", B_position_y, 0);
  n.param<float>("/B_orientation_z", B_orientation_z, 0);
  n.param<float>("/B_orientation_w", B_orientation_w, 1);
  n.param<float>("/C_position_x", C_position_x, 1);
  n.param<float>("/C_position_y", C_position_y, 0);
  n.param<float>("/C_orientation_z", C_orientation_z, 0);
  n.param<float>("/C_orientation_w", C_orientation_w, 1);
  n.param<float>("/D1_position_x", D1_position_x, 1);
  n.param<float>("/D1_position_y", D1_position_y, 0);
  n.param<float>("/D1_orientation_z", D1_orientation_z, 0);
  n.param<float>("/D1_orientation_w", D1_orientation_w, 1);
  n.param<float>("/D2_position_x", D2_position_x, 1);
  n.param<float>("/D2_position_y", D2_position_y, 0);
  n.param<float>("/D2_orientation_z", D2_orientation_z, 0);
  n.param<float>("/D2_orientation_w", D2_orientation_w, 1);
  n.param<float>("/D3_position_x", D3_position_x, 1);
  n.param<float>("/D3_position_y", D3_position_y, 0);
  n.param<float>("/D3_orientation_z", D3_orientation_z, 0);
  n.param<float>("/D3_orientation_w", D3_orientation_w, 1);
  
  n.param<bool>("/Choose_D1", Choose_D1, true);
  n.param<bool>("/Choose_D2", Choose_D2, false);
  n.param<bool>("/Choose_D3", Choose_D3, false);


  ros::spin();
  return 0;
}

        

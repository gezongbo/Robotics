#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
 
#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>
 
#include<iostream>
using namespace std;
 
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "jaka_moveit");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
  const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // ros::Publisher display_publisher  = node_handle.advertise<moveit_msgs::DisplayTrajectory> ("/move_group/display_planned_path", 1, true);
//  moveit_msgs::DisplayTrajectory display_trajectory;
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  
  // 设置初始关节角度
  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = -1.3705;
  joint_group_positions[2] = 2.6543;
  joint_group_positions[3] = -1.3011;
  joint_group_positions[4] = 0.0;
  joint_group_positions[5] = 0.0;  

  group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "initialization completed!" : "FUCKING FALLED");
  if(success)
    group.execute(my_plan);
  else
    cout << "falled"<<endl;
  sleep(5.0);
  ros::shutdown();
}
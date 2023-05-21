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
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // ros::Publisher display_publisher  = node_handle.advertise<moveit_msgs::DisplayTrajectory> ("/move_group/display_planned_path", 1, true);
//  moveit_msgs::DisplayTrajectory display_trajectory;
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  geometry_msgs::Pose target_pose1;


 target_pose1.orientation.x= 0;
  target_pose1.orientation.y = 0;
  target_pose1.orientation.z = 0;
  target_pose1.orientation.w = 1;

  target_pose1.position.x = 1.0;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.0;

  group.setPoseTarget(target_pose1);

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
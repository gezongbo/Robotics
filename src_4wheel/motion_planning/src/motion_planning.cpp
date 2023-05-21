#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "motion_planning/SetGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>
#include <math.h>
#define PI 3.1415926535

double goal_x, goal_y, goal_z;
tf2::Quaternion nav_qtn, arm_qtn;
bool set_goal = false;
bool nav_done = false;
double desire_x = 1.2;

class multiThread
{
public:
    multiThread()
    {
        sub = nm.subscribe("/move_base/result",10,&multiThread::cmdvelCallback,this);
    }

    // 获取导航结果
    void cmdvelCallback(const move_base_msgs::MoveBaseActionResult& msg)
    {
        if(msg.status.text=="Goal reached."){
            ROS_INFO("reached");
            nav_done = true;
        }else{
            ROS_INFO("nav failed");
        }
        
    }

private:
	ros::NodeHandle nm;
	ros::Subscriber sub;
};

bool doReq(motion_planning::SetGoal::Request& request, motion_planning::SetGoal::Response& response){

    goal_x = request.goal_x - desire_x * cos(request.rot_Y);
    goal_y = request.goal_y - desire_x * sin(request.rot_Y);
    goal_z = request.goal_z;
    nav_qtn.setRPY(0,0,request.rot_Y);
    arm_qtn.setRPY(request.rot_R,request.rot_P,0);
    set_goal = true;
    response.success = true;
    return true;
}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 1.初始化 ROS 节点
    ros::init(argc,argv,"set_goal");

    // 2.获取导航完成信息
    multiThread recOb;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(30000);

    // 3.创建发布者对象
    ros::NodeHandle nh;
    ros::Publisher nav_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    ros::ServiceServer server = nh.advertiseService("SetGoal",doReq);
    // ROS_INFO("服务已经启动....");

    while(ros::ok())
    {
        // 设置导航目标点并开始导航
        if(set_goal)
        {
            geometry_msgs::PoseStamped nav_goal;
            ros::Time current_time = ros::Time::now();

            nav_goal.header.frame_id = "map";
            nav_goal.header.stamp = current_time;
            nav_goal.pose.position.x = goal_x;
            nav_goal.pose.position.y = goal_y;
            nav_goal.pose.orientation.w = 1;
            nav_goal.pose.orientation.x = nav_qtn.getX();
            nav_goal.pose.orientation.y = nav_qtn.getY();
            nav_goal.pose.orientation.z = nav_qtn.getZ();
            nav_goal.pose.orientation.w = nav_qtn.getW();
            nav_pub.publish(nav_goal);
            ROS_INFO("set_goal!!!");
            set_goal = false;
        }
        if(nav_done)
        {
            static const std::string PLANNING_GROUP = "arm";
            moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
            geometry_msgs::Pose target_pose1;

            target_pose1.orientation.x = arm_qtn.getX();
            target_pose1.orientation.y = arm_qtn.getY();
            target_pose1.orientation.z = arm_qtn.getZ();
            target_pose1.orientation.w = arm_qtn.getW();
            target_pose1.position.x = desire_x;
            target_pose1.position.y = 0.0;
            target_pose1.position.z = goal_z;
            group.setPoseTarget(target_pose1);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
            ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "initialization completed!" : "FALLED");
            if(success)
                group.execute(my_plan);
            sleep(5.0);
            // ros::shutdown();

            ROS_INFO("nav_done!!!");
            nav_done = false;
        }
        ros::spinOnce();
        usleep(1000);
    }
    return 0;
}

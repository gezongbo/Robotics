#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf2_ros/transform_broadcaster.h"
// #include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#define PI 3.1415926535

double pos_x, pos_y, vel_x, vel_y, ang_z;
geometry_msgs::Quaternion qtn;

class multiThread
{
public:
    multiThread()
    {
        // sub = nm.subscribe("/cmd_vel",10,&multiThread::cmdvelCallback,this);
        sub = nm.subscribe("/gazebo/model_states",50,&multiThread::pubOdom,this);
    }

    void pubOdom(const gazebo_msgs::ModelStates& g_msg)
    {
        // int end_num = sizeof(g_msg.pose);
        // ROS_INFO("num: %d", end_num);
        pos_x = g_msg.pose[2].position.x;
        pos_y = g_msg.pose[2].position.y;
        qtn = g_msg.pose[2].orientation;
        vel_x = g_msg.twist[2].linear.x;
        vel_y = g_msg.twist[2].linear.y;
        ang_z = g_msg.twist[2].angular.z;

        
    }

private:
	ros::NodeHandle nm;
	ros::Subscriber sub;
};


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 1.初始化 ROS 节点
    ros::init(argc,argv,"odometry_publisher");

    // 2.获取odom数据
    multiThread recOb;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(30000);


    // 3.创建发布者对象
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf2_ros::TransformBroadcaster odom_broadcaster;

    //4-2.设置发送频率
    ros::Time current_time;
    ros::Rate rate(50);

    //4-3.循环发送
    while (ros::ok())
    {
        current_time = ros::Time::now();

        // publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = pos_x;
        odom_trans.transform.translation.y = pos_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = qtn;
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        // publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
    
        //set the position
        odom.pose.pose.position.x = pos_x;
        odom.pose.pose.position.y = pos_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = qtn;
    
        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = vel_x;
        odom.twist.twist.linear.y = vel_y;
        odom.twist.twist.angular.z = ang_z;
    
        //publish the message
        odom_pub.publish(odom);

        ros::spinOnce();
        usleep(1000);
    }


    return 0;
}

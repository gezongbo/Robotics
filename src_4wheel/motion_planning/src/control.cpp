#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#define PI 3.1415926535

double linx, angz, angleZ;
std_msgs::Float64 actual_vel;
double x = 0.0;
double y = 0.0;
double th = 0.0;

class multiThread
{
public:
    multiThread()
    {
        sub[0] = nm.subscribe("/cmd_vel",10,&multiThread::cmdvelCallback,this);
        sub[1] = nm.subscribe("/gazebo/model_states",10,&multiThread::poseCallback,this);
    }

    // 获取速度；发布里程计消息
    void cmdvelCallback(const geometry_msgs::Twist& msg)
    {
        ROS_INFO("vel=%.2f,angular=%.2f",msg.linear.x,msg.angular.z);
        linx = msg.linear.x;
        angz = msg.angular.z;   

    }

    void poseCallback(const gazebo_msgs::ModelStates& g_msg)
    {
        // angleZ = acos(g_msg.pose[1].orientation.w) * 2;
        double vx = g_msg.twist[1].linear.x;
        double vy = g_msg.twist[1].linear.y;
        actual_vel.data = sqrt(vx * vx + vy * vy);
    }

private:
	ros::NodeHandle nm;
	ros::Subscriber sub[2];
};


std_msgs::Float64 wheel_vel[4];
std_msgs::Float64 angle[4];
// 组织消息,计算各轮速以及转向角度
void calc_joint(double linx,double angz)
{
    double r = sqrt(0.25 * 0.25 + 0.786 * 0.786);
    double alpha = atan2(0.25 , 0.786);
    // ROS_INFO("v=%.2f,ang=%.2f",linx,angz);
    // 左前
    wheel_vel[0].data = sqrt(pow(linx - angz * r * sin(alpha),2) + pow(angz * r * cos(alpha),2)) / 0.1;
    angle[0].data = atan2(angz * r * cos(alpha), linx - angz * r * sin(alpha));
    // 右前
    wheel_vel[1].data = sqrt(pow(linx + angz * r * sin(alpha),2) + pow(angz * r * cos(alpha),2)) / 0.1;
    angle[1].data = atan2(angz * r * cos(alpha), linx + angz * r * sin(alpha));
    // 左后
    wheel_vel[2].data = sqrt(pow(linx - angz * r * sin(alpha),2) + pow(angz * r * cos(alpha),2)) / 0.1;
    angle[2].data = -atan2(angz * r * cos(alpha), linx - angz * r * sin(alpha));
    // 右后
    wheel_vel[3].data = sqrt(pow(linx + angz * r * sin(alpha),2) + pow(angz * r * cos(alpha),2)) / 0.1;
    angle[3].data = -atan2(angz * r * cos(alpha), linx + angz * r * sin(alpha));

    int j = 0;
    for (j = 0; j < 4; j++){
        if(angle[j].data > PI/2){
            wheel_vel[j].data = -wheel_vel[j].data;
            angle[j].data = angle[j].data - PI;
        }
        else if(angle[j].data < -PI/2){
            wheel_vel[j].data = -wheel_vel[j].data;
            angle[j].data = angle[j].data + PI;
        }
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 1.初始化 ROS 节点
    ros::init(argc,argv,"control");

    // 2.获取cmd_vel
    multiThread recOb;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(30000);


    // 3.创建发布者对象
    ros::NodeHandle nh;
    ros::Publisher pub[8];
    pub[0] = nh.advertise<std_msgs::Float64>("/joint_lf_little_position_controller/command",1);
    pub[1] = nh.advertise<std_msgs::Float64>("/joint_rf_little_position_controller/command",1);
    pub[2] = nh.advertise<std_msgs::Float64>("/joint_bf_little_position_controller/command",1);
    pub[3] = nh.advertise<std_msgs::Float64>("/joint_br_little_position_controller/command",1);
    pub[4] = nh.advertise<std_msgs::Float64>("/joint_lf_foot_velocity_controller/command",1);
    pub[5] = nh.advertise<std_msgs::Float64>("/joint_rf_foot_velocity_controller/command",1);
    pub[6] = nh.advertise<std_msgs::Float64>("/joint_bf_foot_velocity_controller/command",1);
    pub[7] = nh.advertise<std_msgs::Float64>("/joint_br_foot_velocity_controller/command",1);

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("/actual_vel",1);

    // ros::Publisher map_pub = nh.advertise<nav_msgs::Odometry>("map", 50);
    // tf2_ros::TransformBroadcaster odom_broadcaster;



    // 4.循环发布运动控制消息


    //4-2.设置发送频率
    ros::Time current_time;
    ros::Rate rate(10);
    //4-3.循环发送
    while (ros::ok())
    {        
        // current_time = ros::Time::now();

        // // publish the transform over tf
        // geometry_msgs::TransformStamped odom_trans;
        // odom_trans.header.stamp = current_time;
        // odom_trans.header.frame_id = "map";
        // odom_trans.child_frame_id = "odom";

        // odom_trans.transform.translation.x = 0.0;
        // odom_trans.transform.translation.y = 0.0;
        // odom_trans.transform.translation.z = 0.0;
        // odom_trans.transform.rotation.x = 0.0;
        // odom_trans.transform.rotation.y = 0.0;
        // odom_trans.transform.rotation.z = 0.0;
        // odom_trans.transform.rotation.w = 1.0;
        // //send the transform
        // odom_broadcaster.sendTransform(odom_trans);
        
        calc_joint(linx,angz);
        int i = 0;
        for (i = 0; i < 4; i++){
            pub[i].publish(angle[i]);
        }
        for (i = 0; i < 4; i++){
            pub[i+4].publish(wheel_vel[i]);
        }
        
        vel_pub.publish(actual_vel);

        ros::spinOnce();
        usleep(1000);
    }


    return 0;
}

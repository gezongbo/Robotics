#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include <math.h>
#include <iostream> 
#include <fstream>


#define PI 3.1415926535

double linx, angz, actual_angular, actual_x, actual_y, actual_angle;
std_msgs::Float64 actual_vel;

class multiThread
{
public:
    multiThread()
    {
        sub = nm.subscribe("/gazebo/model_states",10,&multiThread::poseCallback,this);
    }

    void cmdvelCallback(const geometry_msgs::Twist& msg)
    {
        ROS_INFO("vel=%.2f,angular=%.2f",msg.linear.x,msg.angular.z);
        linx = msg.linear.x;
        angz = msg.angular.z;   
    }

    void poseCallback(const gazebo_msgs::ModelStates& g_msg)
    {
        double vx = g_msg.twist[1].linear.x;
        double vy = g_msg.twist[1].linear.y;
        actual_vel.data = sqrt(vx * vx + vy * vy);
        actual_angular = g_msg.twist[1].angular.z;
        actual_x = g_msg.pose[1].position.x;
        actual_y = g_msg.pose[1].position.y;
        actual_angle = acos(g_msg.pose[1].orientation.w) * 2;
    }

private:
	ros::NodeHandle nm;
	ros::Subscriber sub;
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
    pub[0] = nh.advertise<std_msgs::Float64>("/myrobot/joint_lf_little_position_controller/command",1);
    pub[1] = nh.advertise<std_msgs::Float64>("/myrobot/joint_rf_little_position_controller/command",1);
    pub[2] = nh.advertise<std_msgs::Float64>("/myrobot/joint_bf_little_position_controller/command",1);
    pub[3] = nh.advertise<std_msgs::Float64>("/myrobot/joint_br_little_position_controller/command",1);
    pub[4] = nh.advertise<std_msgs::Float64>("/myrobot/joint_lf_foot_velocity_controller/command",1);
    pub[5] = nh.advertise<std_msgs::Float64>("/myrobot/joint_rf_foot_velocity_controller/command",1);
    pub[6] = nh.advertise<std_msgs::Float64>("/myrobot/joint_bf_foot_velocity_controller/command",1);
    pub[7] = nh.advertise<std_msgs::Float64>("/myrobot/joint_br_foot_velocity_controller/command",1);

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("/actual_vel",1);


    // 4.循环发布运动控制消息

    // 输出为csv格式文件
    std::ofstream file;//创建一个ofstream对象
	file.open("src/motion_planning/data/test_8.csv",std::ios::out);
    file<<"time"<<","<<"actual velocity"<<","<<"actual angular"<<","<<"actual x"<<","<<"actual y"<<","<<"actual theta"<<std::endl;


    //4-2.设置发送频率
    ros::Rate rate(10);
    //4-3.循环发送
    ros::Time start = ros::Time::now();
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);
    ros::Duration elapsed(0);
    bool done = false;
    while (!done )
    {
        now = ros::Time::now();
        if (actual_vel.data < 0.001){
            start = now;
        }
        dt = now - prev;
        prev = now;
        elapsed = now - start;

        linx = 1;
        angz = 1.2024127 * sin(elapsed.toSec()/2);

        if(elapsed.toSec() >= 4*PI){
            linx = 0;
            angz = 0;
            done = true;
        }

        calc_joint(linx,angz);
        int i = 0;
        for (i = 0; i < 4; i++){
            pub[i].publish(angle[i]);
        }
        for (i = 0; i < 4; i++){
            pub[i+4].publish(wheel_vel[i]);
        }
        
        vel_pub.publish(actual_vel);

        file<<elapsed.toSec()<<","<<actual_vel.data<<","<<actual_angular<<","<<actual_x<<","<<actual_y<<","<<actual_angle<<std::endl;
        ros::spinOnce();
        usleep(1000);
    }

    return 0;
}

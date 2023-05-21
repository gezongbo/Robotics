#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf2_ros/transform_listener.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <math.h>
#include <iostream> 
#include <fstream>


#define PI 3.1415926535

double linx, angz, actual_angular, actual_x, actual_y, actual_angle;
double actual_vel;
Eigen::Vector3d trans_pos;
Eigen::Quaterniond trans_qtn;

class multiThread
{
public:
    multiThread()
    {
        sub = nm.subscribe("/gazebo/model_states",10,&multiThread::poseCallback,this);
    }

    // void cmdvelCallback(const geometry_msgs::Twist& msg)
    // {
    //     ROS_INFO("vel=%.2f,angular=%.2f",msg.linear.x,msg.angular.z);
    //     linx = msg.linear.x;
    //     angz = msg.angular.z;   
    // }

    void poseCallback(const gazebo_msgs::ModelStates& g_msg)
    {
        double vx = g_msg.twist[2].linear.x;
        double vy = g_msg.twist[2].linear.y;
        actual_vel = sqrt(vx * vx + vy * vy);
        actual_angular = g_msg.twist[2].angular.z;
        trans_pos(0) = g_msg.pose[2].position.x;
        trans_pos(1) = g_msg.pose[2].position.y;
        trans_pos(2) = 0.0;
        trans_qtn.x() = g_msg.pose[2].orientation.x;
        trans_qtn.y() = g_msg.pose[2].orientation.y;
        trans_qtn.z() = g_msg.pose[2].orientation.z;
        trans_qtn.w() = g_msg.pose[2].orientation.w;
    }

private:
	ros::NodeHandle nm;
	ros::Subscriber sub;
};

// 定义速度、转向指令
double calc_angz(double t){
	return 0.2 * cos(t);
}
double set_vel = 0.5;

// 计算全局坐标系下速度
double calc_angle(double t){
	return 0.2 * sin(t);
}
double calc_velx(double t){
    return set_vel * cos(calc_angle(t));
}
double calc_vely(double t){
    return set_vel * sin(calc_angle(t));
}

// 数值积分计算当前位置
double calc_posx(double t, double dt){
    double x=0;
	int n = int(t/dt);//沿x轴将面积分为n个小矩形，每个矩形宽为dt
	double s = 0;
    double S = 0;			//s表示每个小矩形的面积，S表示总面积

    for(int i=1;i<=n;i++){
        s=calc_velx((x+x+dt)/2)*dt;
        S+=s;	//	将每个小矩形的面积叠加
        x+=dt;
    }
    return S;
}
double calc_posy(double t, double dt){
    double x=0;
	int n = int(t/dt);//沿x轴将面积分为n个小矩形，每个矩形宽为dt
	double s,S=0;			//s表示每个小矩形的面积，S表示总面积

    for(int i=1;i<=n;i++){
        s=calc_vely((x+x+dt)/2)*dt;
        S+=s;	//	将每个小矩形的面积叠加
        x+=dt;
    }
    return S;
}
// double calc_angle(double t, double dt){
//     double x=0;
// 	int n = int(t/dt);//沿x轴将面积分为n个小矩形，每个矩形宽为dt
// 	double s,S=0;			//s表示每个小矩形的面积，S表示总面积

//     for(int i=1;i<=n;i++){
//         s=calc_angz((x+x+dt)/2)*dt;
//         S+=s;	//	将每个小矩形的面积叠加
//         x+=dt;
//     }
//     return S;
// }

// 获取全局坐标系下位姿在本体坐标系下的表示
Eigen::Isometry3d calc_rel_T(Eigen::Isometry3d abs_T, Eigen::Isometry3d trans_T){
    Eigen::Isometry3d rel_T = Eigen::Isometry3d::Identity();
    rel_T.pretranslate(trans_T.rotation().transpose() * (abs_T.translation()-trans_T.translation()));
    rel_T.rotate(trans_T.rotation().transpose() * abs_T.rotation());
    return rel_T;
}

// 获取本体坐标系下位姿在全局坐标系下的表示
Eigen::Isometry3d calc_abs_T(Eigen::Isometry3d rel_T, Eigen::Isometry3d trans_T){
    Eigen::Isometry3d abs_T = Eigen::Isometry3d::Identity();
    abs_T.pretranslate(trans_T.rotation().transpose() * (rel_T.translation()-trans_T.translation()));
    abs_T.rotate(trans_T.rotation().transpose() * rel_T.rotation());
    return abs_T;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 1.初始化 ROS 节点
    ros::init(argc,argv,"test_move");
    std::string ref_traj;
    if(argv[1] != "line" ||argv[1] != "sine" )
    {
        ref_traj = argv[1];
    }else{
        ROS_ERROR("请输入正确的轨迹名！");
        return 1;
    }

    // 2.获取里程计消息
    multiThread recOb;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(30000);


    // 3.创建发布者对象
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    ros::Publisher trans_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/jaka_arm_controller/command",1);
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);

    // 初始化关节角度
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    // 获取abs_T,trans_T，并计算rel_T
    Eigen::Vector3d abs_pos;
    abs_pos(0) = 0.45;
    abs_pos(1) = -0.2;
    abs_pos(2) = 1.39;
    Eigen::AngleAxisd abs_rot(-PI/2, Eigen::Vector3d(0, 0, 1).normalized());//角轴旋转
    Eigen::Isometry3d abs_T = Eigen::Isometry3d::Identity();
    abs_T.pretranslate(abs_pos);
    abs_T.rotate(abs_rot);
    Eigen::Isometry3d trans_T = Eigen::Isometry3d::Identity();
    trans_T.pretranslate(trans_pos);
    trans_T.rotate(trans_qtn);
    Eigen::Isometry3d rel_T = calc_rel_T(abs_T,trans_T);
    // std::cout << abs_T.matrix() << std::endl;

    // 关节空间设置初始关节角度(直接笛卡尔空间会出问题？)
    joint_group_positions[0] = -1.5708;
    joint_group_positions[1] = 0.0;
    joint_group_positions[2] = 1.5708;
    joint_group_positions[3] = -1.5708;
    joint_group_positions[4] = 0.0;
    joint_group_positions[5] = 0.0;  
    group.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan0;
    moveit::planning_interface::MoveItErrorCode success0 = group.plan(my_plan0);
    group.execute(my_plan0);

    // 笛卡尔空间规划设置初始关节角度
    geometry_msgs::Pose target_pose1;
    Eigen::Vector3d end_eff_trans = rel_T.translation();
    Eigen::Quaterniond end_eff_qtn= Eigen::Quaterniond(rel_T.rotation());
    end_eff_qtn.normalize();
    target_pose1.position.x = end_eff_trans(0);
    target_pose1.position.y = end_eff_trans(1);
    target_pose1.position.z = end_eff_trans(2);
    target_pose1.orientation.x = end_eff_qtn.x();
    target_pose1.orientation.y = end_eff_qtn.y();
    target_pose1.orientation.z = end_eff_qtn.z();
    target_pose1.orientation.w = end_eff_qtn.w();
    group.setPoseTarget(target_pose1);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "initialization completed!" : "FUCKING FALLED");
    if(success)
        group.execute(my_plan);
    else
        std::cout << "falled" << std::endl;
    sleep(1.0);

    // 时间相关常数
    int t_num = 100;
    double T = 4 * PI;
    double dt_arm = T / t_num;
    double current_t;
    // 初始位姿
    Eigen::Vector3d trans_pos0 = trans_pos;
    Eigen::Matrix3d trans_rot0 = trans_qtn.normalized().toRotationMatrix();
    Eigen::Matrix3d current_trans_rot;
    Eigen::Vector3d calc_pos, current_trans_pos;

    // 生成世界坐标系下参考轨迹
    Eigen::MatrixXd abs_traj;
    abs_traj.resize(3,t_num);
    if(ref_traj == "line"){
        // 直线
        for(size_t i=0; i< t_num; i++){
            abs_traj(0,i) = abs_pos(0) + set_vel * i * dt_arm;
            abs_traj(1,i) = abs_pos(1);
            abs_traj(2,i) = abs_pos(2);
        }
    }else if(ref_traj == "sine"){
        // 正弦函数
        for(size_t i=0; i< t_num; i++){
            abs_traj(0,i) = abs_pos(0) + set_vel * i * dt_arm;
            abs_traj(1,i) = abs_pos(1);
            abs_traj(2,i) = abs_pos(2) + 0.2 * sin(4 * set_vel * i * dt_arm);
        }
    }


    // ******************** 循环内部！***********************
    // 关节轨迹初始化
    trajectory_msgs::JointTrajectory joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = "base_footprint";
    joint_state.joint_names.resize(6);
    joint_state.points.resize(1);
    joint_state.joint_names[0] ="joint1";
    joint_state.joint_names[1] ="joint2";
    joint_state.joint_names[2] ="joint3";
    joint_state.joint_names[3] ="joint4";
    joint_state.joint_names[4] ="joint5";
    joint_state.joint_names[5] ="joint6";
    size_t joint_num = 5;

    trajectory_msgs::JointTrajectoryPoint point0;
    point0.positions.resize(6);
    current_state = group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    for(size_t i=0;i<=joint_num;i++) {
      point0.positions[i] = joint_group_positions[i];
    }
    joint_state.points[0] = point0;
    joint_state.points[0].time_from_start = ros::Duration(0.0);


    for(int j=0; j<t_num; j++){
        current_t = (j+1) * dt_arm;
        // 获取abs_T,trans_T，并计算rel_T
        abs_pos = abs_traj.col(j);// abs_rot不变;
        Eigen::Isometry3d current_abs_T = Eigen::Isometry3d::Identity();
        current_abs_T.pretranslate(abs_pos);
        current_abs_T.rotate(abs_rot);

        calc_pos(0) = calc_posx(current_t,dt_arm);
        calc_pos(1) = calc_posy(current_t,dt_arm);
        calc_pos(2) = 0.0;
        current_trans_pos = trans_pos0 + trans_rot0 * calc_pos;
        //  std::cout << current_trans_pos << std::endl;
        current_trans_rot = Eigen::AngleAxisd(calc_angle(current_t), Eigen::Vector3d::UnitZ()) * trans_rot0;
        Eigen::Isometry3d current_trans_T = Eigen::Isometry3d::Identity();
        current_trans_T.pretranslate(current_trans_pos);
        current_trans_T.rotate(current_trans_rot);
        Eigen::Isometry3d current_rel_T = calc_rel_T(current_abs_T,current_trans_T);

        // 求逆解，规划关节轨迹点
        current_state = group.getCurrentState();  
        bool found_ik = current_state->setFromIK(joint_model_group, current_rel_T, 0.01);
        if (found_ik)
        {
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        group.setJointValueTarget(joint_group_positions);
        }
        else
        {
        ROS_INFO("Did not find IK solution");
        }

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(6);
        for(size_t i=0;i<=joint_num;i++) {
            point.positions[i] = joint_group_positions[i];
        }
        // std::cout << point << std::endl;
        joint_state.points.push_back(point);
        joint_state.points[j+1].time_from_start = ros::Duration(current_t);
    }
    // std::cout << joint_state.points[0].positions[0]<<joint_state.points[0].positions[1]<<joint_state.points[0].positions[2]<<joint_state.points[0].positions[3] << std::endl;
    // 计算耗时较长！需要重新指定时间戳
    joint_state.header.stamp = ros::Time::now();
    trans_pub.publish(joint_state);

    // 4.循环发布运动控制消息

    // 输出为csv格式文件
    std::ofstream file;//创建一个ofstream对象
    std::string file_path = "/home/ybe/ROS/hexapod_moveit/src/traj_tracking/data/"+ref_traj+".csv";
    // std::cout << file_path << std::endl;
	file.open(file_path,std::ios::out);
    file<<"time"<<","<<"actual x"<<","<<"actual y"<<","<<"actual z"<<","<<"actual R"<<","<<"actual P"<<","<<"actual Y"<<std::endl;
    

    geometry_msgs::Twist vel;
    //4-2.设置发送频率
    ros::Rate rate(50);
    //4-3.循环发送
    ros::WallTime start = ros::WallTime::now();
    ros::WallTime prev = ros::WallTime::now();
    ros::WallTime now = ros::WallTime::now();
    double dt(0);
    double elapsed(0);
    bool done = false;
    tf2::Quaternion  actual_qtn;
    double roll, pitch, yaw;
    while (!done && ros::ok())
    {
        now = ros::WallTime::now();
        if (actual_vel < 0.001){
            start = now;
        }
        dt = now.toSec() - prev.toSec();
        prev = now;
        elapsed = now.toSec() - start.toSec();

        linx = set_vel;
        angz = calc_angz(elapsed);

        if(elapsed >= 4*PI){
            linx = 0;
            angz = 0;
            done = true;
            file.close();
            // ros::shutdown();
        }

        vel.linear.x = linx;
        vel.angular.z = angz;
        vel_pub.publish(vel);


        try
        {
        //   解析 hand 中的点相对于 odom 的坐标
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("odom","hand",ros::Time(0));
            
            tf::Quaternion q(tfs.transform.rotation.x,
                tfs.transform.rotation.y,
                tfs.transform.rotation.z,
                tfs.transform.rotation.w);
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            file<<elapsed<<","<<tfs.transform.translation.x<<","<<tfs.transform.translation.y<<","<<tfs.transform.translation.z<<","
            <<roll<<","<<pitch<<","<<yaw<<std::endl;
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("异常信息:%s",e.what());
        }

        ros::spinOnce();
        usleep(1000);
    }

    return 0;
}

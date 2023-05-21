#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
 
#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf2_ros/transform_listener.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <fstream>
#include<iostream>
using namespace std;

double pos_x, pos_y, vel_x, vel_y, ang_z;
// geometry_msgs::Quaternion qtn;
Eigen::Quaterniond base_qtn;

class multiThread
{
public:
    multiThread()
    {
        // sub = nm.subscribe("/odom",50,&multiThread::OdomCallback,this);
        sub = nm.subscribe("/gazebo/model_states",50,&multiThread::StateCallback,this);
    }

    // void OdomCallback(const nav_msgs::Odometry& msg)
    // {
    //     // int end_num = sizeof(msg.pose);
    //     // ROS_INFO("num: %d", end_num);
    //     pos_x = msg.pose.pose.position.x;
    //     pos_y = msg.pose.pose.position.x;
    //     base_qtn.x() = msg.pose.pose.orientation.x;
    //     base_qtn.y() = msg.pose.pose.orientation.y;
    //     base_qtn.z() = msg.pose.pose.orientation.z;
    //     base_qtn.w() = msg.pose.pose.orientation.w;
    //     vel_x = msg.twist.twist.linear.x;
    //     vel_y = msg.twist.twist.linear.y;
    //     ang_z = msg.twist.twist.angular.z;

        
    // }

        void StateCallback(const gazebo_msgs::ModelStates& msg)
    {
        // int end_num = sizeof(msg.pose);
        // ROS_INFO("num: %d", end_num);
        pos_x = msg.pose[2].position.x;
        pos_y = msg.pose[2].position.y;
        base_qtn.x() = msg.pose[2].orientation.x;
        base_qtn.y() = msg.pose[2].orientation.y;
        base_qtn.z() = msg.pose[2].orientation.z;
        base_qtn.w() = msg.pose[2].orientation.w;
        vel_x = msg.twist[2].linear.x;
        vel_y = msg.twist[2].linear.y;
        ang_z = msg.twist[2].angular.z;

        
    }

private:
	ros::NodeHandle nm;
	ros::Subscriber sub;
};


double getyaw(Eigen::Quaterniond qua){
  tf::Quaternion q(qua.x(),
    qua.y(),
    qua.z(),
    qua.w());
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track");
  ros::NodeHandle nh;
  multiThread recOb;
  ros::AsyncSpinner spinner(1); // one threads
  spinner.start();
  usleep(30000);

  ros::Publisher trans_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/jaka_arm_controller/command",1);
  tf2_ros::Buffer buffer; 
  tf2_ros::TransformListener listener(buffer);

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
  const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // ros::Publisher display_publisher  = node_handle.advertise<moveit_msgs::DisplayTrajectory> ("/move_group/display_planned_path", 1, true);
//  moveit_msgs::DisplayTrajectory display_trajectory;
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  
  // 设置初始关节角度
  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = 0.0;
  joint_group_positions[2] = 1.5708;
  joint_group_positions[3] = -1.5708;
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
  sleep(1.0);

  // 输出为csv格式文件
  std::ofstream file;//创建一个ofstream对象
  file.open("/home/ybe/ROS/hexapod_moveit/src/traj_tracking/data/track_point.csv",std::ios::out);
  file<<"time"<<","<<"actual x"<<","<<"actual y"<<","<<"actual z"<<","<<"actual R"<<","<<"actual P"<<","<<"actual Y"<<std::endl;
    
  Eigen::Vector3d end_eff_trans;
  // Eigen::Quaterniond end_eff_qtn;
  Eigen::Matrix3d end_eff_rot;
  geometry_msgs::Pose target_pose1;
  moveit::planning_interface::MoveGroupInterface::Plan tracking_plan;
  moveit::planning_interface::MoveItErrorCode tracking_success;
  double timeout = 0.01;
  // 获取机器人状态
  moveit::core::RobotStatePtr init_state = group.getCurrentState();
  // current_state = group.getCurrentState();
  const Eigen::Isometry3d& end_effector_state = init_state->getGlobalLinkTransform("hand");
  // 世界坐标系下基座初始位姿
  Eigen::Quaterniond base_init_qtn = base_qtn;
  double base_init_yaw = getyaw(base_qtn);
  Eigen::Vector3d base_init_pos;
  base_init_pos(0) = pos_x;
  base_init_pos(1) = pos_y;
  base_init_pos(2) = 0.0;
  double rel_pos_x, rel_pos_y, rel_yaw;

  
  ros::Time start = ros::Time::now();
  ros::Time prev = ros::Time::now();
  ros::Time now = ros::Time::now();
  ros::Duration dt(0);
  ros::Duration elapsed(0);
  Eigen::VectorXd joint_group_prev_velocity;
  joint_group_prev_velocity.resize(6);
  Eigen::Isometry3d end_eff_current_state;
  ros::Rate rate(50);
  double roll, pitch, yaw;
  while(ros::ok())
  {
    // 相对初始化状态时基座固定坐标系的位姿
    rel_pos_x = (pos_x - base_init_pos(0)) * cos(base_init_yaw) + (pos_y - base_init_pos(1)) * sin(base_init_yaw);
    rel_pos_y = -(pos_x - base_init_pos(0)) * sin(base_init_yaw) + (pos_y - base_init_pos(1)) * cos(base_init_yaw);
    rel_yaw = getyaw(base_qtn) - base_init_yaw;

    // ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    // ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
    end_eff_trans = end_effector_state.translation();
    end_eff_rot = base_qtn.normalized().toRotationMatrix().transpose()*base_init_qtn.normalized().toRotationMatrix()*end_effector_state.rotation();
    // end_eff_qtn = Eigen::Quaterniond(end_eff_rot);
    // end_eff_qtn.normalize();

    // 笛卡尔空间规划
    // target_pose1.orientation.x = end_eff_qtn.x();
    // target_pose1.orientation.y = end_eff_qtn.y();
    // target_pose1.orientation.z = end_eff_qtn.z();
    // target_pose1.orientation.w = end_eff_qtn.w();
    // target_pose1.position.x = end_eff_trans(0);
    // target_pose1.position.y = (end_eff_trans(1) - pos_y - end_eff_trans(0) * sin(getyaw(base_qtn))) / cos(getyaw(base_qtn));
    // target_pose1.position.z = end_eff_trans(2);
    // group.setPoseTarget(target_pose1);


    // 关节空间规划
    double trans_x, trans_y;
    trans_x = (end_eff_trans(0) - rel_pos_x) * cos(rel_yaw) + (end_eff_trans(1) - rel_pos_y) * sin(rel_yaw);
    trans_y = -(end_eff_trans(0) - rel_pos_x) * sin(rel_yaw) + (end_eff_trans(1) - rel_pos_y) * cos(rel_yaw);
    end_eff_trans(0) = trans_x;
    end_eff_trans(1) = trans_y;
    end_eff_current_state = Eigen::Isometry3d::Identity();
    end_eff_current_state.rotate (end_eff_rot);
    end_eff_current_state.pretranslate (end_eff_trans);
    // cout<<"T1 from r,t:\n"<<end_eff_current_state.matrix()<<endl;
    current_state = group.getCurrentState();  
    std::vector<double> joint_group_prev_positions = group.getCurrentJointValues();
    bool found_ik = current_state->setFromIK(joint_model_group, end_eff_current_state, timeout);
    if (found_ik)
    {
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    group.setJointValueTarget(joint_group_positions);
    }
    else
    {
    ROS_INFO("Did not find IK solution");
    }
    
    // moveit规划任务执行
    // tracking_success = group.plan(tracking_plan);
    // // ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "initialization completed!" : "FALLED");
    // if(tracking_success)
    //     group.execute(tracking_plan);
    // else
    //     cout << "falled"<<endl;

  // 加入速度信息
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);//相对于末端执行器坐标的位姿
    Eigen::MatrixXd jacobian;
    current_state->getJacobian(joint_model_group,
                                current_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                reference_point_position, jacobian);
    // ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
    Eigen::VectorXd end_eff_twist;
    end_eff_twist.resize(6);
    end_eff_twist(0) = -vel_y * sin(base_init_yaw) - vel_x * cos(base_init_yaw);
    end_eff_twist(1) = -vel_y * cos(base_init_yaw) + vel_x * sin(base_init_yaw);
    end_eff_twist(2) = 0.0;
    end_eff_twist(3) = 0.0;
    end_eff_twist(4) = 0.0;
    end_eff_twist(5) = -ang_z;
    Eigen::VectorXd joint_group_velocity =jacobian.lu().solve(end_eff_twist);
    // ROS_INFO_STREAM("joint_group_velocity: \n" << joint_group_velocity << "\n");
    // ROS_INFO_STREAM("dt: \n" << dt.toSec() << "\n");


    now = ros::Time::now();
    dt = now - prev;
    prev = now;
    elapsed = now - start;
    // 直接对关节轨迹赋值
    trajectory_msgs::JointTrajectory joint_state;
    joint_state.header.stamp = now;
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

    // trajectory_msgs::JointTrajectoryPoint point0;
    // point0.positions.resize(6);
    // point0.velocities.resize(6);
    // for(size_t i=0;i<=joint_num;i++) {
    //   point0.positions[i] = joint_group_prev_positions[i];
    //   point0.velocities[i] = joint_group_prev_velocity[i];

    // }
    // joint_state.points[0] = point0;
    // joint_state.points[0].time_from_start = ros::Duration(dt.toSec()/5);

    trajectory_msgs::JointTrajectoryPoint point1;
    point1.positions.resize(6);
    point1.velocities.resize(6);
    for(size_t i=0;i<=joint_num;i++) {
      point1.positions[i] = (joint_group_prev_positions[i]+joint_group_positions[i])/2;
      point1.velocities[i] = (joint_group_prev_velocity[i]+joint_group_velocity[i])/2;

    }
    joint_state.points[0] = point1;
    joint_state.points[0].time_from_start = ros::Duration(dt.toSec()/2);


    trajectory_msgs::JointTrajectoryPoint point2;
    point2.positions.resize(6);
    point2.velocities.resize(6);
    for(size_t i=0;i<=joint_num;i++) {
      point2.positions[i] = joint_group_positions[i];
      point2.velocities[i] = joint_group_velocity[i];
    }
    joint_state.points.push_back(point2);
    joint_state.points[1].time_from_start = dt;
    trans_pub.publish(joint_state);

    joint_group_prev_velocity = joint_group_velocity;

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
        file<<elapsed.toSec()<<","<<tfs.transform.translation.x<<","<<tfs.transform.translation.y<<","<<tfs.transform.translation.z<<","
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
  

//   ros::shutdown();
}
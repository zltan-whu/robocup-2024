#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;
#define TRA_WINDOW 1000
#define TIMEOUT_MAX 0.05
#define NODE_NAME "pos_estimator"
//---------------------------------------相关参数-----------------------------------------------
int input_source;
float rate_hz;
Eigen::Vector3f pos_offset;
float yaw_offset;
string object_name;
ros::Time last_timestamp;

//---------------------------------------mid360相关------------------------------------------
Eigen::Vector3d pos_drone_mid360;
Eigen::Quaterniond q_mid360;

//---------------------------------------发布相关变量--------------------------------------------
ros::Publisher vision_pub;
ros::Publisher drone_state_pub;
ros::Publisher message_pub;
ros::Publisher odom_pub;
ros::Publisher trajectory_pub;
nav_msgs::Odometry Drone_odom;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void send_to_fcu();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


void mid360_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "camera_init")
    {
        pos_drone_mid360 = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        q_mid360 = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    }
    else
    {
        ROS_WARN("frame_id wrong!!!!!!!!!!!!!!!!!!!!");
    }
}



void timerCallback(const ros::TimerEvent &e)
{
    ROS_WARN("px4_pos_estimate.....................");
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_estimator");
    ros::NodeHandle nh("~");

    //　程序执行频率
    nh.param<float>("rate_hz", rate_hz, 70);
    //　定位设备偏移量
    nh.param<float>("offset_x", pos_offset[0], 0);
    nh.param<float>("offset_y", pos_offset[1], 0);
    nh.param<float>("offset_z", pos_offset[2], 0);
    nh.param<float>("offset_yaw", yaw_offset, 0);

    // 【订阅】MID360数据
    ros::Subscriber mid360_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, mid360_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#102), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);


    // 10秒定时打印，以确保程序在正确运行
    ros::Timer timer = nh.createTimer(ros::Duration(10.0), timerCallback);


    // 频率
    ros::Rate rate(rate_hz);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        send_to_fcu();

        rate.sleep();
    }

    return 0;
}

void send_to_fcu()
{
    geometry_msgs::PoseStamped vision;

   
	vision.pose.position.x = pos_drone_mid360[0];
	vision.pose.position.y = pos_drone_mid360[1];
	vision.pose.position.z = pos_drone_mid360[2];

	vision.pose.orientation.x = q_mid360.x();
	vision.pose.orientation.y = q_mid360.y();
	vision.pose.orientation.z = q_mid360.z();
	vision.pose.orientation.w = q_mid360.w();
   

    vision.header.stamp = ros::Time::now();
    vision_pub.publish(vision);
}



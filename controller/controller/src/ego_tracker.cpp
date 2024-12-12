#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define VELOCITY2D_CONTROL 0b101111000111 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
//需要用到VX/VY/VZ/YAW，所以这四个给0，其他都是1.
visualization_msgs::Marker trackpoint;
ros::Publisher *pubMarkerPointer;
unsigned short velocity_mask = VELOCITY2D_CONTROL;
mavros_msgs::PositionTarget current_goal;
mavros_msgs::RCIn rc;
int rc_value, flag = 0, flag1 = 0;
nav_msgs::Odometry position_msg;
geometry_msgs::PoseStamped target_pos;
mavros_msgs::State current_state;
float position_x, position_y, position_z,  current_yaw, targetpos_x, targetpos_y;
float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate; //EGO planner information has position velocity acceleration yaw yaw_dot
bool receive = false;//触发轨迹的条件判断
float pi = 3.14159265;
//read RC 5 channel pwm,estimate auto plan in certain range
 void rc_cb(const mavros_msgs::RCIn::ConstPtr&msg)
{
  rc = *msg;
  rc_value = rc.channels[4];
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

//read vehicle odometry
void position_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    position_msg=*msg;
    position_x = position_msg.pose.pose.position.x;
    position_y = position_msg.pose.pose.position.y;
    position_z = position_msg.pose.pose.position.z;
	tf2::Quaternion quat;
	tf2::convert(msg->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	current_yaw = yaw;
}

void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//读取rviz的航点
{
  target_pos = *msg;
  targetpos_x = target_pos.pose.position.x;
  targetpos_y = target_pos.pose.position.y;
}

//ego传来的指令  包含速度、加速度、位置、yaw
quadrotor_msgs::PositionCommand ego;
void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)//ego的回调函数
{
    receive = true;
	ego = *msg;
    ego_pos_x = ego.position.x;
	ego_pos_y = ego.position.y;
	ego_pos_z = ego.position.z;
	ego_vel_x = ego.velocity.x;
	ego_vel_y = ego.velocity.y;
	ego_vel_z = ego.velocity.z;
	ego_a_x = ego.acceleration.x;
	ego_a_y = ego.acceleration.y;
	ego_a_y = ego.acceleration.y;
	ego_yaw = ego.yaw;
	ego_yaw_rate = ego.yaw_dot;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ego_tracker");
	setlocale(LC_ALL,"");
	ros::NodeHandle nh;
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
	("mavros/state", 10, state_cb);//读取飞控状态的话题
	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
	("mavros/setpoint_raw/local", 1); //控制无人机的位置速度加速度和yaw以及yaw-rate，有掩码选择
	
	ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker> ("/track_drone_point", 5);
    pubMarkerPointer = &pubMarker;
	
	ros::Subscriber rc_sub=nh.subscribe<mavros_msgs::RCIn>
    ("mavros/rc/in",10,rc_cb);//读取遥控器通道的话题
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
	("mavros/cmd/arming");//控制无人机解锁的服务端
	ros::ServiceClient command_client = nh.serviceClient<mavros_msgs::CommandLong>
	("mavros/cmd/command");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
	("mavros/set_mode");//设置飞机飞行模式的服务端
	ros::Subscriber twist_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
	("/position_cmd", 10, twist_cb);//订阅egoplanner的规划指令话题
    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>
	("move_base_simple/goal", 10, target_cb);
	ros::Subscriber position_sub=nh.subscribe<nav_msgs::Odometry>
    ("mavros/local_position/odom",10,position_cb);
    ros::Rate rate(50.0); //控制频率大于30hz
	
	while(ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("Connected!!!");

	mavros_msgs::SetMode offb_set_mode; 
	offb_set_mode.request.custom_mode = "OFFBOARD";
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i)
	{
		current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		local_pos_pub.publish(current_goal);
		ros::spinOnce();
		rate.sleep();
	}

	    ros::Time last_request = ros::Time::now();
	while(ros::ok())
	{
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
       
	   //take off 1m
		if(!receive) //offboard模式下会保持在0，0，1的高度
		{
            current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			current_goal.header.stamp = ros::Time::now();
	        current_goal.type_mask = velocity_mask;
			current_goal.velocity.x = 0;
			current_goal.velocity.y = 0;
			current_goal.velocity.z = (1 - position_z) * 1;
			current_goal.yaw = current_yaw;
			// ROS_INFO("请等待");
		}

     //if receive plan in rviz, the EGO plan information can input mavros and vehicle can auto navigation
		if(receive)//触发后进行轨迹跟踪
		{
			float yaw_erro;
			yaw_erro = (ego_yaw - current_yaw);
			current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择local系
			current_goal.header.stamp = ros::Time::now();
	        current_goal.type_mask = velocity_mask;//对应的掩码设置，mavros_msgs::PositionTarget消息格式
			current_goal.velocity.x =  (ego_pos_x - position_x)*1;
			current_goal.velocity.y =  (ego_pos_y - position_y)*1;
			current_goal.velocity.z =  (ego_pos_z - position_z)*1;
            current_goal.yaw = ego_yaw;
			ROS_INFO("EGO规划速度: vel_x = %.2f", sqrt(pow(current_goal.velocity.x, 2)+pow(current_goal.velocity.y, 2)));
		}
		local_pos_pub.publish(current_goal);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
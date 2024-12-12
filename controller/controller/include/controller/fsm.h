#ifndef FSM_H
#define FSM_H
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <cstdlib>
#include <sstream>
#include <stdio.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include <quadrotor_msgs/PositionCommand.h>

#include "yolo_server/yolo.h"
#include "servo_server/servo_msgs.h"
#include "pid.h"
#include <yaml-cpp/yaml.h>

#define CONTROL_MASK 0b100111111000 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
#define VEL_MASK 0b100111000111
using namespace std;


class fsm
{
private:

  /*飞机的当前状态*/
  enum FSM_STATE
  {
    INIT,
    TAKING_OFF,
    LAND,
    HOLD,
    EXEC_MISSION,
    ACROSS,
    TRACKING
  }fsm_state_;

  enum MISSION_ACTION
  {
    YOLO_INIT,
    ADJUST,
    DROP,
    RISE,
    PUT,
    DROP_LAND
  }mission_action_;


  enum TRACK_MODE
  {
    POS_TRACK,
    VEL_TRACK
  }track_mode_;





/*----------------------------------------------ROS utils-------------------------------------------------*/
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber ego_pos_cmd_, state_sub_, odom_sub_, ego_init_sub_;
  ros::Publisher target_pub_, px4_ctrl_pub_, put_pub_;
  ros::ServiceClient arming_client_, set_mode_client_;
  ros::ServiceClient yolo_client_ , servo_client_, land_client_;
  

  /*接收来自ego的quadrotor_msgs，轨迹消息*/
  void quadmsgCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  /*定时回调，检查fsm的状态*/
  void fsm_state_loopCallback(const ros::TimerEvent &event);
  /*无人机的状态回调*/
  void px4_state_Callback(const mavros_msgs::State::ConstPtr& msg);
  /*订阅来自mavros的融合后的odom，mavros/local_position/odom*/
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);

  void ego_init_Callback(const std_msgs::Bool::ConstPtr &ego_init_Msg);

  void pubNewTarget(double next_target_[3]);

  void startMissionTimer();
  void stopMissionTimer();
  void missionTimer_loop_Callback(const ros::TimerEvent&);

/*-------------------------------------------fsm utils-------------------------------------------------*/
  /*改变fsm状态，并输出调用位置和当前状态*/
  void changeFSMState(FSM_STATE new_state, string pos_call);
  /*打印fsm状态*/
  void printFSMState();

  void changeMissionAction(MISSION_ACTION new_action, string pos_call);

  void checkPOS();
  //
  void updatepnpPos(mavros_msgs::PositionTarget &missin_pose, double dx, double dy, double k);
  void updatepnpPos_vel(mavros_msgs::PositionTarget &missin_pose, double dx, double dy, double k);

  void readYamlAndUpdateConfig(const std::string &yaml_file, bool (&do_put)[4]) ;

/*-------------------------------------------track utils-------------------------------------------------*/
  void track_();
  void track_vel();


private:

  unsigned short track_mask_ = CONTROL_MASK;
  unsigned short vel_mask_ = VEL_MASK;
  bool use_sim_;
  mavros_msgs::State px4_current_state_, px4_last_state_;
  mavros_msgs::SetMode offb_set_mode, land_set_mode;
  mavros_msgs::CommandBool arm_cmd;

  ros::Timer fsm_loop_timer_, mission_loop_timer;
  ros::Time hold_time_;
  bool in_hold_state_;
  mavros_msgs::PositionTarget taking_off_pose_;
  mavros_msgs::PositionTarget hold_pose_;
  bool do_mission, do_put_, have_just_put;
  mavros_msgs::PositionTarget mission_pose_; //执行任务时的目标位置
  double put_height, hold_height, detect_height, land_height, adjust_height;
  double k;//pnp 
  int overtime_t_, overtime_thre1, overtime_thre2, detect_overtime;
  

  bool have_odom_;
  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
  Eigen::Quaterniond odom_orient_;                  //里程计中的四元数
  Eigen::Vector3d targetPos_, targetVel_, targetAcc_;//ego中的目标位置
  double ego_Yaw_;                                    //ego中的目标角度
  bool use_yaw;
  Eigen::Vector3d takeoff_pos_;   //起飞位置
  Eigen::Vector3d hold_pos_;      //在空中悬停的坐标
  Eigen::Quaterniond hold_orient_;//在空中悬停的姿态
  Eigen::Vector3d cur_target_;   //当前的目标点位置
  double target_points_[50][3];
  int target_point_num_, target_id_;
  bool have_pub_next_;
  ros::Time last_pub_time_;
  bool ego_init_; 
  bool pause_track_;

  double pnp_limit_;
  int tent_id, land_id;
  int put_servo_id;
  bool do_land;
  bool have_adjusted;

  bool if_do_put[4];//飞的四个点位是否投递  通过读取yaml实时修改
  std::string yaml_file;

  PID pid_x, pid_y, pid_z;

  
  



  yolo_server::yolo yolo_detect_;  //填充服务请求和读取服务响应的对象。

  servo_server::servo_msgs servo;


public:
    fsm(){};
    ~fsm(){};
    void init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);




};


#endif //FSM_H

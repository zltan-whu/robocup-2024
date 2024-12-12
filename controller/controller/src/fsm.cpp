#include "controller/fsm.h"

void fsm::init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
{
    nh_ = nh;
    nh_private_ = nh_private;


    ego_init_ = false;

    pause_track_ = false;
    have_just_put = false;
    have_adjusted = false;
    do_land = false;
    overtime_t_ = 0;
    detect_overtime = 0;
    put_servo_id = 3;
    yaml_file = "/home/amov/config.yaml";

    readYamlAndUpdateConfig(yaml_file, if_do_put);

    pid_x.setParameters(1.0, 0.3, 0.3);
    pid_y.setParameters(1.0, 0.3, 0.3);
    pid_z.setParameters(1.0, 0.3, 0.3);


    taking_off_pose_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择local系
    taking_off_pose_.header.stamp = ros::Time::now();
    taking_off_pose_.type_mask = 2552;//对应的掩码设置，mavros_msgs::PositionTarget消息格式
    taking_off_pose_.position.x = 0.0;
    taking_off_pose_.position.y = 0.0;
    taking_off_pose_.position.z = 1.1;
    taking_off_pose_.yaw = 0.0;

    hold_pose_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    hold_pose_.type_mask = 2552;

    mission_pose_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    mission_pose_.type_mask = 2552;


    takeoff_pos_ = Eigen::Vector3d(taking_off_pose_.position.x, taking_off_pose_.position.y, taking_off_pose_.position.z);


    /*--------------------subscribe---------------------*/
    ego_pos_cmd_ = nh_.subscribe<quadrotor_msgs::PositionCommand>
                        ("planning/pos_cmd",10, boost::bind(&fsm::quadmsgCallback, this, _1));
    state_sub_ = nh_.subscribe<mavros_msgs::State>
                        ("mavros/state", 10, boost::bind(&fsm::px4_state_Callback, this, _1));
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>
                        ("mavros/local_position/odom", 10, boost::bind(&fsm::odomCallback, this, _1));
    ego_init_sub_ = nh_.subscribe<std_msgs::Bool>
                        ("/ego_state", 10, boost::bind(&fsm::ego_init_Callback, this, _1));
    /*--------------------Timer callback-----------------*/
    fsm_loop_timer_ = nh_.createTimer(ros::Duration(0.01), &fsm::fsm_state_loopCallback, this); // Define timer for constant loop rate
    /*---------------------service-------------------------*/
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    yolo_client_ = nh_.serviceClient<yolo_server::yolo>("yolo");
    // land_client_ = nh_.serviceClient<yolo_server::yolo>("land");
    servo_client_ = nh_.serviceClient<servo_server::servo_msgs>("servo_move");
    /*----------------------publish-----------------------------*/
    // pos_track_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    // vel_track_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10); 
    target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // mission_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pnp_target_pose", 10);
    px4_ctrl_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10); 
    put_pub_ = nh_.advertise<std_msgs::Int32>("/have_put_type", 10); 

    /*-------------------------param--------------------------------*/
    nh_private_.param("use_sim_", use_sim_, false);
    nh_private_.param("put_height", put_height, 0.5);
    nh_private_.param("hold_height", hold_height, 1.0);
    nh_private_.param("detect_height", detect_height, 1.5);
    nh_private_.param("land_height", land_height, 0.3);
    nh_private_.param("adjust_height", adjust_height, 1.0);
    nh_private_.param("do_mission", do_mission, false);
    nh_private_.param("do_put_", do_put_, false);
    nh_private_.param("use_yaw", use_yaw, false);
    nh_private_.param("pnp_k", k, 0.9);
    nh_private_.param("pnp_limit_", pnp_limit_, 0.05);
    nh_private_.param("overtime_thre1", overtime_thre1, 5);
    nh_private_.param("overtime_thre2", overtime_thre2, 5);
    nh_private_.param("tent_id", tent_id, 5);
    nh_private_.param("land_id", land_id, 3);


    nh_private_.param("target_point_num_", target_point_num_, -1);
    for (int i = 0; i < target_point_num_; i++)
    {
      nh_private_.param("target_points_" + to_string(i) + "_x", target_points_[i][0], -1.0);
      nh_private_.param("target_points_" + to_string(i) + "_y", target_points_[i][1], -1.0);
      nh_private_.param("target_points_" + to_string(i) + "_z", target_points_[i][2], -1.0);
    }
    target_id_ = 0;
    
    mission_action_ = YOLO_INIT;

    fsm_state_ = INIT;

}

void fsm::px4_state_Callback(const mavros_msgs::State::ConstPtr& msg)
{
    px4_last_state_ = px4_current_state_;
    px4_current_state_ = *msg;
    if(px4_current_state_.mode != px4_last_state_.mode)
    {
         ROS_INFO("Change mavMode From %s To: %s", px4_last_state_.mode.c_str(), px4_current_state_.mode.c_str());
    }
}

void fsm::ego_init_Callback(const std_msgs::Bool::ConstPtr &ego_init_Msg)
{
    if(ego_init_ != ego_init_Msg->data)
    {
        ego_init_ = ego_init_Msg->data;
        ROS_INFO("ego_init get! ego: %d", int(ego_init_));
    }
}

void fsm::odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    odom_pos_(0) = odomMsg->pose.pose.position.x;
    odom_pos_(1) = odomMsg->pose.pose.position.y;
    odom_pos_(2) = odomMsg->pose.pose.position.z;

    odom_vel_(0) = odomMsg->twist.twist.linear.x;
    odom_vel_(1) = odomMsg->twist.twist.linear.y;
    odom_vel_(2) = odomMsg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = odomMsg->pose.pose.orientation.w;
    odom_orient_.x() = odomMsg->pose.pose.orientation.x;
    odom_orient_.y() = odomMsg->pose.pose.orientation.y;
    odom_orient_.z() = odomMsg->pose.pose.orientation.z;

    have_odom_ = true;

}

void fsm::quadmsgCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
    targetPos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
    targetVel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
    targetAcc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
    ego_Yaw_ = double(cmd->yaw);
    if(fsm_state_ == HOLD && !pause_track_)//保证安全，只能从HOLD模式转为跟踪模式,并且没有到达目标点附近
    {
        changeFSMState(TRACKING, "EGO_MSG");
    }
}

void fsm::changeMissionAction(MISSION_ACTION new_action, string pos_call)
{
    static string action_str[6] = {"YOLO_INIT", "ADJUST", "DROP", "RISE", "PUT", "DROP_LAND"};
    int pre_act = int(mission_action_);
    mission_action_ = new_action;
    cout << "[" + pos_call + "]: from " + action_str[pre_act] + " to " + action_str[int(new_action)] << endl;
}

void fsm::changeFSMState(FSM_STATE new_state, string pos_call)
{
    static string state_str[7] = {"INIT", "TAKING_OFF", "LAND", "HOLD", "EXEC_MISSION", "ACROSS", "TRACKING"};
    int pre_s = int(fsm_state_);
    fsm_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void fsm::printFSMState()
{
    static string state_str[7] = {"INIT", "TAKING_OFF", "LAND", "HOLD", "EXEC_MISSION", "ACROSS", "TRACKING"};
    static string action_str[6] = {"YOLO_INIT", "ADJUST", "DROP", "RISE", "PUT", "DROP_LAND"};
    cout << "[FSM]: state: " + state_str[int(fsm_state_)] << endl;
    if(fsm_state_ == EXEC_MISSION)
    {
        cout << "[MISSION]: action: " + action_str[int(mission_action_)] << endl;
    }
}

void fsm::fsm_state_loopCallback(const ros::TimerEvent &event)
{
    fsm_loop_timer_.stop();
    static bool have_takeoff = false;
    static ros::Time last_request = ros::Time::now();
    /*----------一秒打印一次fsm状态----------*/
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMState();
      if (!have_odom_)
        ROS_WARN("no odom!!");
      fsm_num = 0;
      ROS_WARN("target id:%d", target_id_);
    }

    // readYamlAndUpdateConfig(yaml_file, if_do_put);
    // readYamlAndUpdateConfig(yaml_file, if_do_put);

    checkPOS();

    switch ((fsm_state_))
    {
    case INIT:
    {
        if(!px4_current_state_.connected)
        {
            ROS_WARN("DISCONNECTED!RETRY...");
            goto force_return;
        }
        else
        {
            ROS_INFO("CONNECTED!");
        }
        if(!have_odom_)
        {
            goto force_return;
        }

        changeFSMState(TAKING_OFF, "FSM");
        break;
    }
    
    case TAKING_OFF:
    {
        if(use_sim_)
        {
            offb_set_mode.request.custom_mode = "OFFBOARD";
            arm_cmd.request.value = true;
            if( px4_current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else 
            {
                if( !px4_current_state_.armed && (ros::Time::now() - last_request > ros::Duration(2.0)))
                {
                    if( arming_client_.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        }
        px4_ctrl_pub_.publish(taking_off_pose_);
        if((odom_pos_ - takeoff_pos_).norm()<0.3)
        {            
            changeFSMState(HOLD, "FSM");
            hold_pose_ = taking_off_pose_;
        }      


        break;

    }
    case HOLD://保持在空中悬停，悬停位姿需要在别处设置，刚起飞时为taking_off_pose_，之后则为刚进入hold状态时的里程计位姿
    {
        px4_ctrl_pub_.publish(hold_pose_);
        // std::cout<<"target_point_num_: "<<target_point_num_<<std::endl;

        


        if(!ego_init_ )
        {
            ROS_WARN("wait for ego...");
            break;
        }
        if(target_point_num_ < 1)
        {
            ROS_WARN("target_point_num_ should > 1 ...");
            break;
        }
        if(ego_init_ && (target_id_ ==0))
        {
            pubNewTarget(target_points_[target_id_]);
            last_pub_time_=ros::Time::now();
            target_id_++;
            break;
        }

        if(ego_init_ && (target_id_ <= 4)) //mission point
        {
            if((ros::Time::now()-last_pub_time_).toSec()<2)
            {
                break;
            }
            if(do_mission && !have_just_put  && (put_servo_id >= 1)&&(if_do_put[target_id_-1]))
            {                 
                changeFSMState(EXEC_MISSION, "FSM");
                changeMissionAction(YOLO_INIT, "EXEC_MISSION");
                break;
            }  
            last_pub_time_ = ros::Time::now();
            pubNewTarget(target_points_[target_id_]);
            have_just_put = false;
            target_id_++;
            break;
        }

        if(ego_init_ && (target_id_ < target_point_num_)) //across
        {
            if((ros::Time::now()-last_pub_time_).toSec()<2)
            {
                break;
            }
            
            last_pub_time_ = ros::Time::now();
            pubNewTarget(target_points_[target_id_]);
            have_just_put = false;
            target_id_++;
            break;
        }

        if(ego_init_ && (target_id_ == target_point_num_)) //land
        {
            if((ros::Time::now()-last_pub_time_).toSec()<2)
            {
                break;
            }
            do_land =true;        
            changeFSMState(EXEC_MISSION, "FSM");
            changeMissionAction(YOLO_INIT, "EXEC_MISSION");
            break;
            // if(do_mission && !have_just_put)
            // {         
            //     do_land =true;        
            //     changeFSMState(EXEC_MISSION, "FSM");
            //     changeMissionAction(YOLO_INIT, "EXEC_MISSION");
            //     break;
            // }  
            // last_pub_time_ = ros::Time::now();
            // pubNewTarget(target_points_[target_id_]);
            // have_just_put = false;
            // target_id_++;
            //break;
        }

        
         

        break;
    }
        
    case EXEC_MISSION:
    {
        
        switch (mission_action_)
        {
        case YOLO_INIT:
        {
            this->startMissionTimer();
            ROS_INFO("Waiting for the 'yolo' service to become available...");
            ros::service::waitForService("yolo");
            ROS_INFO("YOLO init! start detect!");

            // if(do_land)
            // {
            //     ROS_INFO("Waiting for the 'land' service to become available...");
            //     ros::service::waitForService("land");
            //     ROS_INFO("YOLO init! start land detect!");
            // }
            changeMissionAction(ADJUST, "EXEC_MISSION");           
            break;
        }
        case ADJUST:
        {
            bool flag = false;
            // if(do_land)
            // {
            //     flag = land_client_.call(yolo_detect_);
            // }
            // else
            // {
                flag = yolo_client_.call(yolo_detect_);
            // }
            if(flag)
            {
                // if(yolo_detect_.response.min_type == tent_id)
                // {
                //     std::cout<<"this type is tent , skippppppp......"<<std::endl;
                //     stopMissionTimer();
                //     have_just_put = true;
                //     changeFSMState(HOLD, "FSM");
                //     hold_pose_ = mission_pose_;
                //     break;
                // }
                overtime_t_ = 0;
                ROS_WARN("detect success, adjusting...");
                double dx = yolo_detect_.response.x/100.0;
                double dy = yolo_detect_.response.y/100.0;
                double dz = yolo_detect_.response.z/100.0;
                ROS_INFO("[dx, dy, dz]: [%.2f, %.2f, %.2f]", dx, dy, dz);
                ROS_WARN("min_type: %d", yolo_detect_.response.min_type);
                if(put_servo_id ==2)//left
                {
                    dx+=0.1;
                }
                else if (put_servo_id ==3)//back
                {
                    dy-=0.1;
                }
                else if (put_servo_id == 1)//right
                {
                    dx-=0.1;
                }
                else
                {
                    
                }
                updatepnpPos(mission_pose_, dx, dy, k);
                detect_overtime++;
                if((abs(dx)<0.2 && abs(dy)<0.2) || (detect_overtime > 800))
                {
                    if(!have_adjusted)
                    {
                        hold_time_ = ros::Time::now();
                        mission_pose_.position.z = adjust_height;
                        mission_pose_.header.stamp = ros::Time::now();
                        detect_overtime = 0;
                        have_adjusted = true;
                        break;
                    }
                    
                }
                if((abs(dx)<0.1 && abs(dy)<0.1) || (detect_overtime > 80))
                {
                    std::cout<<"detect_overtime: "<<detect_overtime<<std::endl;
                    std::cout<<"(ros::Time::now() - hold_time_).toSec():"<<(ros::Time::now() - hold_time_).toSec()<<std::endl;
                    if((ros::Time::now() - hold_time_).toSec()>=5)
                    {
                        std::cout<<"do_land:"<<do_land<<std::endl;
                        if(do_land)
                        {
                            changeMissionAction(DROP_LAND, "EXEC_MISSION");
                            break;
                        }
                        detect_overtime = 0;
                        ROS_INFO("adjust complete! start drop...");
                        changeMissionAction(DROP, "EXEC_MISSION");
                        hold_time_ = ros::Time::now();
                        break;
                    }
                    
                    
                }
            }
            else
            {
                //ROS_WARN("detect failed, retrying...");
                overtime_t_++;
            }  
            static int nn =0;
            if((overtime_t_ > overtime_thre1*60) && nn>=2)
            {
                overtime_t_ = 0;
                ROS_INFO("start drop...");
                changeMissionAction(DROP, "EXEC_MISSION");
                nn = 0;
                break;
            }  


            if(overtime_t_ > overtime_thre1*60)
            {
                overtime_t_ = 0;
                ROS_INFO("upupupupupup1...");
                mission_pose_.position.z += 0.30;
                //detect_height +=0.2;
                nn++;
                
                break;
            } 
            
                  

            break;
        }
        case DROP:
        {
            bool flag = yolo_client_.call(yolo_detect_);
            if(flag)
            {
                overtime_t_ = 0;
                ROS_WARN("droping with adjusting...");
                double dx = yolo_detect_.response.x/100.0;
                double dy = yolo_detect_.response.y/100.0;
                double dz = yolo_detect_.response.z/100.0;
                ROS_INFO("[dx, dy, dz]: [%.2f, %.2f, %.2f]", dx, dy, dz);
                ROS_WARN("min_type: %d", yolo_detect_.response.min_type);
                if(put_servo_id ==2)//left
                {
                    dx+=0.1;
                }
                else if (put_servo_id ==3)//back
                {
                    dy-=0.1;
                }
                else if (put_servo_id == 1)//right
                {
                    dx-=0.1;
                }
                else
                {
                    
                }
                updatepnpPos(mission_pose_, dx, dy, k);
            }
            else
            {
                //ROS_WARN("detect failed, retrying...");
                overtime_t_++;
            }
            mission_pose_.header.stamp = ros::Time::now();
            mission_pose_.position.z = put_height;
            if((abs(odom_pos_(2) - put_height) < 0.2) || overtime_t_ > overtime_thre2*100)
            {
                if((ros::Time::now()-hold_time_).toSec()>=5)
                {
                    ROS_INFO("Start put...");
                    changeMissionAction(PUT, "EXEC_MISSION");
                    hold_time_ = ros::Time::now();
                }
            }
            break;
        }
        case RISE:
        {
            std_msgs::Int32 put_msg;
            put_msg.data = yolo_detect_.response.min_type;
            put_pub_.publish(put_msg);
            mission_pose_.header.stamp = ros::Time::now();
            mission_pose_.position.z = hold_height;
            if(abs(odom_pos_(2) - hold_height) < 0.1)
            {
                ROS_INFO("mission finished...");
                
                stopMissionTimer();
                have_just_put = true;
                have_adjusted = false;
                changeFSMState(HOLD, "FSM");
                hold_pose_ = mission_pose_;
                pid_x.reset();
                pid_y.reset();
                pid_z.reset();
            }
            break;
        }
        case PUT:
        {
            if(do_put_)
            {
                if((ros::Time::now()-hold_time_).toSec()>=3)
                {
                    ROS_INFO("Waiting for the 'servo' service to become available...");
                    ros::service::waitForService("servo_move");
                    ROS_INFO("start put...");
                    servo.request.servo_num = put_servo_id;
                    bool flag = servo_client_.call(servo);
                    if(!flag)
                    {
                        ROS_WARN("put defeat, retrying...");
                        break;
                    }
                    ROS_INFO("put finished, start rise...");
                    put_servo_id--;
                    changeMissionAction(RISE, "EXEC_MISSION");
                }
                break;
            }
            changeMissionAction(RISE, "EXEC_MISSION");
            break;
        }   
        case DROP_LAND:
        {
            bool flag = yolo_client_.call(yolo_detect_);
            if(flag)
            {
                overtime_t_ = 0;
                ROS_WARN("landing with adjusting...");
                double dx = yolo_detect_.response.x/100.0;
                double dy = yolo_detect_.response.y/100.0;
                double dz = yolo_detect_.response.z/100.0;
                ROS_INFO("[dx, dy, dz]: [%.2f, %.2f, %.2f]", dx, dy, dz);
                ROS_WARN("min_type: %d", yolo_detect_.response.min_type);
                updatepnpPos(mission_pose_, dx, dy, k);
                detect_overtime++;

                if((abs(dx)<0.2 && abs(dy)<0.2)|| (detect_overtime > overtime_thre2*100))
                {
                    std::cout<<"have_adjusted: "<<have_adjusted<<std::endl;
                    if(!have_adjusted)
                    {
                        hold_time_ = ros::Time::now();
                        mission_pose_.position.z = 0.6;
                        mission_pose_.header.stamp = ros::Time::now();
                        detect_overtime = 0;
                        have_adjusted = true;
                        break;
                    }
                    // ROS_INFO("Start land...");
                    // changeFSMState(LAND, "FSM");
                }
                if((abs(dx)<0.1 && abs(dy)<0.1) || (detect_overtime > 80))
                {
                    std::cout<<"detect_overtime: "<<detect_overtime<<std::endl;
                    std::cout<<"(ros::Time::now() - hold_time_).toSec():"<<(ros::Time::now() - hold_time_).toSec()<<std::endl;
                    mission_pose_.position.z = 0.3;
                    mission_pose_.header.stamp = ros::Time::now();
                    if((ros::Time::now() - hold_time_).toSec()>=5)
                    {
                        
                        ROS_INFO("Start land...");
                        changeFSMState(LAND, "FSM");
                        break;
                    }
                    
                    
                }
            }
            else
            {
                //ROS_WARN("detect failed, retrying...");
                overtime_t_++;
            }
            //mission_pose_.header.stamp = ros::Time::now();
            //mission_pose_.position.z = land_height;
            
            break;
        }          
        
        default:
            break;
        }
        


        break;
    }
    case LAND:
    {
        land_set_mode.request.custom_mode = "AUTO.LAND";
        if( px4_current_state_.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            if( set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent)
            {
                ROS_INFO("LAND enabled");
            }
            last_request = ros::Time::now();
        } 
        else 
        {
            ROS_INFO("LANDED!!");
        }
        break;
    }
    case ACROSS:
        
        break;
    case TRACKING:
    {
        
        if(pause_track_)
        {          
            changeFSMState(HOLD, "FSM");
            hold_pose_.header.stamp = ros::Time::now();
            hold_pose_.position.x = odom_pos_(0);
            hold_pose_.position.y = odom_pos_(1);
            hold_pose_.position.z = odom_pos_(2);

            break;
        }   
        track_();
        break;
    }
    default:
        break;
    }
    force_return:;
    fsm_loop_timer_.start();
}

void fsm::track_()
{
    // geometry_msgs::PoseStamped track_pos_;
    // track_pos_.header.stamp = ros::Time::now();
    // track_pos_.pose.position.x = targetPos_(0);
    // track_pos_.pose.position.y = targetPos_(1);
    // track_pos_.pose.position.z = targetPos_(2);
    // track_pos_.pose.orientation.x = 0;
    // track_pos_.pose.orientation.y = 0;
    // track_pos_.pose.orientation.z = 0;
    // track_pos_.pose.orientation.w = 1;
    // pos_track_pub_.publish(track_pos_);
    // ROS_INFO("pos_track succeed!");
    mavros_msgs::PositionTarget current_goal;
    current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择local系
    current_goal.header.stamp = ros::Time::now();
    current_goal.type_mask = 2552;//对应的掩码设置，mavros_msgs::PositionTarget消息格式
    current_goal.position.x = targetPos_(0);
    current_goal.position.y = targetPos_(1);
    current_goal.position.z = targetPos_(2);
    if(use_yaw)
    {
        current_goal.yaw = ego_Yaw_;
    }
    else
    {
        current_goal.yaw = 0.0;
    }
    // ROS_INFO("Tracking Vel: [%f, %f, %f]", pow(current_goal.velocity.x, 2), pow(current_goal.velocity.y, 2), pow(current_goal.velocity.z, 2));
    px4_ctrl_pub_.publish(current_goal);
}

void fsm::track_vel()
{
    mavros_msgs::PositionTarget current_goal;
    current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择local系
    current_goal.header.stamp = ros::Time::now();
    current_goal.type_mask = vel_mask_;//对应的掩码设置，mavros_msgs::PositionTarget消息格式
    double err_x = targetPos_(0)-odom_pos_(0);
    double err_y = targetPos_(1)-odom_pos_(1);    
    double err_z = targetPos_(2)-odom_pos_(2);

    double vel_x = pid_x.calculate(err_x);
    double vel_y = pid_y.calculate(err_y);
    double vel_z = pid_x.calculate(err_z);

    current_goal.velocity.x = vel_x;
    current_goal.velocity.y = vel_y;
    current_goal.velocity.z = vel_z;    

    if(use_yaw)
    {
        current_goal.yaw = ego_Yaw_;
    }
    else
    {
        current_goal.yaw = 0.0;
    }
    // ROS_INFO("Tracking Vel: [%f, %f, %f]", pow(current_goal.velocity.x, 2), pow(current_goal.velocity.y, 2), pow(current_goal.velocity.z, 2));
    px4_ctrl_pub_.publish(current_goal);
}


void fsm::pubNewTarget(double next_target_[3])
{
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";

    goal.pose.position.x = next_target_[0];
    goal.pose.position.y = next_target_[1];
    goal.pose.position.z = next_target_[2];
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;

    target_pub_.publish(goal);
    ROS_INFO("Published next target to /move_base_simple/goal");

    cur_target_ = Eigen::Vector3d(next_target_[0], next_target_[1], next_target_[2]);
}

void fsm::startMissionTimer()
{
    mission_pose_.header.stamp = ros::Time::now();
    mission_pose_.position.x = odom_pos_(0);
    mission_pose_.position.y = odom_pos_(1);
    mission_pose_.position.z = detect_height;

    mission_loop_timer = nh_.createTimer(ros::Duration(0.02), &fsm::missionTimer_loop_Callback, this);

}
void fsm::stopMissionTimer()
{
    mission_loop_timer.stop();
}
void fsm::missionTimer_loop_Callback(const ros::TimerEvent&)
{
    px4_ctrl_pub_.publish(mission_pose_);
}

void fsm::checkPOS()
{
    if((odom_pos_ - cur_target_).norm()<0.5)
    {
        pause_track_ = true;
    }
    else
    {
        pause_track_ = false;
    }
}

void fsm::updatepnpPos(mavros_msgs::PositionTarget &mission_pose, double dx, double dy, double k)
{
    double m_dx, m_dy;
    if(dx > pnp_limit_)
    {
        m_dx = pnp_limit_;
    }
    else if (dx < -pnp_limit_)
    {
        m_dx = -pnp_limit_;
    }    
    else
    {
        m_dx = dx;
    }

    if(dy > pnp_limit_)
    {
        m_dy = pnp_limit_;
    }
    else if (dy < -pnp_limit_)
    {
        m_dy = -pnp_limit_;
    }    
    else
    {
        m_dy = dy;
    }
    
    
    double last_x_ = mission_pose.position.x;
    double last_y_ = mission_pose.position.y;

    double cur_x_ = odom_pos_(0) - m_dy;
    double cur_y_ = odom_pos_(1) - m_dx;

    // mission_pose.position.x = last_x_*k + cur_x_*(1-k);
    // mission_pose.position.y = last_y_*k + cur_y_*(1-k);

    

    mission_pose.position.x = odom_pos_(0) - m_dy - 0.15;
    mission_pose.position.y = odom_pos_(1) - m_dx;
    mission_pose.header.stamp = ros::Time::now();
}

void fsm::updatepnpPos_vel(mavros_msgs::PositionTarget &mission_pose, double dx, double dy, double k)
{
    double m_dx, m_dy;
    if(dx > pnp_limit_)
    {
        m_dx = pnp_limit_;
    }
    else if (dx < -pnp_limit_)
    {
        m_dx = -pnp_limit_;
    }    
    else
    {
        m_dx = dx;
    }

    if(dy > pnp_limit_)
    {
        m_dy = pnp_limit_;
    }
    else if (dy < -pnp_limit_)
    {
        m_dy = -pnp_limit_;
    }    
    else
    {
        m_dy = dy;
    }
    
    
    double last_x_ = mission_pose.position.x;
    double last_y_ = mission_pose.position.y;

    double cur_x_ = odom_pos_(0) - m_dy;
    double cur_y_ = odom_pos_(1) - m_dx;

    double err_dx = m_dy;
    double err_dy = m_dx;
    double err_dz = detect_height - odom_pos_(2);

    mission_pose.header.stamp = ros::Time::now();
    mission_pose.type_mask = vel_mask_;
   

    mission_pose.velocity.x = pid_x.calculate(err_dx);
    mission_pose.velocity.y = pid_y.calculate(err_dy);
    mission_pose.velocity.z = pid_z.calculate(err_dz);
   
}

void fsm::readYamlAndUpdateConfig(const std::string &yaml_file, bool (&do_put)[4]) 
{
    try {
        YAML::Node yaml = YAML::LoadFile(yaml_file);
        std::vector<bool> if_do_put = yaml["if_do_put"].as<std::vector<bool>>();
        if (if_do_put.size() == 4) 
        {
            for (size_t i = 0; i < 4; ++i) 
            {
                do_put[i] = if_do_put[i];
            }
        } 
        else
        {
            ROS_ERROR("YAML array 'if_do_put' size is not 4");
        }
        ROS_INFO("Updated config: if_do_put=[%s, %s, %s, %s]",
                 do_put[0] ? "true" : "false",
                 do_put[1] ? "true" : "false",
                 do_put[2] ? "true" : "false",
                 do_put[3] ? "true" : "false");
    } catch (const std::exception &e) {
        ROS_ERROR("Failed to read YAML file: %s", e.what());
    }
}

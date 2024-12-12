#include "ros/ros.h"
#include "servo_server/servo_msgs.h"
#include <iostream>
#include "servo_server/gpio.h"
using namespace std;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 初始化 ROS 节点
    ros::init(argc,argv,"servo_client");
    // 创建 ROS 句柄
    ros::NodeHandle nh;
    servo_server::servo_msgs ai;
    ai.request.servo_num = 2;
    // 创建 服务 对象
    ros::ServiceClient  servo_client_ = nh.serviceClient<servo_server::servo_msgs>("servo_move");
    ROS_INFO("qingqiu.........");

    bool flag = servo_client_.call(ai);
    if(flag)
    {
        std::cout<<"success.................";
    }
    else
    {
        
        ROS_WARN("..........................");
    }

    return 0;
}

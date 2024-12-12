#include "ros/ros.h"
#include "yolo_server/yolo.h"
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 初始化 ROS 节点
    ros::init(argc,argv,"client");
    yolo_server::yolo yolo_detect_;  //填充服务请求和读取服务响应的对象。
    // 创建 ROS 句柄
    ros::NodeHandle nh;
    
    yolo_detect_.request.num = -1;
    // 创建 服务 对象
    ros::ServiceClient  yolo_client_ = nh.serviceClient<yolo_server::yolo>("yolo");
    ROS_INFO("Waiting for the 'yolo' service to become available...");
    ros::service::waitForService("yolo");
    ROS_INFO("YOLO init! start detect!");

    while(ros::ok())
    {
        bool flag = yolo_client_.call(yolo_detect_);
        if(flag)
        {
            ROS_WARN("detect success, adjusting...");
            double dx = yolo_detect_.response.x/100.0;
            double dy = yolo_detect_.response.y/100.0;
            double dz = yolo_detect_.response.z/100.0;
            ROS_INFO("[dx, dy, dz]: [%.2f, %.2f, %.2f]", dx, dy, dz);
            ROS_WARN("min_type: %d", yolo_detect_.response.min_type);
        }
        else
        {
            ROS_WARN("detect filed, retrying...");
        }
    }
    

    return 0;
}
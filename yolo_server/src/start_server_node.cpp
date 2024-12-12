#include "ros/ros.h"
#include <YoloServer.h>



int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"yolo_Server");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    YoloServer server;
    server.init(nh, nh_private);
    server.startService();
    ros::spin();
    return 0;
}
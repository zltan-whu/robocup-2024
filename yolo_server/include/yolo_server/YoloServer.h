// YoloServer.h
#ifndef YOLOSERVER_H
#define YOLOSERVER_H

#include <iostream>
#include <chrono>
#include <cmath>
#include <map>
#include <vector>
#include <math.h>
#include "cuda_utils.h"
#include "logging.h"
#include "common.h"
#include "utils.h"
#include "preprocess.h"
#include "yolov5-detect.h"
#include "ros/ros.h"
#include "yolo_server/yolo.h"
#include <float.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int32.h>
using namespace cv;
using namespace std;

class YoloServer 
{
public:
    YoloServer(){};
    ~YoloServer(){delete det;det = nullptr;};
    void startService();
    void init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    void reset();
    //更新待识别列表 在控制程序里更新 投放完就更新
    void update_Callback(const std_msgs::Int32::ConstPtr& msg);
    //检查待识别列表
    void check_targets();

private:
    std::string engine_name_;
    bool doReq(yolo_server::yolo::Request& req, yolo_server::yolo::Response& resp);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool process_once(yolo_server::yolo::Response& resp);

private:
    ros::NodeHandle nh_;//初始化句柄
    ros::NodeHandle nh_private_;//初始化私有命名空间句柄
    ros::ServiceServer server;
    ros::Subscriber have_put_sub_;
    image_transport::Publisher detect_img_pub;
    sensor_msgs::ImagePtr detect_img_msg;

    int target1,target2,target3;//设置三个识别目标
    
    bool target1_put,target2_put,target3_put;//是否完成投放

    double drop_dis;

    std::vector<int> detect_type;

    bool have_init_;
    int deviceID;//相机设备号
    string engine_name;

    yolov5 *det = nullptr;
    VideoCapture cap;

    Mat latest_image;
    sensor_msgs::ImageConstPtr latest_ros_image; // 存储最新的ROS图像指针
    image_transport::Subscriber sub;

    double pnp_long;

     

};

#endif // YOLOSERVER_H


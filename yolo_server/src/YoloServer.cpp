// YoloServer.cpp
#include "YoloServer.h"
using namespace cv;
using namespace std;




void YoloServer::init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
{
    nh_ = nh;
    nh_private_ = nh_private;

    latest_image = cv::Mat(); 

    image_transport::ImageTransport it(nh_);
    sub = it.subscribe("camera/image", 1, &YoloServer::imageCallback, this);


    detect_img_pub = it.advertise("/detect_img", 1);

    have_put_sub_=nh_.subscribe<std_msgs::Int32>("/have_put_type", 10, boost::bind(&YoloServer::update_Callback, this, _1));

    this->reset();//开始的时候就reset为全部未识别
    nh_private_.param("drop_dis", drop_dis, 0.1);//距离目标10cm可以投
    nh_private_.param("target1", target1, 2);
    nh_private_.param("target2", target2, 3);
    nh_private_.param("target3", target3, 4);
    nh_private_.param("target1_put", target1_put, false);
    nh_private_.param("target2_put", target2_put, false);
    nh_private_.param("target3_put", target3_put, false);
    nh_private_.param("deviceID", deviceID, 0);
    nh_private_.param("engine_name", engine_name, string("/home/amov/robocup/src/yolo_server/engine/zhengchang.engine"));
    nh_private_.param("pnp_long", pnp_long, 0.1);//距离目标10cm可以投
    

    std::cout<<"engine_name: "<<engine_name<<std::endl;
    det = new yolov5(engine_name);
    cudaError_t cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) 
    {
        fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
        return; 
    }
}
void YoloServer::reset()
{
    target1_put = false;
    target2_put = false;
    target3_put = false;
}

void YoloServer::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    latest_ros_image = msg; // 直接保存ROS图像消息的指针
    //ROS_INFO("ROS Image pointer updated and stored");
}


bool YoloServer::doReq(yolo_server::yolo::Request& req, yolo_server::yolo::Response& resp) 
{

    bool is_detected = this->process_once(resp);

    return is_detected;
}


//这里可以改称void？就是如果没有检测到的话response.isdetect是false  但doReq的结果可以不是false
//doReq为false的情况就是一些输入错误，没有检测到可能就是视野里没有待检测目标 都可以问题不大
//每一帧请求一次，返回pnp的结果
bool YoloServer::process_once(yolo_server::yolo::Response& resp) 
{
    ROS_WARN("start process...");
    //每次检测一张图片都看一下要检测什么目标  放到detect_type里
    this->check_targets();
    if (!latest_ros_image) 
    {
        ROS_ERROR("No ROS image available for processing.");
        return false;
    }

    // 在需要时从ROS图像转换到OpenCV图像
    cv_bridge::CvImagePtr cv_ptr;
    try 
    {
        cv_ptr = cv_bridge::toCvCopy(latest_ros_image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("Failed to convert ROS image to OpenCV image: %s", e.what());
        return false;
    }

    cv::Mat img = cv_ptr->image;//从ROS图像转为opencv
    det->undisort(img);//除掉畸变
    int w = img.cols;
    int h = img.rows;
    unsigned char *d_image;
    cudaMalloc((void **)&d_image, sizeof(unsigned char) * w * h * 3);
    cudaMemcpy(d_image, img.data, w * h * 3 * sizeof(unsigned char), cudaMemcpyHostToDevice);

    // 进行目标检测 type_dis存储每一帧识别到的目标种类和对应的pnp结果
    std::map<int, std::vector<double>> type_dis = det->detect(d_image, w, h, img, this->detect_type);

    detect_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    detect_img_pub.publish(detect_img_msg);
    
    // 计算每个类型的距离 找出最近的
    double min_dis = DBL_MAX;
    int min_dis_type = -1;//最近距离对应的种类
    for(auto& pair : type_dis) 
    {
        std::vector<double>& dis_vec = pair.second;
        double dis = sqrt(dis_vec[0]*dis_vec[0]+dis_vec[1]*dis_vec[1]);//计算xy距离
        if(dis<min_dis)
        {
            min_dis = dis;
            min_dis_type = pair.first;
        }
    }

    //释放
    cudaFree(d_image);

    // 设置响应
    if(min_dis_type == -1) 
    {
        resp.is_detect = false;
        resp.x = resp.y = resp.z = 0;
        resp.min_type = -1;
    } else 
    {
        resp.is_detect = true;
        const std::vector<double>& dis_vec = type_dis[min_dis_type];
        resp.min_type = min_dis_type;//这样控制那边投放的时候就知道自己投放的是哪个最近的 然后update_targets(min_type)更新
        resp.x = dis_vec[0];
        resp.y = dis_vec[1];
        resp.z = dis_vec[2];
    }

    return resp.is_detect;

}

void YoloServer::check_targets()
{
    detect_type.clear();

    // if(this->target1_put==false){
    //     this->detect_type.push_back(this->target1);

    //     cout<<"还未被投放的目标有："<<this->target1<<endl;
    // }
    // if(this->target2_put==false){
    //     this->detect_type.push_back(this->target2);
        
    //     cout<<"还未被投放的目标有："<<this->target2<<endl;
    // }
    // if(this->target3_put==false){
    //     this->detect_type.push_back(this->target3);
        
    //     cout<<"还未被投放的目标有："<<this->target3<<endl;
    // }
}
//更新待识别列表 在控制程序里更新 投放完就更新
void YoloServer::update_Callback(const std_msgs::Int32::ConstPtr& msg)
{
    if(msg->data==this->target1){
        this->target1_put = true;
    }
    else if(msg->data==this->target2){
        this->target2_put = true;
    }
    else if(msg->data==this->target3){
        this->target3_put = true;
    }
}



void YoloServer::startService() 
{
    server = nh_.advertiseService("yolo", &YoloServer::doReq, this);
    ROS_INFO("yolo检测服务已经启动....");
}

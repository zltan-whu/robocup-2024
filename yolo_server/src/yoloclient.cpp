#include "ros/ros.h"
#include "yolo_server/yolo.h"

int main(int argc, char *argv[]){
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"AddInts_Client");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 客户端 对象
    ros::ServiceClient client = nh.serviceClient<yolo_server::yolo>("yolo");
    //等待服务启动成功
    //方式1
    ros::service::waitForService("yolo");
    yolo_server::yolo ai;
    ai.request.num = 150;
    //设置想投放的目标
    ros::param::set("target1",2);
    ros::param::set("target2",3);
    ros::param::set("target3",4);
    //设置想投放的目标有没有被投放
    ros::param::set("target1_detect",false);
    ros::param::set("target2_detect",false);
    ros::param::set("target3_detect",false);
    ROS_INFO("第一次请求");
    bool flag = client.call(ai);
    if(flag){
        ROS_INFO("请求正常处理,x响应结果:%f",ai.response.x);
        ROS_INFO("请求正常处理,y响应结果:%f",ai.response.y);
        ROS_INFO("请求正常处理,z响应结果:%f",ai.response.z);
    }
    else{
        ROS_INFO("请求处理失败");
    }
    ROS_INFO("第二次请求");
    bool flag1 = client.call(ai);
    if(flag1){
        ROS_INFO("请求正常处理,x响应结果:%f",ai.response.x);
        ROS_INFO("请求正常处理,y响应结果:%f",ai.response.y);
        ROS_INFO("请求正常处理,z响应结果:%f",ai.response.z);
    }
    else{
        ROS_INFO("请求处理失败");
    }
    ROS_INFO("第三次请求");
    bool flag2 = client.call(ai);
    if(flag2){
        ROS_INFO("请求正常处理,x响应结果:%f",ai.response.x);
        ROS_INFO("请求正常处理,y响应结果:%f",ai.response.y);
        ROS_INFO("请求正常处理,z响应结果:%f",ai.response.z);
    }
    else{
        ROS_INFO("请求处理失败");
    }
    return 0;
}
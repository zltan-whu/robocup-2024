#include "ros/ros.h"
#include "servo_server/servo_msgs.h"
#include <iostream>
#include "servo_server/gpio.h"
using namespace std;

// bool 返回值由于标志是否处理成功
bool doReq(servo_server::servo_msgs::Request& req,
          servo_server::servo_msgs::Response& resp){
    int i;
    ROS_INFO("服务器接收到的请求数据为:num = %d",req.servo_num);
    gpio *gp = new gpio();
    if(req.servo_num==1)
{
  	
	cout<<"START"<<endl;
	if(gp->openGpio(BUZZERA) != 0)  //打开GPIO
	{
		return false;	
	}
	usleep(500000);  //两次操作之间延时10ms
	if(gp->setGpioDirection(BUZZERA, DIRECTION) != 0)  //设置GPIO方向
	{
		gp->closeGpio(BUZZERA);
		return false;		
	}
	usleep(100000);
	while(i<100)
	{
		i++;
		if(gp->setGpioValue(BUZZERA, ON) != 0)  //设置高电平输出，开启
		{
			gp->closeGpio(BUZZERA);
			return false;		
		}
		usleep(600);
		if(gp->setGpioValue(BUZZERA, OFF) != 0)  //设置低电平输出，关闭
		{
			gp->closeGpio(BUZZERA);
			return false;		
		}
		usleep(19400);
	}
	cout<<"END"<<endl;
    resp.result = true;
    return true;
}
    if(req.servo_num==2)
{

	cout<<"START"<<endl;
	if(gp->openGpio(BUZZERB) != 0)  //打开GPIO
	{
		return false;	
	}
	usleep(500000);  //两次操作之间延时10ms
	if(gp->setGpioDirection(BUZZERB, DIRECTION) != 0)  //设置GPIO方向
	{
		gp->closeGpio(BUZZERB);
		return false;		
	}
	usleep(100000);
	while(i<100)
	{
		i++;
		if(gp->setGpioValue(BUZZERB, ON) != 0)  //设置高电平输出，开启
		{
			gp->closeGpio(BUZZERB);
			return false;		
		}
		usleep(500);
		if(gp->setGpioValue(BUZZERB, OFF) != 0)  //设置低电平输出，关闭
		{
			gp->closeGpio(BUZZERB);
			return false;		
		}
		usleep(19500);
	}
	cout<<"END"<<endl;
    resp.result = true;
    return true;
}
      if(req.servo_num==3)
{

	cout<<"START"<<endl;
	if(gp->openGpio(BUZZERA) != 0)  //打开GPIO
	{
		return false;	
	}
	usleep(500000);  //两次操作之间延时10ms
	if(gp->setGpioDirection(BUZZERA, DIRECTION) != 0)  //设置GPIO方向
	{
		gp->closeGpio(BUZZERA);
		return false;		
	}
	usleep(100000);
	while(i<100)
	{
		i++;
		if(gp->setGpioValue(BUZZERA, ON) != 0)  //设置高电平输出，开启
		{
			gp->closeGpio(BUZZERA);
			return false;		
		}
		usleep(300);
		if(gp->setGpioValue(BUZZERA, OFF) != 0)  //设置低电平输出，关闭
		{
			gp->closeGpio(BUZZERA);
			return false;		
		}
		usleep(19700);
	}
	cout<<"END"<<endl;
    resp.result = true;
    return true;
}
    resp.result = false;
    return false;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 初始化 ROS 节点
    ros::init(argc,argv,"servo_server");
    // 创建 ROS 句柄
    ros::NodeHandle nh;
    // 创建 服务 对象
    ros::ServiceServer server = nh.advertiseService("servo_move",doReq);
    ROS_INFO("服务已经启动....");
    // 回调函数处理请求并产生响应
    // 由于请求有多个，需要调用 ros::spin()
    ros::spin();
    return 0;
}

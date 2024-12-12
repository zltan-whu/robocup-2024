#ifndef GPIO_H
#define GPIO_H

#include <cstdlib>  
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h> 

#define ON "1"           //开启
#define OFF "0"          //关闭
#define BUZZERA "267"     //控制引脚
#define BUZZERB "419"
#define BUZZERC "421"
#define DIRECTION "out"  //方向为输出

class gpio
{
private:
    /* data */
public:
    gpio(/* args */);
    ~gpio();
    int openGpio(const char *port);  //导入gpio
    int closeGpio(const char *port); //导出gpio
    int readGPIOValue(const char *port,int &level);  //读取gpio值
    int setGpioDirection(const char *port,const char *direction);  //设置gpio方向
    int setGpioValue(const char *port,const char *level);  //设置gpio值
    int setGpioEdge(const char *port,const char *edge);  //设置gpio输入模式
};

#endif

#include "servo_server/gpio.h"

gpio::gpio(/* args */)
{
}

gpio::~gpio()
{
}

int gpio::openGpio(const char *port)
{
    int fd;
	const char *path = "/sys/class/gpio/export";

	fd = open(path, O_WRONLY);
	if(fd == -1)
	{
	   perror("Failed to open gpio! ");
	   return -1;
	}

	write(fd, port ,sizeof(port)); 
	close(fd);
	return 0; 	
}
	
int gpio::closeGpio(const char *port)
{
    int fd;
	const char *path = "/sys/class/gpio/unexport";

	fd = open(path, O_WRONLY);
	if(fd == -1)
	{
	   perror("Failed to open gpio! ");
	   return -1;
	}

	write(fd, port ,sizeof(port)); 
	close(fd); 	
	return 0;
}

int gpio::readGPIOValue(const char *port,int &level)
{
	int fd;
	char path[40];
	sprintf(path,"/sys/class/gpio/gpio%s/value", port);
	fd = open(path, O_RDONLY);
	if(fd == -1)
	{
		perror("Failed to read GPIO value!");
		return -1;
	}
	char value[2];
	read(fd,value,1);
	level = atoi(value);
	
	return 0;
}

int gpio::setGpioDirection(const char *port,const char *direction)
{
	int fd;
	char path[40];
	sprintf(path, "/sys/class/gpio/gpio%s/direction", port);
	fd = open(path, O_WRONLY);

	if(fd == -1)
	{
	   perror("Failed to set GPIO direction. ");

	   return -1;
	}

	write(fd, direction, sizeof(direction)); 
	close(fd); 
	return 0;
}

int gpio::setGpioEdge(const char *port,const char *edge)
{
	int fd;
	char path[40];
	sprintf(path, "/sys/class/gpio/gpio%s/edge", port);
	fd = open(path, O_WRONLY);

	if(fd == -1)
	{
	   perror("Failed to set GPIO edge. ");

	   return -1;
	}

	write(fd, edge, sizeof(edge)); 
	close(fd); 
	return 0;
}

int gpio::setGpioValue(const char *port,const char *level)
{
	int fd;
	char path[40];
	sprintf(path, "/sys/class/gpio/gpio%s/value", port);
	fd = open(path, O_RDWR);
	if(fd == -1)
	{
	   perror("Failed to set GPIO value! ");
	   return -1;
	}       

	write(fd, level, sizeof(level));
	close(fd);
	return 0;
}



#include <ros/ros.h>
#include <controller/fsm.h>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "controll_node");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_("~");

  fsm fsm;

  fsm.init(nh_, nh_private_);

  // ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

#include "ros/ros.h"
#include "ros_libvlc/RosLibVlc.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "libvlc_wrapper");
  ros::NodeHandle nh("~");

  RosLibVlc *rlv = RosLibVlc::getInstance(nh);

  ROS_INFO("Ready to go...");

  rlv->spin();
}
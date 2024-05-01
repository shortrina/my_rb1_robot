
#include "../include/my_rb1_ros/my_rb1_rotate.hpp"
#include <ros/init.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service");

  MyRb1Rotate rotate;

  ROS_INFO("Ready to rotate.");

  ros::spin();
  return 0;
}

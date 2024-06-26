#ifndef MY_RB1_ROTATE_HPP
#define MY_RB1_ROTATE_HPP

#include "my_rb1_ros/Rotate.h"
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <tf/tf.h>

class MyRb1Rotate {
private:
  ros::NodeHandle nh_;
  ros::ServiceServer service;

  ros::Subscriber rotate_sub_;
  nav_msgs::Odometry sub_position_msg_;

  ros::Publisher rotate_pub_;
  geometry_msgs::Twist pub_rotate_msg_;

  int degrees = 0; // Requested degrees

  double angluar_velocity = 1.0;
  double current_angle = 0.0;
  double target_angle = 0.0;

  double input_radians = 0.0;

  bool result = false; // Response result
  bool rotate_start = false;

  boost::mutex mutex_odom;
  const double PI = 3.14159265358979323846;

  ros::Rate loop_rate_ = ros::Rate(1.0); // 100Hz

  bool Service_Rotate_Callback(my_rb1_ros::Rotate::Request &req,
                               my_rb1_ros::Rotate::Response &resp);
  void Get_MyRb1_Position_Callback(
      const nav_msgs::Odometry::ConstPtr &nav_rb1_Odometry_status);

public:
  MyRb1Rotate();

  bool SetMyRb1RotateMsg(int degrees = 0);
};

#endif // MY_RB1_ROTATE_HPP

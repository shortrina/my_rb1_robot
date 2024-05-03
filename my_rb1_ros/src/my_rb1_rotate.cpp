

#include "../include/my_rb1_ros/my_rb1_rotate.hpp"
#include "ros/init.h"

MyRb1Rotate::MyRb1Rotate() {

  /* Service call for rotate */
  service = nh_.advertiseService("/rotate_robot",
                                 &MyRb1Rotate::Service_Rotate_Callback, this);

  /* Publisher and Subscriber for rotate */
  /* Send Command to Rb1 */
  rotate_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  /* Get current position from Rb1 */
  rotate_sub_ = nh_.subscribe("/odom", 1000,
                              &MyRb1Rotate::Get_MyRb1_Position_Callback, this);

  ROS_INFO("Ready to rotate.");
}

/* Service call for rotate */
bool MyRb1Rotate::Service_Rotate_Callback(my_rb1_ros::Rotate::Request &req,
                                          my_rb1_ros::Rotate::Response &resp) {
  ROS_INFO("Rotate received. Degrees: %d", req.degrees);

  degrees = req.degrees;

  result = SetMyRb1RotateMsg(degrees);

  resp.result = result;

  return true;
}

/* Get Position from Rb1 */
void MyRb1Rotate::Get_MyRb1_Position_Callback(
    const nav_msgs::Odometry::ConstPtr &nav_rb1_Odometry_status) {

  sub_position_msg_ = *nav_rb1_Odometry_status;
  // Get the current angle from the odometry
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(sub_position_msg_.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // Update the current angle with thread protection
  mutex_odom.lock();
  current_angle = yaw;
  mutex_odom.unlock();
}

bool MyRb1Rotate::SetMyRb1RotateMsg(int degrees) {
  ROS_INFO("Set MyRb1 Rotate Msg.");
  bool rotation_complete = false;
  double tolerance = 0.15;
  target_angle = degrees * PI / 180.0; // Convert to radians

  double start_angle;
  mutex_odom.lock();
  start_angle = current_angle;
  mutex_odom.unlock();

  double relative_angle = target_angle - start_angle;
  relative_angle =
      atan2(sin(relative_angle), cos(relative_angle)); // Normalize the angle

  pub_rotate_msg_.angular.z = 0.5; // Angular velocity (adjust as needed)

  while (ros::ok()) {
    mutex_odom.lock();
    double current_relative_angle = current_angle - start_angle;
    current_relative_angle =
        atan2(sin(current_relative_angle), cos(current_relative_angle));
    mutex_odom.unlock();

    double angle_diff = fabs(current_relative_angle - relative_angle);
    if (angle_diff < tolerance) {
      pub_rotate_msg_.angular.z = 0.0;
      rotate_pub_.publish(pub_rotate_msg_);
      rotation_complete = true;
      break;
    }

    rotate_pub_.publish(pub_rotate_msg_);
    ros::spinOnce();
    loop_rate_.sleep();
  }

  return rotation_complete;
}

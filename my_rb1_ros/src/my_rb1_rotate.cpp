

#include "../include/my_rb1_ros/my_rb1_rotate.hpp"
#include "ros/init.h"
#include <cmath>
#include <iostream>

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

/* Get Position from Rb1 */
void MyRb1Rotate::Get_MyRb1_Position_Callback(
    const nav_msgs::Odometry::ConstPtr &nav_rb1_Odometry_status) {

  sub_position_msg_ = *nav_rb1_Odometry_status;

  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(sub_position_msg_.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  current_angle = yaw;
}

/* Service call for rotate */
bool MyRb1Rotate::Service_Rotate_Callback(my_rb1_ros::Rotate::Request &req,
                                          my_rb1_ros::Rotate::Response &resp) {
  ROS_INFO("Rotate received. Degrees: %d", req.degrees);

  double init_position = current_angle;
  double size_angle = 0.0;
  double input_angle = req.degrees * PI / 180.0;
  double target_angle = current_angle + input_angle;

  double tolerance = 1.0; // degrees

  // Calculate the difference between target and current angles
  double angleDiff = input_angle;

  ROS_INFO("target_angle : %f ", target_angle);
  ROS_INFO("input_angle : %f , current_angle : %f", input_angle, current_angle);

  // Define the direction of rotation based on the sign of angleDiff
  double direction = (input_angle > 0) ? 1.0 : -1.0;

  while (std::fabs(input_angle) > std::fabs(size_angle)) {
    // Publish twist message to rotate the robot
    pub_rotate_msg_.angular.z =
        direction * 0.5; // Angular velocity (adjust as needed)
    rotate_pub_.publish(pub_rotate_msg_);

    ros::spinOnce();
    loop_rate_.sleep();

    if (input_angle < PI) {
      size_angle = current_angle - init_position;
    } else {
      size_angle = current_angle - init_position + 0.1;
    }

    // ROS_INFO(" ");
    ROS_INFO("size_angle : %f , current_angle : %f", size_angle, current_angle);
    // ROS_INFO("size_angle : %f", size_angle);

  } // end while

  ROS_INFO("input_angle %f size_angle : %f", std::fabs(input_angle),
           fabs(size_angle));
  //  Stop the robot
  pub_rotate_msg_.angular.z = 0.0;
  rotate_pub_.publish(pub_rotate_msg_);

  loop_rate_.sleep();
  angleDiff = std::fabs(input_angle) - std::fabs(size_angle);
  // Set rotation complete flag
  if (std::fabs(angleDiff) <= tolerance) {
    rotation_complete = true;
    ROS_INFO("Tolerance is bigger");
  } else {
    rotation_complete = false;
    ROS_INFO("Tolerance is smaller");
  }
  resp.result = rotation_complete;

  return rotation_complete;
}

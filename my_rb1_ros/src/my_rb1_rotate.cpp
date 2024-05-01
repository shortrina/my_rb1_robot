

#include "../include/my_rb1_ros/my_rb1_rotate.hpp"

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

  if (degrees < -90 || degrees > 90) {
    ROS_INFO("Degree should be between -90 and 90.");
    return false;
  }

  result = SetMyRb1RotateMsg(degrees);

  resp.result = result;

  return true;
}

/* Get Position from Rb1 */
void MyRb1Rotate::Get_MyRb1_Position_Callback(
    const nav_msgs::Odometry::ConstPtr &nav_rb1_Odometry_status) {

  sub_position_msg_ = *nav_rb1_Odometry_status;
  double roll, pitch, yaw;

  tf::Quaternion q(sub_position_msg_.pose.pose.orientation.x,
                   sub_position_msg_.pose.pose.orientation.y,
                   sub_position_msg_.pose.pose.orientation.z,
                   sub_position_msg_.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  current_yaw = yaw;
}

bool MyRb1Rotate::SetMyRb1RotateMsg(int degrees) {
  ROS_INFO("Set MyRb1 Rotate Msg.");

  // Convert to radians
  target_yaw = current_yaw + degrees * PI / 180.0;
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = (target_yaw > current_yaw) ? 0.1 : -0.1;

  while (fabs(target_yaw - current_yaw) > 0.01) {
    rotate_pub_.publish(cmd_vel);
    ros::spinOnce();
  }

  cmd_vel.angular.z = 0;
  rotate_pub_.publish(cmd_vel);

  return true;
}

nav_msgs::Odometry MyRb1Rotate::GetMyRb1Position() {
  ROS_INFO("Get MyRb1 Position.");
  return sub_position_msg_;
}

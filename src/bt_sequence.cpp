#include "bt_sequence.h"

BT::NodeStatus MoveForward::tick() {

  geometry_msgs::Twist msg;
  msg.linear.x = 0.5;
  msg.angular.z = 0;

  ROS_INFO("Let's go forward!!! %f, %f", msg.linear.x, msg.angular.z);

	for (int count_num=0; count_num<1500000; count_num++)
		_pub.publish(msg);

  return BT::NodeStatus::SUCCESS;
};

BT::NodeStatus Turn::tick() {

  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.angular.z = 0.5;

  ROS_INFO("Let's Turn!!! %f, %f", msg.linear.x, msg.angular.z);

	for (int count_num=0; count_num<1000000; count_num++)
		_pub.publish(msg);

	return BT::NodeStatus::SUCCESS;
};

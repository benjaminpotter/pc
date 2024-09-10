/// @file pc_node.cpp
///
/// @author Ben Potter
/// @date 24-09-10

#include <ros/ros.h>

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "pc");

	ROS_INFO("Hello, world!");

	ros::spinOnce();

	return 0;
}

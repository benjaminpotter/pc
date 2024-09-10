/// @file pc_node.hpp
///
/// @author Ben Potter
/// @date 24-09-10

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <queue>

#pragma once


namespace pc {
	
class PCNode {

public:
	PCNode();
	virtual ~PCNode();

	void spin();

private:
	
	void handle_image(const sensor_msgs::ImageConstPtr &msg);
	void process_image(const sensor_msgs::ImageConstPtr &msg);
	double extract_azimuth_from_image(const cv::Mat image);

private:

	ros::NodeHandle node_handle;

	image_transport::ImageTransport image_transport;
	image_transport::Subscriber image_subscriber;

	std::queue<sensor_msgs::ImageConstPtr> msg_queue;

	ros::Publisher orientation_publisher;

};

}

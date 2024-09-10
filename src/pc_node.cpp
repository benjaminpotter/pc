/// @file pc_node.cpp
///
/// @author Ben Potter
/// @date 24-09-10

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "pc/pc_node.hpp"

namespace pc {

PCNode::PCNode() :
	node_handle(),
	image_transport(node_handle)
{

	image_subscriber = image_transport.subscribe("/arena_camera_node/image_raw", 1, &PCNode::handle_image, this);
	ROS_INFO("subscribed to image topic");

}

PCNode::~PCNode() 
{

}

void PCNode::spin() 
{

	ros::Rate loop_rate(10);
	while(node_handle.ok()) 
	{
		if(!msg_queue.empty()) {
			process_image(msg_queue.front());
			msg_queue.pop();
		}

		cv::waitKey(30);
		loop_rate.sleep();	
	}

	ROS_INFO("stop node");
}

void PCNode::handle_image(const sensor_msgs::ImageConstPtr &msg) 
{
	ROS_INFO("add image to queue");
	msg_queue.push(msg);

}

void PCNode::process_image(const sensor_msgs::ImageConstPtr &msg)
{
	ROS_INFO("process image");

	cv_bridge::CvImageConstPtr image_ptr;
	image_ptr = cv_bridge::toCvShare(msg, "mono8");

	cv::Mat image_scaled;
	cv::resize(image_ptr->image, image_scaled, cv::Size(), 0.5, 0.5, cv::InterpolationFlags::INTER_AREA);
	cv::imshow("image_raw", image_scaled);
}


}



int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "pc");
	pc::PCNode node;

	// Allow another thread to process ROS events.
	boost::thread th(boost::bind(&ros::spin));

	node.spin();

	return 0;
}

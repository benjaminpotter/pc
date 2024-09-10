/// @file pc_node.cpp
///
/// @author Ben Potter
/// @date 24-09-10

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <novatel_oem7_msgs/INSPVA.h>
#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#define CCOMPASS_IMPLEMENTATION
#include "ccompass.h"

#include "pc/pc_node.hpp"


namespace pc {

PCNode::PCNode() :
	node_handle(),
	image_transport(node_handle)
{
	image_subscriber = image_transport.subscribe("/arena_camera_node/image_raw", 1, &PCNode::handle_image, this);
	ROS_INFO("subscribed to image topic");

	reference_subscriber = node_handle.subscribe("/novatel/oem7/inspva", 1, &PCNode::handle_reference, this);
	ROS_INFO("subscribed to reference topic");

	orientation_publisher = node_handle.advertise<geometry_msgs::Vector3Stamped>("/pc/orientation", 1);
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

		// ROS_INFO("image azimuth: %f", last_image_azimuth);
		// ROS_INFO("reference azimuth: %f", last_reference_azimuth);

		cv::waitKey(30);
		loop_rate.sleep();	
	}

	ROS_INFO("stop node");
}

void PCNode::handle_image(const sensor_msgs::ImageConstPtr &msg) 
{
	msg_queue.push(msg);
}


void PCNode::handle_reference(const novatel_oem7_msgs::INSPVA &msg)
{
	last_reference_azimuth = msg.azimuth;
}

void PCNode::process_image(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImageConstPtr image_ptr;
	image_ptr = cv_bridge::toCvShare(msg, "mono8");

	cv::Mat image_scaled;
	cv::resize(image_ptr->image, image_scaled, cv::Size(), 0.5, 0.5, cv::InterpolationFlags::INTER_AREA);
	cv::imshow("image_raw", image_scaled);
	
	double azimuth = extract_azimuth_from_image(image_ptr->image);
	last_image_azimuth = azimuth;
	
	geometry_msgs::Vector3Stamped orientation;
	orientation.header = msg->header;
	orientation.vector.x = 0.0;
	orientation.vector.y = azimuth;
	orientation.vector.z = 0.0;
	orientation_publisher.publish(orientation);
}

double PCNode::extract_azimuth_from_image(const cv::Mat image) 
{
	cv::Size dim = image.size();
	int w = dim.width/2, h = dim.height/2;

	struct cc_stokes *stokes_vectors;
	stokes_vectors = (struct cc_stokes*) malloc(sizeof(struct cc_stokes) * w * h);
	cc_compute_stokes(image.data, stokes_vectors, w, h);

	double *aolps;
	aolps = (double*) malloc(sizeof(double) * w * h);

	double *dolps;
	dolps = (double*) malloc(sizeof(double) * w * h);

	cc_transform_stokes(stokes_vectors, w, h);
	cc_compute_aolp(stokes_vectors, aolps, w, h);
	cc_compute_dolp(stokes_vectors, dolps, w, h);

	double azimuth;
	cc_hough_transform(aolps, dolps, w, h, &azimuth);

	free(stokes_vectors);
	free(aolps);
	free(dolps);	

	return azimuth * 180 / 3.14159265;
}

}


int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "pc");
	pc::PCNode node;

	// Process ROS events on another thread.
	boost::thread th(boost::bind(&ros::spin));

	node.spin();

	return 0;
}

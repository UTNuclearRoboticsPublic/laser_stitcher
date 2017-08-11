
#include <ros/ros.h>
#include "laser_stitcher/stationary_scan.h"
#include "pcl_processing_server/server.h"
#include "pcl_processing_server/pcl_utilities.h"
#include "pcl_processing_server/pcl_process_publisher.h"
#include "pcl_processing_server/primitive_search.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_stitcher_client");

	ros::NodeHandle nh;
	ros::ServiceClient scanning_client = nh.serviceClient<laser_stitcher::stationary_scan>("laser_stitcher/stationary_scan");
	ros::ServiceClient processing_client = nh.serviceClient<pcl_processing_server::pcl_process>("pcl_service");
	ros::ServiceClient search_client = nh.serviceClient<pcl_processing_server::primitive_process>("primitive_search");

	laser_stitcher::stationary_scan scan_srv;
	scan_srv.request.min_angle = -1.57;
	scan_srv.request.max_angle =  1.57;
	scan_srv.request.external_angle_sensing = false;

	//pcl_processing_server::pcl_process basic_process;
	//std::string temp = "fish";
	//PCLTaskCreation::processFromYAML(&basic_process, temp, "pcl_process");

	//PCLProcessPublisher basic_publisher; 
	//basic_publisher.updateNodeHandle(nh);
  	//basic_publisher.updatePublishers(basic_process);

	//PCLProcessPublisher search_publisher; 
	//pcl_processing_server::primitive_process search_process;
	ros::Duration(2.0).sleep();

	while( ros::ok() )
	{
		
		if( ! scanning_client.call(scan_srv) )
			ROS_ERROR_STREAM("Scanning service call failed - prob not up yet");
		else
			ROS_ERROR_STREAM("Successfully called scanning service - pointcloud output size is " << scan_srv.response.output_cloud.height*scan_srv.response.output_cloud.width << ".");
		ros::Duration(0.2).sleep();
/*
		if( ! processing_client.call(basic_process))
			ROS_ERROR_STREAM("Processing service call failed - prob not up yet");
		else 
		{
			ROS_ERROR_STREAM("Successfully called processing service.");
			basic_publisher.publish(basic_process);
		}  

		if( ! search_client.call(search_process))
			ROS_ERROR_STREAM("Processing service call failed - prob not up yet");
		else 
		{
			ROS_ERROR_STREAM("Successfully called processing service.");
			search_publisher.publish(search_process);
		}
*/

		//if( ! )

		break;
	}

}	
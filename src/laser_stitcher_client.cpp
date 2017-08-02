
#include <ros/ros.h>
#include "laser_stitcher/stationary_scan.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_stitcher");

	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<laser_stitcher::stationary_scan>("laser_stitcher/stationary_scan");

	laser_stitcher::stationary_scan scan_srv;
	scan_srv.request.min_angle = -1.57;
	scan_srv.request.max_angle =  1.57;
	scan_srv.request.external_angle_sensing = false;

	while( ros::ok() )
	{
		if( ! client.call(scan_srv) )
			ROS_ERROR_STREAM("Service call failed - prob not up yet");
		else
			ROS_ERROR_STREAM("Successfully called service - pointcloud output size is " << scan_srv.response.output_cloud.height*scan_srv.response.output_cloud.width << ".");
		ros::Duration(0.2).sleep();
	}

}	
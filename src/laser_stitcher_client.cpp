
#include <ros/ros.h>
#include "laser_stitcher/stationary_scan.h"
#include <pointcloud_processing_server/pointcloud_task_creation.h>
#include <pointcloud_processing_server/pointcloud_process.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_stitcher_client");

	ros::NodeHandle nh;
	ros::ServiceClient scanning_client = nh.serviceClient<laser_stitcher::stationary_scan>("laser_stitcher/stationary_scan");
	ros::ServiceClient client = nh.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");
	ros::Publisher final_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_stitcher/final_cloud", 1);

	std::string postprocessing_file_name;
	nh.param<std::string>("laser_stitcher/postprocessing_file_name", postprocessing_file_name, "laser_stitcher_postprocess");

	bool should_loop;
	nh.param<bool>("laser_stitcher/should_loop", should_loop, false);

	while(ros::ok())
	{
		laser_stitcher::stationary_scan scan_srv;
		float temp_angle;
		nh.param<float>("lidar_ur5_manager/min_angle", temp_angle, -1.57);
		scan_srv.request.min_angle = temp_angle;
		nh.param<float>("lidar_ur5_manager/max_angle", temp_angle, 1.57);
		scan_srv.request.max_angle = temp_angle;
		scan_srv.request.external_angle_sensing = false;

		ros::Duration(2.0).sleep();

		while( ros::ok() )
		{
			
			if( ! scanning_client.call(scan_srv) )
				ROS_ERROR_STREAM("[LaserStitcherClient] Scanning service call failed - prob not up yet");
			else
				ROS_ERROR_STREAM("[LaserStitcherClient] Successfully called scanning service - pointcloud output size is " << scan_srv.response.output_cloud.height*scan_srv.response.output_cloud.width << ".");
			ros::Duration(0.2).sleep();

			break;
		}

		pointcloud_processing_server::pointcloud_process postprocess;
		postprocess.request.pointcloud = scan_srv.response.output_cloud;
		PointcloudTaskCreation::processFromYAML(&postprocess, postprocessing_file_name, "pointcloud_service");

		if(postprocess.request.tasks.size() != 0)
		{
			while(!client.call(postprocess))
			{
				ROS_ERROR("[LaserStitcherClient] Postprocessing call failed - trying again...");
				ros::Duration(1.0).sleep();
			}

			int postprocess_length = postprocess.request.tasks.size();
			sensor_msgs::PointCloud2 final_cloud = postprocess.response.task_results[postprocess_length-1].task_pointcloud;
			ROS_INFO_STREAM("[LaserStitcherClient] Publishing final cloud! Size: " << final_cloud.height*final_cloud.width);
			final_cloud_pub.publish(final_cloud);
		}
		else
			ROS_WARN_STREAM("[LaserStitcherClient] Not preforming postprocessing - failed to get tasks from server. Probably something wrong with yaml files / parameters. Used file name " << postprocessing_file_name);

		// If we shouldn't loop, break the loop
		if(!should_loop)
			break;
	}	
	
	ros::Duration(1.0).sleep();

}	
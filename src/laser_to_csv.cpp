
// ROS Stuff
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// Std Stuff
#include <iostream>
#include <fstream>
#include <istream>

bool found;

void laserCallback(sensor_msgs::LaserScan scan)
{	
	laser_geometry::LaserProjection scan_converter;
	tf::TransformListener listener;

	ROS_INFO_STREAM("[LaserToCSV] Waiting a second to ensure listener buffer gets filled with frames...");
	ros::Duration(1.0).sleep();

	// Transform LaserScan into XYZ(I) PointCloud
	sensor_msgs::PointCloud2 cloud_msg;
	scan_converter.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_msg, listener);
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(cloud_msg, *pointcloud);

	std::ofstream output_file;
	std::string file_name = "scan_time" + std::to_string(ros::Time::now().toSec()) + ".csv";
	output_file.open (file_name);
	for(int i=0; i<pointcloud->points.size(); i++)
	{
		output_file << pointcloud->points[i].x << ", " << pointcloud->points[i].y << ", " << pointcloud->points[i].z;
		if(scan.intensities.size() == scan.ranges.size())
			if(scan.intensities[i] != 0)
				output_file << ", " << pointcloud->points[i].intensity;
		output_file << "\n";
	}
	output_file.close();

	found = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_stitcher");

//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//    ros::console::notifyLoggerLevelsChanged();

	ros::NodeHandle nh;

	found = false;

	std::string laser_topic;
	nh.param<std::string>("laser_to_csv/laser_topic", laser_topic, "hokuyo_scan");
	ROS_INFO_STREAM("Subscribing to LaserScan on topic " << laser_topic);	
	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, laserCallback);

	while(ros::ok() && !found)
	{
		ros::spinOnce();
		ros::Duration(0.4).sleep();
	}

}

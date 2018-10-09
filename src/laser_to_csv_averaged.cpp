
// ROS Stuff
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// Std Stuff
#include <iostream>
#include <fstream>
#include <istream>

bool use_intensity_;
int scans_caught_;
int scans_desired_;
float voxel_leaf_size_;
float voxel_leaf_size_depth_;
pcl::PointCloud<pcl::PointXYZI> voxelized_scan_;
pcl::PointCloud<pcl::PointXYZI>::Ptr cumulative_scan_ptr_(new pcl::PointCloud<pcl::PointXYZI>);

void laserCallback(sensor_msgs::LaserScan scan)
{	
	laser_geometry::LaserProjection scan_converter;
	tf::TransformListener listener;
	ROS_INFO_STREAM("[LaserToCSV] Caught " << scans_caught_ << "th of " << scans_desired_ << " planar scans.");

	bool use_intensity_ = ( (scan.intensities.size() == scan.ranges.size()) );
	// Transform LaserScan into XYZ(I) PointCloud
	sensor_msgs::PointCloud2 cloud_msg;
	scan_converter.transformLaserScanToPointCloud(scan.header.frame_id, scan, cloud_msg, listener);
	pcl::PointCloud<pcl::PointXYZI>::Ptr new_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(cloud_msg, *new_pointcloud);

	for(int i=0; i<new_pointcloud->points.size(); i++)
	{
		cumulative_scan_ptr_->points.push_back(new_pointcloud->points[i]);
	}

	scans_caught_++;
}

void printScan()
{
	ROS_INFO_STREAM("[LaserToCSV] Finished catching all scans! Current cloud size is " << cumulative_scan_ptr_->points.size() << " Performing voxelization...");
	// Create the Filtering object: downsample the dataset
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(cumulative_scan_ptr_);
	vg.setLeafSize(voxel_leaf_size_depth_, voxel_leaf_size_, voxel_leaf_size_);
	// Apply Filter and return Voxelized Data
	//pcl::PointCloud<pcl::PointXYZI> voxelized_scan_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
	voxelized_scan_.points.clear();
	vg.filter(voxelized_scan_);
	ROS_INFO_STREAM("[LaserToCSV] Voxelized with leaf sizes " << voxel_leaf_size_ << " and " << voxel_leaf_size_depth_ << ". New cloud size is " << voxelized_scan_.points.size());

	std::ofstream output_file;
	std::string file_name = "scan_time" + std::to_string(ros::Time::now().toSec()) + ".csv";
	output_file.open (file_name);
	for(int i=0; i<voxelized_scan_.points.size(); i++)
	{
		output_file << voxelized_scan_.points[i].x << ", " << voxelized_scan_.points[i].y << ", " << voxelized_scan_.points[i].z;
		if(use_intensity_)
			output_file << ", " << voxelized_scan_.points[i].intensity;
		output_file << "\n";
	}
	output_file.close();
	ROS_INFO_STREAM("[LaserToCSV] Finished printing output file with name " << file_name << ".csv");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_to_csv_averaged");

//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//    ros::console::notifyLoggerLevelsChanged();

	ros::NodeHandle nh;
	ros::Publisher output_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("voxelized_planar_cloud", 1);
	
	std::string laser_topic;
	nh.param<std::string>("laser_to_csv/laser_topic", laser_topic, "hokuyo_scan");
	ROS_INFO_STREAM("Subscribing to LaserScan on topic " << laser_topic);	
	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, laserCallback);

	nh.param<int>("laser_to_csv/scans_desired", scans_desired_, 20);
	nh.param<float>("laser_to_csv/voxel_leaf_size", voxel_leaf_size_, 0.002);
	nh.param<float>("laser_to_csv/voxel_leaf_size_depth", voxel_leaf_size_depth_, 0.2);
	
	ROS_INFO_STREAM("[LaserToCSV] Waiting a second to ensure listener buffer gets filled with frames...");
	ros::Duration(1.0).sleep();

	while(ros::ok() && scans_caught_ < scans_desired_)
	{
		ros::spinOnce();
		ros::Duration(0.4).sleep();
	}

	printScan();

	sensor_msgs::PointCloud2 output_cloud;
	pcl::toROSMsg(voxelized_scan_, output_cloud);
	output_cloud.header.frame_id = "hokuyo_lidar";
	output_cloud.header.stamp = ros::Time::now();
	output_cloud_pub.publish(output_cloud);
	ROS_INFO_STREAM("size: " << voxelized_scan_.points.size() << " header: " << output_cloud.header);
	ros::Duration(2.0).sleep();
}

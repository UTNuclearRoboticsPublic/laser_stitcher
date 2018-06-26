
#ifndef LASER_STITCHER_H
#define LASER_STITCHER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <rosbag/bag.h>

#include "laser_stitcher/stitched_clouds.h"

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sstream>


class LaserStitcher
{
public: 
	LaserStitcher();
private:
	ros::NodeHandle nh_;
	ros::Subscriber scan_sub_;
	ros::Subscriber finish_sub_;
	ros::Subscriber reset_sub_;	
	ros::Publisher planar_cloud_pub_;			// This publishes a single plane every time an input laser_scan is received
	ros::Publisher partial_scan_pub_;			// This publishes every time a plane is added (or some subset of those times)
	ros::Publisher full_scan_pub_;				// This publishes only once per scan

	bool incremental_update_;

	bool is_running_;
	bool save_data_;
	bool publish_after_updating_;
	std::string bag_name_;
	std::string output_frame_;
	sensor_msgs::PointCloud2 current_scan_;
	float wait_time_if_not_running_;
	bool scale_intensities_;
	float intensity_scale_exp_;

	laser_geometry::LaserProjection scan_converter_;
	tf::TransformListener listener_;
	tf::StampedTransform last_transform_;

	void laserCallback(sensor_msgs::LaserScan scan_in);
	void setScanningState(const std_msgs::Bool::ConstPtr& shut_down);
	void resetCloud(const std_msgs::Bool::ConstPtr& placeholder);
	bool lidarHasMoved(std::string);
};

#endif //LASER_STITCHER_H
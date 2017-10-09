
#ifndef LASER_STITCHER_H
#define LASER_STITCHER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>

#include <pcl_ros/transforms.h>


class LaserStitcher
{
public: 
	LaserStitcher();
private:
	ros::NodeHandle nh_;
	ros::Subscriber scan_sub_;
	ros::Subscriber finish_sub_;
	ros::Subscriber reset_sub_;
	ros::Publisher cloud_pub_;

	bool is_running_;
	bool save_data_;
	bool publish_after_updating_;
	std::string bag_name_;
	std::string target_frame_;
	sensor_msgs::PointCloud2 summed_pointcloud_;
	float sleepy_time_;
	int publishing_throttle_;
	int throttle_index_;

	bool should_check_movement_;
	float distance_threshold_;
	float angle_threshold_;

	laser_geometry::LaserProjection scan_converter_;
	tf::TransformListener listener_;
	tf::StampedTransform last_transform_;

	//pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
	bool should_voxelize_;
	bool should_throttle_voxel_;
	int voxel_throttle_counter_;
	int voxel_throttle_;
	float leaf_size_;

	bool reset_cloud_when_stopped_;

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void setScanningState(const std_msgs::Bool::ConstPtr& shut_down);
	void resetCloud(const std_msgs::Bool::ConstPtr& placeholder);
	bool lidarHasMoved(std::string);
};

#endif //LASER_STITCHER_H
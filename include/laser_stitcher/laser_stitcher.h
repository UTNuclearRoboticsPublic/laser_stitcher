
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
#include <pointcloud_processing_server/pointcloud_process.h>
#include <pointcloud_processing_server/pointcloud_task_creation.h>

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

	class StitchedCloud
	{
	public:
		// Direct Output Stuff
		std::string cloud_name_;
		
		// Publishing
		bool should_pub_;
		bool incremental_update_;
		ros::Publisher cloud_pub_;
		bool throttle_publish_;
		int publishing_throttle_counter_;
		int publishing_throttle_max_;
		bool should_save_;

		// Postprocessing
		bool should_postprocess_;
		bool throttle_postprocess_;
		int postprocess_throttle_counter_;
		int postprocess_throttle_max_;
		pointcloud_processing_server::pointcloud_process postprocess_;
		
		// Updating Options
		bool retain_after_scan_;

		// Initial Transform
		bool should_transform_first_;
		std::string first_transform_frame_;
		// Clipping
		bool should_clip_;
		float clipping_dimensions_[6];
		// Voxelization
		bool should_voxelize_;
		float voxel_leaf_size_;
		bool throttle_voxelization_;
		int voxel_throttle_max_;
		int voxel_throttle_counter_;
		// Final Transform
		bool should_do_final_transform_;
		std::string final_transform_frame_;

		sensor_msgs::PointCloud2 cloud_;

		void initializeFromYAML(std::string yaml_file_name);
		void increment(sensor_msgs::PointCloud2 new_cloud);
		void output();
	};

	std::vector<StitchedCloud> output_settings_;

	bool is_running_;
	bool save_data_;
	bool publish_after_updating_;
	std::string bag_name_;
	std::string target_frame_;
	sensor_msgs::PointCloud2 summed_pointcloud_;
	float sleepy_time_;

	ros::ServiceClient postprocessor_;

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

	bool buildSettings(std::string yaml_file_name);
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void setScanningState(const std_msgs::Bool::ConstPtr& shut_down);
	void resetCloud(const std_msgs::Bool::ConstPtr& placeholder);
	bool lidarHasMoved(std::string);
};

#endif //LASER_STITCHER_H
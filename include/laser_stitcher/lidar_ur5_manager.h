
#ifndef LIDAR_UR5_MANAGER
#define LIDAR_UR5_MANAGER

#include <ros/ros.h>
#include "laser_stitcher/stationary_scan.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class LIDARUR5Manager
{
public: 
public: 
	LIDARUR5Manager();
	bool stationaryScan(laser_stitcher::stationary_scan::Request &req, laser_stitcher::stationary_scan::Response &res);
	void angleCallback(const std_msgs::Float32::ConstPtr& angle);
private:
	float max_angle_;
	float min_angle_;
	float angle_step_;
	float servo_angle_;
	float wait_time_;

	std::string lidar_frame_name_;
	std::string parent_frame_name_;

	ros::NodeHandle nh_;
	ros::Publisher ur5_script_pub_;
	ros::Publisher scanning_state_pub_;
	ros::Subscriber angle_input_;
};

#endif //LIDAR_UR5_MANAGER
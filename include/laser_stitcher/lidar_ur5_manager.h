
#ifndef LIDAR_UR5_MANAGER
#define LIDAR_UR5_MANAGER

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "laser_stitcher/stationary_scan.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

class LIDARUR5Manager
{
public: 
	LIDARUR5Manager();
	bool stationaryScan(laser_stitcher::stationary_scan::Request &req, laser_stitcher::stationary_scan::Response &res);
	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states);
	bool updateJoints();
	void getOutputCloud(std::string output_cloud_topic);
	void cloudCallback(const sensor_msgs::PointCloud2 output_cloud);

private:
	float max_angle_;
	float min_angle_;
	float wrist_speed_;
	float wait_time_;
	float callbacks_received_;
	float correct_callbacks_;
	float wrist_angle_;
	sensor_msgs::JointState joint_states_;

	bool fixed_start_state_;
	std::vector<float> start_state_;

	std::string lidar_frame_name_;
	std::string parent_frame_name_;

	ros::NodeHandle nh_;
	ros::NodeHandle joint_state_nh_;
	ros::CallbackQueue joint_state_queue_;
	ros::Publisher urscript_pub_;
	ros::Publisher scanning_state_pub_;
	ros::Subscriber joint_state_sub_;
	ros::Subscriber output_cloud_sub_;

	sensor_msgs::PointCloud2 output_cloud_;
	bool still_need_cloud_;
};

#endif //LIDAR_UR5_MANAGER
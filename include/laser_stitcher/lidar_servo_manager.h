
#ifndef LIDAR_SERVO_MANAGER
#define LIDAR_SERVO_MANAGER

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "laser_stitcher/stationary_scan.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

class LIDARServoManager
{
public: 
	LIDARServoManager();
	bool stationaryScan(laser_stitcher::stationary_scan::Request &req, laser_stitcher::stationary_scan::Response &res);
	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states);
	bool updateJoints();
	void getOutputCloud();
	void outputCallback(const sensor_msgs::PointCloud2 final_cloud);

private:
	float max_angle_;
	float min_angle_;
	float pan_speed_;
	float pan_speed_returning_;
	float wait_time_;
	float callbacks_received_;
	float correct_callbacks_;
	float pan_angle_;
	std::vector<std::string> joint_names_;
	sensor_msgs::JointState joint_states_;

	bool scan_while_returning_;
	bool fixed_start_state_;
	std::vector<float> start_state_;

	std::string lidar_frame_name_;
	std::string parent_frame_name_;
	tf2_ros::TransformBroadcaster lidar_frame_broadcaster;

	ros::NodeHandle nh_;
	ros::NodeHandle joint_state_nh_;
	ros::CallbackQueue joint_state_queue_;
	ros::Publisher servo_pub_;
	ros::Publisher scanning_state_pub_;
	ros::Subscriber joint_state_sub_;
	ros::Subscriber output_cloud_sub_;

	std::string output_cloud_topic_;
	sensor_msgs::PointCloud2 final_cloud_;
	bool still_need_cloud_;
};

#endif //LIDAR_UR5_MANAGER
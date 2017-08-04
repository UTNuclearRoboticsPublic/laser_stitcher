
#include "laser_stitcher/lidar_servo_manager.h"

LIDARUR5Manager::LIDARUR5Manager()
{
	std::string angle_command_topic;
	nh_.param<std::string>("LIDARUR5Manager/angle_command_topic", angle_command_topic, "lidar_servo_manager/angle_command_topic");
	ur5script_pub_ = nh_.advertise<std_msgs::String>(angle_command_topic, 1);
	std::string scanning_state_topic;
	nh_.param<std::string>("LIDARUR5Manager/scanning_state_topic", scanning_state_topic, "laser_stitcher/scanning_state");
	scanning_state_pub_ = nh_.advertise<std_msgs::Bool>(scanning_state_topic, 1);
	std::string angle_topic_in;
	nh_.param<std::string>("LIDARUR5Manager/angle_topic_in", angle_topic_in, "laser_stitcher/angle_topic_in");
	angle_input_ = nh_.subscribe<std_msgs::Float32>("laser_topic", 20, &LIDARUR5Manager::angleCallback, this);
	std::string service_name;
	nh_.param<std::string>("LIDARUR5Manager/service_name", service_name, "laser_stitcher/stationary_scan");
	ros::ServiceServer object_pose_server = nh_.advertiseService(service_name, &LIDARUR5Manager::stationaryScan, this);

	angle_step_ = 0.05; 		// radians
	wait_time_ = 0.028; 		// seconds
	lidar_frame_name_ = "3d_lidar_plane";
	parent_frame_name_ = "base_link";

	ros::spin();
}

bool LIDARUR5Manager::stationaryScan(laser_stitcher::stationary_scan::Request &req, laser_stitcher::stationary_scan::Response &res)
{
	std_msgs::Bool scanning_state;
	scanning_state.data = true;
	scanning_state_pub_.publish(scanning_state);

	min_angle_ = req.min_angle;
	max_angle_ = req.max_angle;	

	tf2_ros::TransformBroadcaster broadcaster;
	geometry_msgs::TransformStamped transform_from_parent;

	transform_from_parent.header.frame_id = parent_frame_name_;
	transform_from_parent.child_frame_id = lidar_frame_name_;
	transform_from_parent.transform.translation.x = 0.0;
	transform_from_parent.transform.translation.y = 0.0;
	transform_from_parent.transform.translation.z = 0.5;
	transform_from_parent.transform.rotation.x = 0.0;
	transform_from_parent.transform.rotation.y = 0.0;

	std_msgs::Float32 angle_msg;

	if(!req.external_angle_sensing) 
		servo_angle_ = min_angle_;

	ROS_DEBUG_STREAM("Servo Angle: " << servo_angle_ << " Max Angle: " << max_angle_);

	// Turn counterclockwise while scanning:
	while(servo_angle_ + angle_step_	 < max_angle_)
	{
		angle_msg.data = servo_angle_ + angle_step_;
		ur5script_pub_.publish(angle_msg);

		if(req.external_angle_sensing)
			while(servo_angle_ < servo_angle_ + angle_step_);
		else
		{
			ros::Duration(wait_time_).sleep();
			servo_angle_ += angle_step_;
		}

		transform_from_parent.header.stamp = ros::Time::now();
		transform_from_parent.transform.rotation.z = sin(servo_angle_);
		transform_from_parent.transform.rotation.w = cos(servo_angle_);

		broadcaster.sendTransform(transform_from_parent);
		ROS_DEBUG_STREAM("[LIDARUR5Manager] Sent a servo command... new angle: " << servo_angle_ << "; quat info: " << transform_from_parent.transform.rotation.z << " " << transform_from_parent.transform.rotation.w << ".");
	}

	scanning_state.data = false;
	scanning_state_pub_.publish(scanning_state); 			// Shut down laser stitcher process

	//Reset to clockwise, 'ready' position
	angle_msg.data = min_angle_;
	ur5script_pub_.publish(angle_msg);
	transform_from_parent.header.stamp = ros::Time::now();
	transform_from_parent.transform.rotation.z = sin(servo_angle_);
	transform_from_parent.transform.rotation.w = cos(servo_angle_);
	broadcaster.sendTransform(transform_from_parent);

	return true;
}

void LIDARUR5Manager::angleCallback(const std_msgs::Float32::ConstPtr& angle)
{
	servo_angle_ = angle->data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_stitcher");

if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

	LIDARUR5Manager servo_manager;
}

#include "laser_stitcher/lidar_ur5_manager.h"

LIDARUR5Manager::LIDARUR5Manager()
{
	// Topic for UR5 Command output (specify pos/vel goals for rotation)
	std::string urscript_command_topic;
	nh_.param<std::string>("lidar_ur5_manager/angle_command_topic", urscript_command_topic, "left_ur5_controller/left_ur5_URScript");
	urscript_pub_ = nh_.advertise<std_msgs::String>(urscript_command_topic, 1);
		
	// Topic for UR5 State input (check to see whether we've reached each goal)
	std::string scanning_state_topic;
	nh_.param<std::string>("laser_stitcher/scanning_state_topic", scanning_state_topic, "laser_stitcher/scanning_state");
	scanning_state_pub_ = nh_.advertise<std_msgs::Bool>(scanning_state_topic, 1);  

	// Topic for output cloud - don't know if we actually need this? After the change to output topic architecture.  
	nh_.param<std::string>("laser_stitcher/partial_scan_topic", output_cloud_topic_, "laser_stitcher/partial_cloud");
	
	// Subscriber to UR5 State - sensor_msgs/JointState 
	//   Separate the JOINT_STATE subscriber so it can be spun within the STATIONARY_SCAN service call
	joint_state_nh_.setCallbackQueue(&joint_state_queue_);
	std::string jointstate_topic;
	nh_.param<std::string>("laser_stitcher/jointstate_topic", jointstate_topic, "/joint_states");
	joint_state_sub_ = joint_state_nh_.subscribe<sensor_msgs::JointState>(jointstate_topic, 20, &LIDARUR5Manager::jointStateCallback, this);  

	// Service for UR5 Controller
	std::string service_name;
	nh_.param<std::string>("lidar_ur5_manager/service_name", service_name, "laser_stitcher/stationary_scan");
	ros::ServiceServer scanning_server = nh_.advertiseService(service_name, &LIDARUR5Manager::stationaryScan, this);

	// Get Start Pose
	//   Is movement to a start pose requested?
	nh_.param<bool>("lidar_ur5_manager/fixed_start_pose", fixed_start_state_, false);
	//   If so, what is that start pose? 
	if(fixed_start_state_)
		if(!nh_.getParam("lidar_ur5_manager/start_pose", start_state_))
		{
			ROS_WARN_STREAM("[LIDARUR5Manager] Told to use a fixed_start_pose, but none exists on parameter server. Defaulting to NOT use a fixed start pose.");
			fixed_start_state_ = false;
		}

	// Procedural parameters... 
	nh_.param<float>("lidar_ur5_manager/wrist_speed", wrist_speed_, 0.3);
	nh_.param<float>("lidar_ur5_manager/wrist_speed_returning", wrist_speed_returning_, wrist_speed_);
	nh_.param<bool>("lidar_ur5_manager/scan_while_returning", scan_while_returning_, false);
	wait_time_ = 0.01; 		// seconds
	lidar_frame_name_ = "3d_lidar_plane";
	parent_frame_name_ = "base_link";

	// Commence operation!
	ROS_INFO_STREAM("[LIDARUR5Manager] Manager online.");
	ros::spin(); 
}

// Actual service call for the entire laser_stitcher system
//   Most parameters of behavior are specified in yaml files --> parameter server, though
//   Only input parameters are min/max angle 
//   Ouput parameters are clouds and cloud names 
bool LIDARUR5Manager::stationaryScan(laser_stitcher::stationary_scan::Request &req, laser_stitcher::stationary_scan::Response &res)
{ 
	ROS_INFO_STREAM("[LIDARUR5Manager] Received stationary_scan service callback. Finding jointstate...");
	int joint_update_attempts = 0;
	int max_joint_update_attempts = 5;
	while(!this->updateJoints() && joint_update_attempts < max_joint_update_attempts)
	{
		ROS_ERROR_STREAM("[LIDARUR5Manager] Failed to update joints " << joint_update_attempts << " times.");
		joint_update_attempts++;
		ros::Duration(0.05).sleep();
	}
	if(joint_update_attempts >= max_joint_update_attempts)
	{
		ROS_ERROR_STREAM("[LIDARUR5Manager] Failed to update joints too many times... exiting this service call.");
		return false;
	}

	// Move wrist to start position:
	char start_cmd[200];
	if(fixed_start_state_)
	{
		sprintf(start_cmd, "movej([%f, %f, %f, %f, %f, %f], 0.8, 0.1)", start_state_[0], start_state_[1], start_state_[2], start_state_[3], start_state_[4], start_state_[5]);	
		ros::Duration(3.0).sleep();
		ROS_INFO_STREAM("[LIDARUR5Manager] Starting command:  " << start_cmd);
		std_msgs::String start_msg;
		start_msg.data = start_cmd;
		urscript_pub_.publish(start_msg);
	}

	min_angle_ = req.min_angle;
	max_angle_ = req.max_angle;	

	std_msgs::Bool scanning_state;
	if(scan_while_returning_)
	{
		scanning_state.data = true;
		scanning_state_pub_.publish(scanning_state);
		ROS_INFO_STREAM("[LIDARUR5Manager] Sending scanning start message.");
	}
	ros::Duration(0.15).sleep();

	// Turn counterclockwise prior to scanning - get to 'max_angle' starting point:
	ROS_INFO_STREAM("[LIDARUR5Manager] Moving wrist towards point " << max_angle_ << " at speed " << wrist_speed_returning_);
	while(wrist_angle_ < max_angle_ && ros::ok())
	{
		char counterclockwise_cmd[200];
		sprintf(counterclockwise_cmd, "speedj([0.0, 0.0, 0.0, 0.0, 0.0, %f], 0.8, 0.1)", wrist_speed_returning_);
		ROS_DEBUG_STREAM("[LIDARUR5Manager] Counterclockwise command:  " << counterclockwise_cmd);
		std_msgs::String counterclockwise_msg;
		counterclockwise_msg.data = counterclockwise_cmd;
		urscript_pub_.publish(counterclockwise_msg);
		
		int joint_update_attempts = 0;
		int max_joint_update_attempts = 5;
		while(!this->updateJoints() && joint_update_attempts < max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[LIDARUR5Manager] Failed to update joints " << joint_update_attempts << " times.");
			joint_update_attempts++;
			ros::Duration(0.05).sleep();
		}
		if(joint_update_attempts >= max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[LIDARUR5Manager] Failed to update joints too many times... exiting this service call.");
			return false;
		}

		ROS_DEBUG_STREAM("[LIDARUR5Manager] Sent a counterclockwise motion command. Current position: " << wrist_angle_);
	}
	
	if(!scan_while_returning_)
	{
		scanning_state.data = true;
		scanning_state_pub_.publish(scanning_state);
		ROS_INFO_STREAM("[LIDARUR5Manager] Sending scanning start message.");
	}
	ros::Duration(0.15).sleep();

	// Turn clockwise while scanning - get to 'min_angle' stopping point:
	ROS_INFO_STREAM("[LIDARUR5Manager] Moving wrist towards point " << min_angle_ << " at speed " << wrist_speed_);
	while(wrist_angle_ > min_angle_ && ros::ok())
	{
		char clockwise_cmd[200];
		sprintf(clockwise_cmd, "speedj([0.0, 0.0, 0.0, 0.0, 0.0, %f], 0.4, 0.1)", -wrist_speed_);
		ROS_DEBUG_STREAM("[LIDARUR5Manager] Clockwise command:  " << clockwise_cmd << " Min Angle: " << min_angle_ << " Current Angle: " << wrist_angle_ << " Current Command Speed: " << wrist_speed_);
		std_msgs::String clockwise_msg;
		clockwise_msg.data = clockwise_cmd;
		urscript_pub_.publish(clockwise_msg);
		
		int joint_update_attempts = 0;
		int max_joint_update_attempts = 5;
		while(!this->updateJoints() && joint_update_attempts < max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[LIDARUR5Manager] Failed to update joints " << joint_update_attempts << " times.");
			joint_update_attempts++;
			ros::Duration(0.05).sleep();
		}
		if(joint_update_attempts >= max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[LIDARUR5Manager] Failed to update joints too many times... exiting this service call.");
			return false;
		}

		ROS_DEBUG_STREAM("[LIDARUR5Manager] Sent a counterclockwise motion command. Current position: " << wrist_angle_);
	}

	ROS_INFO_STREAM("[LIDARUR5Manager] Finished one scanning routine, capturing output clouds.");
	getOutputCloud();
	res.output_cloud = final_cloud_;
	
	scanning_state.data = false;
	ROS_INFO_STREAM("[LIDARUR5Manager] About to publish turn scanning routine off message");
	scanning_state_pub_.publish(scanning_state); 			// Shut down laser stitcher process


	return true;  
}

// Output Cloud Gatherer
//   This is run at the end of each routine --> populates output of service object from stationaryScan function
void LIDARUR5Manager::getOutputCloud()
{
	still_need_cloud_ = true;
	output_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(output_cloud_topic_, 1, &LIDARUR5Manager::outputCallback, this);
	while(still_need_cloud_ && ros::ok())
	{
		ros::spinOnce();
		ros::Duration(wait_time_).sleep();
	}
}

// Laser_Stitcher Cloud Output Callback
//   Used internally within getOutputCloud
//   Just catches the final output cloud from laser_stitcher and assigns it to stationaryScan service object results
//   Then tells the owning class that it's done and we can end
void LIDARUR5Manager::outputCallback(const sensor_msgs::PointCloud2 final_cloud)
{
	if(still_need_cloud_)
	{
		final_cloud_ = final_cloud;
		still_need_cloud_ = false;
	}
	output_cloud_sub_.shutdown();
}

// Joint State Gatherer
//   Tries to get jointstate of UR5. If it can't, continues trying. If it fails too many times, throws an error and exits
bool LIDARUR5Manager::updateJoints()
{
	callbacks_received_ = 0;
	correct_callbacks_ = 0;
	ros::Duration time_elapsed;
	ros::Time time_started = ros::Time::now();
	while(correct_callbacks_ < 1 && callbacks_received_ < 100 && time_elapsed < ros::Duration(2.0) && ros::ok())
	{
		ros::Duration(wait_time_).sleep();
		joint_state_queue_.callAvailable(ros::WallDuration());
		time_elapsed = ros::Time::now() - time_started;
	}
	if(correct_callbacks_ > 0)
	{
		ROS_DEBUG_STREAM("[LIDARUR5Manager] Successfully updated joint states. New states:");
		ROS_DEBUG_STREAM(joint_states_);
		ROS_DEBUG_STREAM("[LIDARUR5Manager] Correct callbacks: " << correct_callbacks_ << " Total callbacks: " << callbacks_received_);	
		return true;
	}
	else 
	{
		ROS_ERROR_STREAM("[LIDARUR5Manager] Joint update failure. " << callbacks_received_ << " total callbacks received, and no callbacks for the correct jointspace, in " << time_elapsed << " seconds.");
		return false;
	}
}

// Joint State Callback
//   Run between successive output commands to get input - current state of UR5
//   Increments a class member counter of number of attempts to get jointstate
//   If we fail enough times, higher level service will decide something isn't set up right, throw an error, and exit
void LIDARUR5Manager::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
	callbacks_received_++;
	if(joint_states->name[0] == "left_ur5_shoulder_pan_joint")
	{
		joint_states_ = *joint_states;
		wrist_angle_ = joint_states->position[5];
		correct_callbacks_++;
	}
	std::string name = joint_states->name[0];
	ROS_DEBUG_STREAM(name << " " << callbacks_received_ << " " << correct_callbacks_);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_ur5_manager");

//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
  //  ros::console::notifyLoggerLevelsChanged();

	LIDARUR5Manager servo_manager;
}
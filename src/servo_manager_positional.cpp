
#include "laser_stitcher/servo_manager_positional.h"

ServoManagerPositional::ServoManagerPositional()
{
	ros::Duration(0.5).sleep();

	// Topic for Servo Command output (specify pos/vel goals for rotation)
	std::string angle_command_topic;
	nh_.param<std::string>("servo_manager_positional/angle_command_topic", angle_command_topic, "/ptu/cmd");
	servo_pub_ = nh_.advertise<std_msgs::Float64>(angle_command_topic, 1);

	// For now just hardcoding these...
	deadband_ = 0.01;
	overshoot_ = 0.03;
	
	// Topic for Servo State input (check to see whether we've reached each goal)
	std::string scanning_state_topic;
	nh_.param<std::string>("laser_stitcher/scanning_state_topic", scanning_state_topic, "laser_stitcher/scanning_state");
	scanning_state_pub_ = nh_.advertise<std_msgs::Bool>(scanning_state_topic, 1);  
	// Get Joint Names for input/output servo commands
	if( !nh_.param<std::string>("servo_manager_positional/pan_joint", pan_joint_, "servo_pan_joint") )
		ROS_WARN_STREAM("[ServoManagerPositional] Warning - could not get name of pan joint. Defaulting to servo_pan_joint.");
	
	// Subscriber to Servo State - sensor_msgs/JointState 
	//   Separate the JOINT_STATE subscriber so it can be spun within the STATIONARY_SCAN service call
	joint_state_nh_.setCallbackQueue(&joint_state_queue_);
	std::string jointstate_topic;
	nh_.param<std::string>("servo_manager_positional/jointstate_topic", jointstate_topic, "/joint_states");
	joint_state_sub_ = joint_state_nh_.subscribe<sensor_msgs::JointState>(jointstate_topic, 20, &ServoManagerPositional::jointStateCallback, this); 

	// TF Frames
	//   Need to manually set the frame for the LIDAR, unless the servo used automatically generates an output frame in ROS
	//   Also specify the world frame to be worked in 
	nh_.param<std::string>("servo_manager_positional/lidar_frame", lidar_frame_name_, "hokuyo_lidar");
	parent_frame_name_ = "map";

	// Service for Servo Controller
	std::string service_name;
	nh_.param<std::string>("servo_manager_positional/service_name", service_name, "laser_stitcher/stationary_scan");
	ros::ServiceServer scanning_server = nh_.advertiseService(service_name, &ServoManagerPositional::stationaryScan, this);

	// Get Start Pose
	//   Is movement to a start pose requested?
	nh_.param<bool>("servo_manager_positional/fixed_start_pose", fixed_start_state_, false);
	//   If so, what is that start pose? 
	if(fixed_start_state_)
		if(!nh_.getParam("servo_manager_positional/start_pose", start_state_))
		{
			ROS_WARN_STREAM("[ServoManagerPositional] Told to use a fixed_start_pose, but none exists on parameter server. Defaulting to NOT use a fixed start pose.");
			fixed_start_state_ = false;
		}
	
	// Topic for output cloud - don't know if we actually need this? After the change to output topic architecture.  
	nh_.param<std::string>("laser_stitcher/partial_scan_topic", output_cloud_topic_, "laser_stitcher/partial_cloud");

	// Procedural parameters... 
	nh_.param<float>("servo_manager_positional/pan_speed", pan_speed_, 0.3);
	nh_.param<float>("servo_manager_positional/pan_speed_returning", pan_speed_returning_, pan_speed_);
	nh_.param<bool>("servo_manager_positional/scan_while_returning", scan_while_returning_, false);
	nh_.param<float>("servo_manager_positional/wait_time", wait_time_, 0.03); 		// seconds


	// Commence operation!
	ROS_INFO_STREAM("[ServoManagerPositional] Manager online.");
	ros::spin(); 
}

// Actual service call for the entire laser_stitcher system
//   Most parameters of behavior are specified in yaml files --> parameter server, though
//   Only input parameters are min/max angle 
//   Ouput parameters are clouds and cloud names  
bool ServoManagerPositional::stationaryScan(laser_stitcher::stationary_scan::Request &req, laser_stitcher::stationary_scan::Response &res)
{ 
	// Enforce max speed on joints...
	nh_.setParam("/arbotix/joints/" + pan_joint_ + "/max_speed", pan_speed_);

	// Declare the positional command to the servos 
	std_msgs::Float64 pos_cmd;
	pos_cmd.data = 0.0;	

	ROS_INFO_STREAM("[ServoManagerPositional] Received stationary_scan service callback. Finding jointstate...");
	int joint_update_attempts = 0;
	int max_joint_update_attempts = 5;
	while(!this->updateJoints() && joint_update_attempts < max_joint_update_attempts)
	{
		ROS_ERROR_STREAM("[ServoManagerPositional] Failed to update joints " << joint_update_attempts << " times.");
		joint_update_attempts++;
		ros::Duration(0.05).sleep();
	}
	if(joint_update_attempts >= max_joint_update_attempts)
	{
		ROS_ERROR_STREAM("[ServoManagerPositional] Failed to update joints too many times... exiting this service call.");
		return false;
	}

	// Move pan joint to start position:
	if(fixed_start_state_)
	{
		// Currently set up only to reset the state of one servo in the PTU... might come back and add second if we use two
		pos_cmd.data = start_state_[0];
		ROS_INFO_STREAM("[ServoManagerPositional] Moving to initial position:  " << pos_cmd);
		ros::Duration(3.0).sleep();
		servo_pub_.publish(pos_cmd);
	}

	min_angle_ = req.min_angle;
	max_angle_ = req.max_angle;	

	std_msgs::Bool scanning_state;
	if(scan_while_returning_)
	{
		scanning_state.data = true;
		scanning_state_pub_.publish(scanning_state);
		ROS_INFO_STREAM("[ServoManagerPositional] Sending scanning start message.");
	}
	ros::Duration(0.15).sleep();

	// Turn counterclockwise prior to scanning - get to 'max_angle' starting point:
	ROS_INFO_STREAM("[ServoManagerPositional] Moving pan joint from " << pan_angle_ << " towards point " << max_angle_ << " at speed " << pan_speed_returning_);
	while(std::abs(pan_angle_ - max_angle_) > deadband_ && pan_angle_ < max_angle_ && ros::ok())
	{
		pos_cmd.data = max_angle_ + overshoot_;
		// Debugging output and send servo command
		ROS_DEBUG_STREAM("[ServoManagerPositional] Counterclockwise pan speed:  " << pan_speed_ << " Current Angle: " << pan_angle_ << " Max Angle: " << max_angle_);
		servo_pub_.publish(pos_cmd);
		
		int joint_update_attempts = 0;
		int max_joint_update_attempts = 5;
		while(!this->updateJoints() && joint_update_attempts < max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[ServoManagerPositional] Failed to update joints " << joint_update_attempts << " times.");
			joint_update_attempts++;
			ros::Duration(0.05).sleep();
		}
		if(joint_update_attempts >= max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[ServoManagerPositional] Failed to update joints too many times... exiting this service call.");
			return false;
		}

		ROS_DEBUG_STREAM("[ServoManagerPositional] Sent a counterclockwise motion command. Current position: " << pan_angle_);

	}
	
	if(!scan_while_returning_)
	{
		scanning_state.data = true;
		scanning_state_pub_.publish(scanning_state);
		ROS_INFO_STREAM("[ServoManagerPositional] Sending scanning start message.");
	}
	ros::Duration(0.15).sleep();

	// Turn clockwise while scanning - get to 'min_angle' stopping point:
	ROS_INFO_STREAM("[ServoManagerPositional] Moving pan joint from " << pan_angle_ << " towards point " << min_angle_ << " at speed " << pan_speed_);
	while(std::abs(pan_angle_ - min_angle_) > deadband_ && pan_angle_ > min_angle_ && ros::ok())
	{
		pos_cmd.data = min_angle_ - overshoot_;
		// Debugging output and send servo command
		ROS_DEBUG_STREAM("[ServoManagerPositional] Clockwise pan speed:  " << pan_speed_ << " Current Angle: " << pan_angle_ << " Min Angle: " << min_angle_);
		servo_pub_.publish(pos_cmd);
		
		int joint_update_attempts = 0;
		int max_joint_update_attempts = 5;
		while(!this->updateJoints() && joint_update_attempts < max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[ServoManagerPositional] Failed to update joints " << joint_update_attempts << " times.");
			joint_update_attempts++;
			ros::Duration(0.05).sleep();
		}
		if(joint_update_attempts >= max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[ServoManagerPositional] Failed to update joints too many times... exiting this service call.");
			return false;
		}

		ROS_DEBUG_STREAM("[ServoManagerPositional] Sent a counterclockwise motion command. Current position: " << pan_angle_);
	}

	ROS_INFO_STREAM("[ServoManagerPositional] Finished one scanning routine, capturing output clouds.");
	getOutputCloud();
	res.output_cloud = final_cloud_;

	scanning_state.data = false;
	ROS_INFO_STREAM("[ServoManagerPositional] About to publish message to turn scanning routine off!");
	scanning_state_pub_.publish(scanning_state); 			// Shut down laser stitcher process

	// Temporary, to ensure output message is published prior to this node dying...
	ros::Duration(1.0).sleep();

	return true;  
}

// Output Cloud Gatherer
//   This is run at the end of each routine --> populates output of service object from stationaryScan function
void ServoManagerPositional::getOutputCloud()
{
	still_need_cloud_ = true;
	output_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(output_cloud_topic_, 1, &ServoManagerPositional::outputCallback, this);
	while(still_need_cloud_ && ros::ok())
	{
		ros::spinOnce();
		ros::Duration(wait_time_).sleep();
	}
}

// Laser_Stitcher Cloud Output Callback
//   Used internally within getOutputCloud
//   Just catches each output cloud from laser_stitcher and assigns them to stationaryScan service object list
//   Then tells the owning class that it's done and we can end
void ServoManagerPositional::outputCallback(const sensor_msgs::PointCloud2 output_cloud)
{
	if(still_need_cloud_)
	{
		final_cloud_ = output_cloud;
		still_need_cloud_ = false;
	}
	output_cloud_sub_.shutdown();
}

// Joint State Gatherer
//   Tries to get jointstate of servo. If it can't, continues trying. If it fails too many times, throws an error and exits
bool ServoManagerPositional::updateJoints()
{
	callbacks_received_ = 0;
	correct_callbacks_ = 0;
	ros::Time time_started = ros::Time::now();
	ros::Duration time_elapsed = ros::Time::now() - time_started;
	while(correct_callbacks_ < 1 && callbacks_received_ < 100 && time_elapsed < ros::Duration(2.0) && ros::ok())
	{
		ros::Duration(wait_time_).sleep();
		joint_state_queue_.callAvailable(ros::WallDuration());
		time_elapsed = ros::Time::now() - time_started;
/*
		// If necessary...
		// Manually build the relevant frame for the LIDAR
		//   Build transform, set frames and time
	  	geometry_msgs::TransformStamped lidar_transform;
	  	lidar_transform.header.stamp = ros::Time::now();
	  	lidar_transform.header.frame_id = parent_frame_name_;
	  	lidar_transform.child_frame_id = lidar_frame_name_;
	  	//   Set Translation (fixed, here)
	  	lidar_transform.transform.translation.x = 0.0;
	  	lidar_transform.transform.translation.y = 0.0;
	  	lidar_transform.transform.translation.z = 1.0;
	  	//   Set Rotation (pan is dynamic)
	  	tf2::Quaternion rotation;
	  	rotation.setRPY(0.0, -1.5708, pan_angle_);
	  	lidar_transform.transform.rotation.x = rotation.x();
	  	lidar_transform.transform.rotation.y = rotation.y();
	  	lidar_transform.transform.rotation.z = rotation.z();
	  	lidar_transform.transform.rotation.w = rotation.w();
	  	//   Broadcast it!
	  	lidar_frame_broadcaster.sendTransform(lidar_transform);
	  	//ROS_DEBUG_STREAM("Published a transform at: " << lidar_transform);
*/	  	
	}
	if(correct_callbacks_ > 0)
	{
		ROS_DEBUG_STREAM("[ServoManagerPositional] Successfully updated joint states. New states:");

		ROS_DEBUG_STREAM("[ServoManagerPositional] Correct callbacks: " << correct_callbacks_ << " Total callbacks: " << callbacks_received_);	
		return true;
	}
	else 
	{
		ROS_ERROR_STREAM("[ServoManagerPositional] Joint update failure. " << callbacks_received_ << " total callbacks received, and no callbacks for the correct jointspace, in " << time_elapsed << " seconds.");
		return false;
	}
}

// Joint State Callback
//   Run between successive output commands to get input - current state of servo
//   Increments a class member counter of number of attempts to get jointstate
//   If we fail enough times, higher level service will decide something isn't set up right, throw an error, and exit
void ServoManagerPositional::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
	callbacks_received_++;
	bool found_joint = false;
	int i = 0;
	while(i<joint_states->name.size() && !found_joint)
	{
		std::string name = joint_states->name[i];
		ROS_DEBUG_STREAM("name: " << name << " received:" << callbacks_received_ << " correct: " << correct_callbacks_ << " current angle: " << pan_angle_);
		if(joint_states->name[i] == pan_joint_)
		{
			joint_states_ = *joint_states;
			pan_angle_ = joint_states->position[i];
			correct_callbacks_++;
			found_joint = true;
			break;
		}
		i++;
	}
	if(!found_joint)
	{
		ROS_WARN_STREAM_THROTTLE(0.5, "[ServoManagerPositional] Warning: got a jointstate, but name does not match expected. First two joint names in jointstate received were " << joint_states->name[0] << " and " << joint_states->name[1] << " but expected " << pan_joint_);
		i--;
	}
	else
		ROS_DEBUG_STREAM("[ServoManagerPositional] Received a correct jointstate, with name " << joint_states->name[i] << " position " << joint_states->position[i]);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "servo_manager_positional");

    // Uncomment the following to enable DEBUGGING output
    //if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    //  ros::console::notifyLoggerLevelsChanged();

	ServoManagerPositional servo_manager;
}
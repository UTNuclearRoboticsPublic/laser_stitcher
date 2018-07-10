
#include "laser_stitcher/lidar_servo_manager.h"

LIDARServoManager::LIDARServoManager()
{
	// Topic for Servo Command output (specify pos/vel goals for rotation)
	std::string angle_command_topic;
	nh_.param<std::string>("lidar_servo_manager/angle_command_topic", angle_command_topic, "/ptu/cmd");
	servo_pub_ = nh_.advertise<sensor_msgs::JointState>(angle_command_topic, 1);
	
	// Topic for Servo State input (check to see whether we've reached each goal)
	std::string scanning_state_topic;
	nh_.param<std::string>("laser_stitcher/scanning_state_topic", scanning_state_topic, "laser_stitcher/scanning_state");
	scanning_state_pub_ = nh_.advertise<std_msgs::Bool>(scanning_state_topic, 1);  
	// Get Joint Names for input/output servo commands
	if( !nh_.getParam("lidar_servo_manager/joint_names", joint_names_) );
	{
		joint_names_.push_back("ptu_pan");
		joint_names_.push_back("ptu_tilt");
		ROS_WARN_STREAM("[LIDARServoManager] Warning - could not get list of names for joints. Defaulting to Pan and Tilt.");
	}

	
	// Subscriber to Servo State - sensor_msgs/JointState 
	//   Separate the JOINT_STATE subscriber so it can be spun within the STATIONARY_SCAN service call
	joint_state_nh_.setCallbackQueue(&joint_state_queue_);
	std::string jointstate_topic;
	nh_.param<std::string>("lidar_servo_manager/jointstate_topic", jointstate_topic, "/joint_states");
	joint_state_sub_ = joint_state_nh_.subscribe<sensor_msgs::JointState>(jointstate_topic, 20, &LIDARServoManager::jointStateCallback, this); 

	// TF Frames
	//   Need to manually set the frame for the LIDAR, unless the servo used automatically generates an output frame in ROS
	//   Also specify the world frame to be worked in 
	nh_.param<std::string>("lidar_servo_manager/lidar_frame", lidar_frame_name_, "hokuyo_lidar");
	parent_frame_name_ = "map";

	// Service for Servo Controller
	std::string service_name;
	nh_.param<std::string>("lidar_servo_manager/service_name", service_name, "laser_stitcher/stationary_scan");
	ros::ServiceServer scanning_server = nh_.advertiseService(service_name, &LIDARServoManager::stationaryScan, this);

	// Get Start Pose
	//   Is movement to a start pose requested?
	nh_.param<bool>("lidar_servo_manager/fixed_start_pose", fixed_start_state_, false);
	//   If so, what is that start pose? 
	if(fixed_start_state_)
		if(!nh_.getParam("lidar_servo_manager/start_pose", start_state_))
		{
			ROS_WARN_STREAM("[LIDARServoManager] Told to use a fixed_start_pose, but none exists on parameter server. Defaulting to NOT use a fixed start pose.");
			fixed_start_state_ = false;
		}
	
	// Topic for output cloud - don't know if we actually need this? After the change to output topic architecture.  
	nh_.param<std::string>("lidar_servo_manager/output_cloud_topic", output_clouds_topic_, "laser_stitcher/output_cloud_list");

	// Procedural parameters... 
	nh_.param<float>("lidar_servo_manager/pan_speed", pan_speed_, 0.3);
	nh_.param<float>("lidar_servo_manager/pan_speed_returning", pan_speed_returning_, pan_speed_);
	nh_.param<bool>("lidar_servo_manager/scan_while_returning", scan_while_returning_, false);
	wait_time_ = 0.01; 		// seconds


	// Commence operation!
	ROS_INFO_STREAM("[LIDARServoManager] Manager online.");
	ros::spin(); 
}

// Actual service call for the entire laser_stitcher system
//   Most parameters of behavior are specified in yaml files --> parameter server, though
//   Only input parameters are min/max angle 
//   Ouput parameters are clouds and cloud names  
bool LIDARServoManager::stationaryScan(laser_stitcher::stationary_scan::Request &req, laser_stitcher::stationary_scan::Response &res)
{ 
	// Set up the velocity command output!    http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
	//   Just doing this here because most fields only need to be set once - only one velocity is changed between routines
	//   Set of different vectors, with each vectors ith value corresponding to the ith link in the joint chain
	//   Here, the pan-tilt used has two joints, but we only care about controlling one of them (the first, the pan)
	sensor_msgs::JointState vel_cmd;
	// ** Joint Names **
	//   Names of each joint (not sure whether these matter to the receiving program; might be important)
	vel_cmd.name.push_back(joint_names_[0]);
	vel_cmd.name.push_back(joint_names_[1]);
	// ** Header ** 
	//   I don't think the frame matters, but the time has to match amongst all joints
	vel_cmd.header.stamp = ros::Time::now(); 	
	// ** Position (rad) ** 
	//   Don't specify these, and the program won't force them. Leaving fields blank is allowable as long as they're blank for all joints
	//   For now, we're only commanding velocity, and checking position between scans - just runs until desired position is reached
	//   Might consider switching to commanding a target position with a fixed velocity (an option, and might work better/be cleaner than this...)
	//   vel_cmd.position[0] = 0.0;
	//   vel_cmd.position[1] = 0.0;
	// ** Velocity (rad/s) ** 
	//    setting these to returning speed to start with, although the pan joint will be changed throughout this program
	vel_cmd.velocity.push_back(pan_speed_returning_);
	vel_cmd.velocity.push_back(pan_speed_returning_);
	// ** Effort **
	//    the torque to be applied to each joint. Again, we can leave these blank and they won't be controlled, which suits us here
	// vel_cmd.effort[0] = 0.0; 
	// vel_cmd.effort[1] = 0.0;

	ROS_INFO_STREAM("[LIDARServoManager] Received stationary_scan service callback. Finding jointstate...");
	int joint_update_attempts = 0;
	int max_joint_update_attempts = 5;
	while(!this->updateJoints() && joint_update_attempts < max_joint_update_attempts)
	{
		ROS_ERROR_STREAM("[LIDARServoManager] Failed to update joints " << joint_update_attempts << " times.");
		joint_update_attempts++;
		ros::Duration(0.05).sleep();
	}
	if(joint_update_attempts >= max_joint_update_attempts)
	{
		ROS_ERROR_STREAM("[LIDARServoManager] Failed to update joints too many times... exiting this service call.");
		return false;
	}

	// Move pan joint to start position:
	if(fixed_start_state_)
	{
		vel_cmd.position.push_back(start_state_[0]);
		vel_cmd.position.push_back(start_state_[1]);
		ROS_INFO_STREAM("[LIDARServoManager] Moving to initial position:  " << vel_cmd);
		ros::Duration(3.0).sleep();
		servo_pub_.publish(vel_cmd);
	}

	min_angle_ = req.min_angle;
	max_angle_ = req.max_angle;	

	std_msgs::Bool scanning_state;
	if(scan_while_returning_)
	{
		scanning_state.data = true;
		scanning_state_pub_.publish(scanning_state);
		ROS_INFO_STREAM("[LIDARServoManager] Sending scanning start message.");
	}
	ros::Duration(0.15).sleep();

	// Turn counterclockwise prior to scanning - get to 'max_angle' starting point:
	ROS_INFO_STREAM("[LIDARServoManager] Moving pan joint towards point " << max_angle_ << " at speed " << pan_speed_returning_);
	while(pan_angle_ < max_angle_ && ros::ok())
	{
		// During scans we don't want to force position, just specify velocity, so overwrite the existing position specifications to an empty vector
		vel_cmd.position.clear();
		vel_cmd.position.push_back(max_angle_*1.1); 		// make position command overshoot slightly to overcome servo deadband... will stop at target anyway
		vel_cmd.position.push_back(0.0);
		// Set new velocity goal for Pan joint only - Tilt joint remains at 0.0
		vel_cmd.velocity.clear();
		vel_cmd.velocity.push_back(pan_speed_returning_);
		vel_cmd.velocity.push_back(0.0);
		// Debugging output and send servo command
		ROS_DEBUG_STREAM("[LIDARServoManager] Counterclockwise pan speed:  " << pan_speed_ << " Current Angle: " << pan_angle_ << " Max Angle: " << max_angle_);
		servo_pub_.publish(vel_cmd);
		
		int joint_update_attempts = 0;
		int max_joint_update_attempts = 5;
		while(!this->updateJoints() && joint_update_attempts < max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[LIDARServoManager] Failed to update joints " << joint_update_attempts << " times.");
			joint_update_attempts++;
			ros::Duration(0.05).sleep();
		}
		if(joint_update_attempts >= max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[LIDARServoManager] Failed to update joints too many times... exiting this service call.");
			return false;
		}

		ROS_DEBUG_STREAM("[LIDARServoManager] Sent a counterclockwise motion command. Current position: " << pan_angle_);
		ros::Duration(0.05).sleep();
	}
	
	if(!scan_while_returning_)
	{
		scanning_state.data = true;
		scanning_state_pub_.publish(scanning_state);
		ROS_INFO_STREAM("[LIDARServoManager] Sending scanning start message.");
	}
	ros::Duration(0.15).sleep();

	// Turn clockwise while scanning - get to 'min_angle' stopping point:
	ROS_INFO_STREAM("[LIDARServoManager] Moving pan joint towards point " << min_angle_ << " at speed " << pan_speed_);
	while(pan_angle_ > min_angle_ && ros::ok())
	{
		// During scans we don't want to force position, just specify velocity, so overwrite the existing position specifications to an empty vector
		vel_cmd.position.clear();
		vel_cmd.position.push_back(min_angle_*1.1); 		// make position command overshoot slightly to overcome servo deadband... will stop at target anyway
		vel_cmd.position.push_back(0.0);
		// Set new velocity goal for Pan joint only - Tilt joint remains at 0.0
		vel_cmd.velocity.clear();
		vel_cmd.velocity.push_back(pan_speed_);
		vel_cmd.velocity.push_back(0.0);
		// Debugging output and send servo command
		ROS_DEBUG_STREAM("[LIDARServoManager] Clockwise pan speed:  " << pan_speed_ << " Current Angle: " << pan_angle_ << " Min Angle: " << min_angle_);
		servo_pub_.publish(vel_cmd);
		
		int joint_update_attempts = 0;
		int max_joint_update_attempts = 5;
		while(!this->updateJoints() && joint_update_attempts < max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[LIDARServoManager] Failed to update joints " << joint_update_attempts << " times.");
			joint_update_attempts++;
			ros::Duration(0.05).sleep();
		}
		if(joint_update_attempts >= max_joint_update_attempts)
		{
			ROS_ERROR_STREAM("[LIDARServoManager] Failed to update joints too many times... exiting this service call.");
			return false;
		}

		ROS_DEBUG_STREAM("[LIDARServoManager] Sent a counterclockwise motion command. Current position: " << pan_angle_);
		ros::Duration(0.05).sleep();
	}

	output_cloud_names_.clear();
	output_clouds_.clear();
	getOutputClouds();
	for(int i=0; i<output_clouds_.size(); i++)
	{
		res.cloud_names.push_back(output_cloud_names_[i]);
		res.output_clouds.push_back(output_clouds_[i]);
	}

	scanning_state.data = false;
	ROS_INFO_STREAM("[LIDARServoManager] About to publish message to turn scanning routine off!");
	scanning_state_pub_.publish(scanning_state); 			// Shut down laser stitcher process


	return true;  
}

// Output Cloud Gatherer
//   This is run at the end of each routine --> populates output of service object from stationaryScan function
void LIDARServoManager::getOutputClouds()
{
	still_need_cloud_ = true;
	output_cloud_sub_ = nh_.subscribe<laser_stitcher::stitched_clouds>(output_clouds_topic_, 1, &LIDARServoManager::outputCallback, this);
	while(still_need_cloud_ && ros::ok())
	{
		ros::spinOnce();
		ros::Duration(0.05).sleep();
	}
}

// Laser_Stitcher Cloud Output Callback
//   Used internally within getOutputClouds
//   Just catches each output cloud from laser_stitcher and assigns them to stationaryScan service object list
//   Then tells the owning class that it's done and we can end
void LIDARServoManager::outputCallback(const laser_stitcher::stitched_clouds output_clouds)
{
	if(still_need_cloud_)
	{
		for(int i=0; i<output_clouds.clouds.size(); i++)
		{
			output_cloud_names_.push_back(output_clouds.cloud_names[i]);
			output_clouds_.push_back(output_clouds.clouds[i]);
		}
		still_need_cloud_ = false;
	}
	output_cloud_sub_.shutdown();
}

// Joint State Gatherer
//   Tries to get jointstate of servo. If it can't, continues trying. If it fails too many times, throws an error and exits
bool LIDARServoManager::updateJoints()
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

		/* If necessary...
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
	  	rotation.setRPY(0.0, 0.0, pan_angle_);
	  	lidar_transform.transform.rotation.x = rotation.x();
	  	lidar_transform.transform.rotation.y = rotation.y();
	  	lidar_transform.transform.rotation.z = rotation.z();
	  	lidar_transform.transform.rotation.w = rotation.w();
	  	//   Broadcast it!
	  	lidar_frame_broadcaster.sendTransform(lidar_transform);
	  	//ROS_DEBUG_STREAM("Published a transform at: " << lidar_transform);
	  	*/
		ros::Duration(0.05).sleep();
	}
	if(correct_callbacks_ > 0)
	{
		ROS_DEBUG_STREAM("[LIDARServoManager] Successfully updated joint states. New states:");

		ROS_DEBUG_STREAM("[LIDARServoManager] Correct callbacks: " << correct_callbacks_ << " Total callbacks: " << callbacks_received_);	
		return true;
	}
	else 
	{
		ROS_ERROR_STREAM("[LIDARServoManager] Joint update failure. " << callbacks_received_ << " total callbacks received, and no callbacks for the correct jointspace, in " << time_elapsed << " seconds.");
		return false;
	}
}

// Joint State Callback
//   Run between successive output commands to get input - current state of servo
//   Increments a class member counter of number of attempts to get jointstate
//   If we fail enough times, higher level service will decide something isn't set up right, throw an error, and exit
void LIDARServoManager::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
	ROS_DEBUG_STREAM("[LIDARServoManager] Received a servo jointstate input: " << joint_states);

	callbacks_received_++;
	if(joint_states->name[0] == joint_names_[0])
	{
		joint_states_ = *joint_states;
		pan_angle_ = joint_states->position[0];
		correct_callbacks_++;
	}
	std::string name = joint_states->name[0];
	ROS_DEBUG_STREAM(name << " " << callbacks_received_ << " " << correct_callbacks_);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_servo_manager");

    // Uncomment the following to enable DEBUGGING output
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
      ros::console::notifyLoggerLevelsChanged();

	LIDARServoManager servo_manager;
}

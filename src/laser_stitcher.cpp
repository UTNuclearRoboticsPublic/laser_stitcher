
#include "laser_stitcher/laser_stitcher.h"

/* ------------------------- Constructor -------------------------
  Attempts to initialize most variables from ros Parameter Server
    Defaults are set for most things, in case yaml isn't loaded\
*/
LaserStitcher::LaserStitcher()
{

	// These three variables not used elsewhere in class...
	std::string laser_topic, pointcloud_topic, finished_topic, reset_topic;

	if( !nh_.param<std::string>("laser_stitcher/laser_topic", laser_topic, "hokuyo_scan") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get laser topic from parameter server - defaulting to " << laser_topic << ".");
	nh_.param<float>("laser_stitcher/sleepy_time", sleepy_time_, 0.1);
	nh_.param<std::string>("laser_stitcher/reset_topic", reset_topic, "reset_map_scan");
	nh_.param<std::string>("laser_stitcher/finished_topic", finished_topic, "laser_scan_finished");

	// ----- Movement Check -----
	nh_.param<bool>("laser_stitcher/should_check_movement", should_check_movement_, false);
	nh_.param<float>("laser_stitcher/distance_threshold", distance_threshold_, 0.02);	
	nh_.param<float>("laser_stitcher/angle_threshold", angle_threshold_, 0.05);  
	nh_.param<std::string>("laser_stitcher/target_frame", target_frame_, "map");
	/*
	tf::Quaternion zero_rotation(0.0, 0.0, 0.0, 1.0);
	tf::Vector3 zero_vector(0.0, 0.0, 0.0);
	tf::Transform transform;
	transform.setOrigin(zero_vector);
	transform.setRotation(zero_rotation);
	last_transform.setData(transform);
*/
	last_transform_.setIdentity();

	planar_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("laser_stitcher/planar_cloud", 1, this);

	// ----- Subscribers -----
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, &LaserStitcher::laserCallback, this);
	finish_sub_ = nh_.subscribe<std_msgs::Bool>(finished_topic, 20, &LaserStitcher::setScanningState, this);
	reset_sub_ = nh_.subscribe<std_msgs::Bool>(reset_topic, 20, &LaserStitcher::resetCloud, this);
	
	//tf2_ros::tfBuffer tfBuffer_;
	tf::TransformListener listener_;
	//tf2::BufferCore buffer_core_(ros::Duration(10));

	is_running_ = false;

	std::string yaml_file_name;
	if(!nh_.getParam("laser_stitcher/yaml_file_name", yaml_file_name))
	{
		ROS_ERROR_STREAM("[LaserStitcher] Failed to get yaml file name for output cloud format.");
		return;
	}
	this->buildSettings(yaml_file_name);

	postprocessor_ = nh_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");

	output_cloud_list_pub_ = nh_.advertise<laser_stitcher::stitched_clouds>("laser_stitcher/output_cloud_list", 1, this);

	ros::Duration(0.50).sleep();
	ROS_INFO_STREAM("[LaserStitcher] Stitcher online and ready.");

	while(ros::ok())
		ros::spinOnce();
}

bool LaserStitcher::buildSettings(std::string yaml_file_name)
{
	std::vector<std::string> cloud_list;
	if(!nh_.getParam(yaml_file_name + "/cloud_list", cloud_list))
	{
		ROS_ERROR_STREAM("[LaserStitcher] Failed to get list of output cloud names from the parameters server. Settings initialization failed.");
		return false;
	}
	ROS_INFO_STREAM("[LaserStitcher] Retrieved list of output cloud names from parameter server - " << cloud_list.size() << " entries.");

	output_settings_.clear();
	for(int i=0; i<cloud_list.size(); i++)
	{
		StitchedCloud cloud_options;

		cloud_options.cloud_name_ = cloud_list[i];
		cloud_options.cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("laser_stitcher/" + cloud_list[i],1,this);

		nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/incremental_update", cloud_options.incremental_update_, true);
		nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/retain_after_scan", cloud_options.retain_after_scan_, true);
		nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/should_pub", cloud_options.should_pub_, true);
		nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/should_save", cloud_options.should_save_, true);
		nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/should_postprocess", cloud_options.should_postprocess_, false);

		nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/throttle_publish", cloud_options.throttle_publish_, false);
		if(cloud_options.throttle_publish_)
		{
			if(!nh_.param<int>(yaml_file_name + "/" + cloud_list[i] + "/publishing_throttle_max", cloud_options.publishing_throttle_max_, 5))
				ROS_WARN_STREAM("[LaserStitcher] Publishing throttling requested for output " << cloud_list[i] << ", but throttle maximum not found in parameter server. Setting publish throttle to " << cloud_options.voxel_throttle_max_);
			cloud_options.publishing_throttle_counter_ = 0;
		}

		if(cloud_options.should_postprocess_)
		{
			pointcloud_processing_server::pointcloud_process postprocess;
			PointcloudTaskCreation::processFromYAML(&postprocess, cloud_list[i], yaml_file_name);
			cloud_options.postprocess_ = postprocess;
			
			nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/throttle_postprocess", cloud_options.throttle_postprocess_, false);
			if(cloud_options.throttle_postprocess_)
			{
				if(!nh_.param<int>(yaml_file_name + "/" + cloud_list[i] + "/postprocess_throttle_max", cloud_options.postprocess_throttle_max_, 50))
					ROS_WARN_STREAM("[LaserStitcher] Postprocessing throttling requested for output " << cloud_list[i] << ", but throttle maximum not found in parameter server. Setting voxel throttle to " << cloud_options.voxel_throttle_max_);
				cloud_options.postprocess_throttle_counter_ = 0;
			}
			
		}

		output_settings_.push_back(cloud_options);
	}
}

/* ------------------------- Laser Callback -------------------------
  Primary callback node - receives laser_scan data from external source
	Checks whether LIDAR has moved meaningfully from last scan added
	If so, adds the new scan to the total pointcloud
	Then, checks whether cloud size threshold has been met
	If so, saves off the cloud and starts a new one
*/
void LaserStitcher::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{ 
try{
	if(is_running_)
	{
		if(!listener_.waitForTransform(scan_in->header.frame_id, target_frame_, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0)))
		{
			ROS_ERROR_STREAM("[LaserStitcher] Received laser_scan callback, but failed to get transform between target " << target_frame_ << " and scan frame " << scan_in->header.frame_id);
	 		//return;
	  	}

	  	output_cloud_list_.cloud_names.clear();
	  	output_cloud_list_.clouds.clear();		// Clear all output clouds from publishing message

	  	bool add_cloud = true;
	  	if(should_check_movement_)
	  		add_cloud = lidarHasMoved(scan_in->header.frame_id);

	  	if( add_cloud )
	  	{
	  		//std::string frames_found;
	  		//frames_found = buffer_core_.allFramesAsString();
	  		//ROS_ERROR_STREAM(frames_found);
		  	sensor_msgs::PointCloud2 new_planar_cloud;
		  	scan_converter_.transformLaserScanToPointCloud(target_frame_, *scan_in, new_planar_cloud, listener_);
		  	pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		  	pcl::fromROSMsg(new_planar_cloud, *temp_cloud);
		  	pcl::toROSMsg(*temp_cloud, new_planar_cloud);
		  	planar_cloud_pub_.publish(new_planar_cloud);
		  	for(int i=0; i<output_settings_.size(); i++)
		  	{
			  	const sensor_msgs::PointCloud2 previous_cloud_state = output_settings_[i].cloud_;

			    pcl::concatenatePointCloud(previous_cloud_state, new_planar_cloud, output_settings_[i].cloud_);
			    ROS_DEBUG_STREAM("[LaserStitcher] Laser scan caught and stitched!");
			    if(output_settings_[i].should_postprocess_)
			    {
			    	bool postprocess = true;
			    	if(output_settings_[i].throttle_postprocess_)
			    	{
				    	if(output_settings_[i].postprocess_throttle_counter_ < output_settings_[i].postprocess_throttle_max_)
				    	{
					    	postprocess = false;
			    			output_settings_[i].postprocess_throttle_counter_++;
				    	}
				    	else
				    		output_settings_[i].postprocess_throttle_counter_ = 0;
			    	}
			    	ROS_DEBUG_STREAM(output_settings_[i].cloud_name_ << " " << postprocess << " " << output_settings_[i].postprocess_throttle_counter_ << " " << output_settings_[i].postprocess_throttle_max_);
			    	if(postprocess)
			    	{
			    		ROS_ERROR_STREAM(output_settings_[i].cloud_.data.size() << " " << output_settings_[i].cloud_name_ << output_settings_[i].cloud_.height << " " << output_settings_[i].cloud_.width);
			    		output_settings_[i].postprocess_.request.pointcloud = output_settings_[i].cloud_;
			    		postprocessor_.call(output_settings_[i].postprocess_);
			    		
			    		sensor_msgs::PointCloud2Modifier cloud_modifier_(output_settings_[i].cloud_);
						cloud_modifier_.resize(0);

			    		sensor_msgs::PointCloud2 postprocessed_cloud = output_settings_[i].postprocess_.response.task_results[output_settings_[i].postprocess_.response.task_results.size()-1].task_pointcloud;
			    		
			    		output_settings_[i].cloud_.point_step = postprocessed_cloud.point_step;
			    		output_settings_[i].cloud_.row_step = postprocessed_cloud.row_step;
			    		output_settings_[i].cloud_.width = postprocessed_cloud.width;
			    		output_settings_[i].cloud_.data = postprocessed_cloud.data;

			    		ROS_ERROR_STREAM(output_settings_[i].cloud_.data.size() << " " << output_settings_[i].cloud_name_ << output_settings_[i].cloud_.height << " " << output_settings_[i].cloud_.width);
			    	}
			    }
		    	ROS_DEBUG_STREAM("[LaserStitcher] Should publish: " << output_settings_[i].incremental_update_ << "; Publishing topic: " << cloud_pub_.getTopic() << "; Current cloud size: " << summed_pointcloud_.width*summed_pointcloud_.height);
			    if(output_settings_[i].incremental_update_)
		    	{
			    	bool publish = true;
			    	if(output_settings_[i].throttle_publish_)
			    	{
			    		if(output_settings_[i].publishing_throttle_counter_ < output_settings_[i].publishing_throttle_max_)
			    		{
			    			publish = false;
			    			output_settings_[i].publishing_throttle_counter_++;
			    		}
			    		else
			    			output_settings_[i].publishing_throttle_counter_ = 0;
			    	}
			    	if(publish)
	    				output_settings_[i].cloud_pub_.publish(output_settings_[i].cloud_);
				}

				output_cloud_list_.clouds.push_back(output_settings_[i].cloud_);
				output_cloud_list_.cloud_names.push_back(output_settings_[i].cloud_name_);
			}

		}
		output_cloud_list_pub_.publish(output_cloud_list_);
    }
    else 
    	ros::Duration(sleepy_time_).sleep();
}
catch(tf2::TransformException e)
{
ROS_ERROR_THROTTLE(2, "laser_stitcher got a tf2 exception");
}

}

/* ------------------------- Set Scanning State -------------------------
  Callback to tell system to begin or end scanning routine
    Receives a boolean input to set the scanning state.
    When a scan finishes, it saves the created cloud to a yaml file 
      and publishes it to an output topic as well. 
*/
void LaserStitcher::setScanningState(const std_msgs::Bool::ConstPtr& is_running)
{ 
	if(!(is_running->data) && is_running_)			// Turning off, after it's been on
	{
		for(int i=0; i<output_settings_.size(); i++)
		{
			// Run one final postprocess where necessary
			if(output_settings_[i].should_postprocess_)
		    {
	    		ROS_ERROR_STREAM(output_settings_[i].cloud_.data.size() << " " << output_settings_[i].cloud_name_ << output_settings_[i].cloud_.height << " " << output_settings_[i].cloud_.width);
	    		output_settings_[i].postprocess_.request.pointcloud = output_settings_[i].cloud_;
	    		postprocessor_.call(output_settings_[i].postprocess_);
	    		
	    		sensor_msgs::PointCloud2Modifier cloud_modifier_(output_settings_[i].cloud_);
				cloud_modifier_.resize(0);

	    		sensor_msgs::PointCloud2 temp_cloud = output_settings_[i].postprocess_.response.task_results[output_settings_[i].postprocess_.response.task_results.size()-1].task_pointcloud;
	    		
	    		output_settings_[i].cloud_.point_step = temp_cloud.point_step;
	    		output_settings_[i].cloud_.row_step = temp_cloud.row_step;
	    		output_settings_[i].cloud_.width = temp_cloud.width;
	    		output_settings_[i].cloud_.data = temp_cloud.data;

	    		ROS_ERROR_STREAM(output_settings_[i].cloud_.data.size() << " " << output_settings_[i].cloud_name_ << output_settings_[i].cloud_.height << " " << output_settings_[i].cloud_.width);
		    }

			if(output_settings_[i].should_save_)
			{
				rosbag::Bag bag;
				bag.open(output_settings_[i].cloud_name_+".bag", rosbag::bagmode::Write);
				bag.write(output_settings_[i].cloud_pub_.getTopic(), ros::Time::now(), output_settings_[i].cloud_);
				ROS_INFO_STREAM("[LaserStitcher] Saved a ROSBAG to the file " << output_settings_[i].cloud_name_+".bag");
			}
			output_settings_[i].cloud_pub_.publish(output_settings_[i].cloud_);
			ROS_INFO_STREAM("[LaserStitcher] Finished a stitching routine on cloud " << output_settings_[i].cloud_name_ << ". Final cloud size: " << output_settings_[i].cloud_.height*output_settings_[i].cloud_.width << ".");

			if(!output_settings_[i].retain_after_scan_)
			{
				sensor_msgs::PointCloud2Modifier cloud_modifier_(output_settings_[i].cloud_);
				cloud_modifier_.resize(0);
			}
		}
	}
	else if(is_running->data && !is_running_)		// Turning on, after it's been off
	{
		ROS_INFO_STREAM("[LaserStitcher] Beginning a stitching routine.");
	}
	else if(is_running->data)						// Turning on, when it's already on
		ROS_DEBUG_STREAM("[LaserStitcher] Received call to begin stitching, but was already stitching. No change made.");
	else									// Turning off, when it's already off 
		ROS_DEBUG_STREAM("[LaserStitcher] Received call to stop stitching, but was already not stitching. No change made.");

	is_running_ = is_running->data;
}

/* ------------------------- Kill Cloud -------------------------
  Callback to tell system to depopulate and reset the cloud.
    This can also be done automatically when a scan is halted, if the
	  class variable reset_cloud_when_stopped_ is set to TRUE
*/
void LaserStitcher::resetCloud(const std_msgs::Bool::ConstPtr& placeholder)
{ 
	sensor_msgs::PointCloud2Modifier cloud_modifier_(summed_pointcloud_);
	cloud_modifier_.resize(0);
}

/* ------------------------- LIDAR Has Moved -------------------------
  
  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  XXXXXXXXXXXXXXXXXXXXXX    NOT FULLY IMPLEMENTED XXXXXXXXXXXXXXXXXXXXXX
  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

  Checks whether the LIDAR frame has moved from the last scan position
    meaningfully enough to warrant adding a new cloud. 
    Prevents many identical scans from being added.
    If operating in a dynamic environment (instead of a static environment
      with a dynamic robot positon), including a parameter whcih can keep this
      check from occurring. 
*/
bool LaserStitcher::lidarHasMoved(std::string laser_frame)
{
	bool output = false;  
	try
	{ 
		tf::StampedTransform current_transform;
		listener_.lookupTransform(target_frame_, laser_frame, ros::Time(0), current_transform);

		// Distance Check
		float distance = sqrt(pow((current_transform.getOrigin().x() - last_transform_.getOrigin().x()),2) + pow((current_transform.getOrigin().y() - last_transform_.getOrigin().y()),2) + pow((current_transform.getOrigin().z() - last_transform_.getOrigin().z()),2));
		if ( distance > distance_threshold_ ) 
			output = true;

		// Rotation Check 
		tf::Quaternion quat = current_transform.getRotation()*last_transform_.getRotation().inverse();
		if ( quat.getW() > angle_threshold_ )
			output = true;

		last_transform_ = current_transform; 
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	} 
	return output;
}

int main(int argc, char** argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	ros::init(argc, argv, "laser_stitcher");

//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
 //   ros::console::notifyLoggerLevelsChanged();

	LaserStitcher laser_stitcher;
}

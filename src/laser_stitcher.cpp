
#include "laser_stitcher/laser_stitcher.h"

/* ------------------------- Constructor -------------------------
  Attempts to initialize most variables from ros Parameter Server
    Defaults are set for most things, in case yaml isn't loaded\
*/
LaserStitcher::LaserStitcher()
{
	// These three variables not used elsewhere in class...
	std::string laser_topic, pointcloud_topic, finished_topic;

	// ----- Input Topics -----
	if( !nh_.param<std::string>("laser_stitcher/laser_topic", laser_topic, "hokuyo_scan") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get laser topic from parameter server - defaulting to " << laser_topic << ".");
	if( !nh_.param<std::string>("laser_stitcher/laser_topic", finished_topic, "laser_stitcher/scanning_state") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get scanning-state topic from parameter server - defaulting to " << finished_topic << ".");

	// ----- Output Stuff -----
	if( !nh_.param<std::string>("laser_stitcher/laser_topic", target_frame_, "map") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get target frame from parameter server - defaulting to " << target_frame_ << ".");
	nh_.param<float>("laser_stitcher/sleepy_time", sleepy_time_, 0.1);


	// ----- Publishing -----
	if( !nh_.param<std::string>("laser_stitcher/laser_topic", pointcloud_topic, "laser_stitcher/output_cloud") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get output topic from parameter server - defaulting to " << pointcloud_topic << ".");
	nh_.param<bool>("laser_stitcher/publish_after_updating", publish_after_updating_, true);
	nh_.param<int>("laser_stitcher/publishing_throttle", publishing_throttle_, 1);
	throttle_index_ = 0;

	// ----- Data Saving -----
	nh_.param<bool>("laser_stitcher/save_data", save_data_, true);
	nh_.param<std::string>("laser_stitcher/bag_name", bag_name_, "stitched_pointcloud");

	// ----- Movement Check -----
	nh_.param<bool>("laser_stitcher/should_check_movement", should_check_movement_, true);
	nh_.param<float>("laser_stitcher/distance_threshold", distance_threshold_, 0.02);	
	nh_.param<float>("laser_stitcher/angle_threshold", angle_threshold_, 0.05);  /*
	last_transform_.transform.translation.x = 0;
	last_transform_.transform.translation.y = 0;
	last_transform_.transform.translation.z = 0;
	last_transform_.transform.rotation.x = 0;
	last_transform_.transform.rotation.y = 0;
	last_transform_.transform.rotation.z = 0;
	last_transform_.transform.rotation.w = 0;  */

	// ----- Subscribers, Publishers, Listeners -----
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(laser_topic, 20, &LaserStitcher::laserCallback, this);
	finish_sub_ = nh_.subscribe<std_msgs::Bool>(finished_topic, 20, &LaserStitcher::setScanningState, this);
	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 3);
	
	//tf2_ros::tfBuffer tfBuffer_;
	tf::TransformListener listener_;
	//tf2::BufferCore buffer_core_(ros::Duration(10));

	is_running_ = false;

	ros::Duration(2.0).sleep();


	while(ros::ok())
		ros::spinOnce();
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
	if(is_running_)
	{
		if(!listener_.waitForTransform(scan_in->header.frame_id, target_frame_, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0)))
		{
			ROS_ERROR_STREAM("[LaserStitcher] Received laser_scan callback, but failed to get transform between target " << target_frame_ << " and scan frame " << scan_in->header.frame_id);
	 		//return;
	  	}

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

		  	sensor_msgs::PointCloud2 new_summed_cloud;
		  	const sensor_msgs::PointCloud2 current_summed_cloud = summed_pointcloud_;

		    pcl::concatenatePointCloud(current_summed_cloud, new_planar_cloud, summed_pointcloud_);
		    ROS_DEBUG_STREAM("[LaserStitcher] Laser scan caught and stitched!");

		    if(publish_after_updating_)
		    	if(throttle_index_+1 == publishing_throttle_)
		    		cloud_pub_.publish(summed_pointcloud_);
		}
    }
    else 
    	ros::Duration(sleepy_time_).sleep();
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
		if(save_data_)
		{
			rosbag::Bag bag;
			bag.open(bag_name_+".bag", rosbag::bagmode::Write);
			bag.write(cloud_pub_.getTopic(), ros::Time::now(), summed_pointcloud_);
			ROS_INFO_STREAM("[LaserStitcher] Published a ROSBAG to the file " << bag_name_+".bag");
		}
		cloud_pub_.publish(summed_pointcloud_);
		ROS_DEBUG_STREAM("[LaserStitcher] Finished a stitching routine. Final cloud size: " << summed_pointcloud_.height*summed_pointcloud_.width << ".");

		sensor_msgs::PointCloud2Modifier cloud_modifier_(summed_pointcloud_);
		cloud_modifier_.resize(0);
	}
	else if(is_running->data && !is_running_)		// Turning on, after it's been off
	{
		ROS_DEBUG_STREAM("[LaserStitcher] Beginning a stitching routine.");
	}
	else if(is_running->data)						// Turning on, when it's already on
		ROS_DEBUG_STREAM("[LaserStitcher] Received call to begin stitching, but was already stitching. No change made.");
	else									// Turning off, when it's already off 
		ROS_DEBUG_STREAM("[LaserStitcher] Received call to stop stitching, but was already not stitching. No change made.");

	is_running_ = is_running->data;

}

/* ------------------------- LIDAR Has Moved -------------------------
  Checks whether the LIDAR frame has moved from the last scan position
    meaningfully enough to warrant adding a new cloud. 
    Prevents many identical scans from being added.
    If operating in a dynamic environment (instead of a static environment
      with a dynamic robot positon), including a parameter whcih can keep this
      check from occurring. 
*/
bool LaserStitcher::lidarHasMoved(std::string laser_frame)
{
	return true;  /*
	try
	{ 
		tf::StampedTransform current_transform;
		listener_.lookupTransform(target_frame_, laser_frame, ros::Time(0), current_transform);

		// Distance Check
		float distance = sqrt(pow((current_transform.x - last_transform_.x),2) + pow((current_transform.y - last_transform_.y),2) + pow((current_transform.z - last_transform_.z),2));
		if ( distance > distance_threshold_ ) 
			return true;

		// Rotation Check 
		//tf::

		last_transform_ = current_transform; 
	}
	catch
	{

	} */
}

int main(int argc, char** argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	ros::init(argc, argv, "laser_stitcher");

if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

	LaserStitcher laser_stitcher;
}
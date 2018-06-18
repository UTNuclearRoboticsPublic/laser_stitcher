
#include "laser_stitcher/laser_stitcher.h"

/* ------------------------- Constructor -------------------------
  Attempts to initialize most variables from ros Parameter Server
    Defaults are set for most things, in case yaml isn't loaded\
*/
LaserStitcher::LaserStitcher()
{

	// ---------- Input Topics/Subscribers ----------
	// Get Topic Names
	std::string laser_topic, scanning_state_topic, reset_topic;
	if( !nh_.param<std::string>("laser_stitcher/laser_topic", laser_topic, "scan") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get laser topic from parameter server - defaulting to " << laser_topic << ".");
	if( !nh_.param<std::string>("laser_stitcher/reset_topic", reset_topic, "laser_stitcher/reset_scan") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get reset topic from parameter server - defaulting to " << reset_topic << ".");
	if( !nh_.param<std::string>("laser_stitcher/scanning_state_topic", scanning_state_topic, "laser_stitcher/scanning_state") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get scanning_state topic from parameter server - defaulting to " << scanning_state_topic << ".");
	// Create Subscribers
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, &LaserStitcher::laserCallback, this);
	finish_sub_ = nh_.subscribe<std_msgs::Bool>(scanning_state_topic, 20, &LaserStitcher::setScanningState, this);
	reset_sub_ = nh_.subscribe<std_msgs::Bool>(reset_topic, 20, &LaserStitcher::resetCloud, this);
	

	// ---------- Output Topics/Publishers ----------
	// Get Topic Names
	std::string planar_scan_topic, partial_scan_topic, full_scan_topic;
	if( !nh_.param<std::string>("laser_stitcher/planar_scan_topic", planar_scan_topic, "laser_stitcher/planar_scan") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get planar_scan topic from parameter server - defaulting to " << planar_scan_topic << ".");
	if( !nh_.param<std::string>("laser_stitcher/partial_scan_topic", partial_scan_topic, "laser_stitcher/partial_scan") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get partial_scan topic from parameter server - defaulting to " << partial_scan_topic << ".");
	if( !nh_.param<std::string>("laser_stitcher/full_scan_topic", full_scan_topic, "laser_stitcher/full_scan") )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get full_scan topic from parameter server - defaulting to " << full_scan_topic << ".");
	// Create Publishers
	planar_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(planar_scan_topic, 1, this);
 	partial_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(partial_scan_topic, 1, this);
 	full_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(full_scan_topic, 1, this);
 

	// ------------------ TF Stuff ------------------
	nh_.param<std::string>("laser_stitcher/output_frame", output_frame_, "map");
	tf::TransformListener listener_;

	// ----------- Basic Processing Stuff -----------
	is_running_ = false; 
	nh_.param<float>("laser_stitcher/wait_time_if_not_running", wait_time_if_not_running_, 0.1);

	ros::Duration(0.50).sleep();
	ROS_INFO_STREAM("[LaserStitcher] Stitcher online and ready.");

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
	// Try/Catch to prevent crashing on TF exception
	try{
		if(is_running_)
		{
			if(!listener_.waitForTransform(scan_in->header.frame_id, output_frame_, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0)))
			{
				ROS_ERROR_STREAM("[LaserStitcher] Received laser_scan callback, but failed to get transform between output frame " << output_frame_ << " and scan frame " << scan_in->header.frame_id);
		  		return;
		  	}

		  	sensor_msgs::PointCloud2 new_planar_cloud;
		  	scan_converter_.transformLaserScanToPointCloud(output_frame_, *scan_in, new_planar_cloud, listener_);
		  	pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		  	pcl::fromROSMsg(new_planar_cloud, *temp_cloud);
		  	pcl::toROSMsg(*temp_cloud, new_planar_cloud);
		  	planar_cloud_pub_.publish(new_planar_cloud);
		  	ROS_DEBUG_STREAM("[LaserStitcher] Laser scan caught! Publishing a planar scan on topic " << planar_cloud_pub_.getTopic());

		  	const sensor_msgs::PointCloud2 previous_cloud_state = current_scan_;
		    pcl::concatenatePointCloud(previous_cloud_state, new_planar_cloud, current_scan_);
		    ROS_DEBUG_STREAM("[LaserStitcher] Laser scan stitched! Publishing a partial scan on topic " << partial_scan_pub_.getTopic());
		    partial_scan_pub_.publish(current_scan_);
	    }
	    else 
	    	ros::Duration(wait_time_if_not_running_).sleep();
	}
	catch(tf2::TransformException e)
	{
		ROS_ERROR_THROTTLE(2, "%s", e.what());
	}

}

/* ------------------------- Set Scanning State -------------------------
  Callback to tell system to begin or end scanning routine
    Receives a boolean input to set the scanning state.
    When a scan finishes, it saves the created cloud to a yaml file 
      and publishes it to an output topic as well. 
*/
void LaserStitcher::setScanningState(const std_msgs::Bool::ConstPtr& new_state)
{ 
	if(!(new_state->data) && is_running_)			// Turning off, after it's been on
	{
		if(save_data_)
		{
			rosbag::Bag bag;
			std::string bag_name = "laser_stitcher" + std::to_string(ros::Time::now().toSec()) + ".bag";
			bag.open(bag_name, rosbag::bagmode::Write);
			bag.write(full_scan_pub_.getTopic(), ros::Time::now(), current_scan_);
			ROS_INFO_STREAM("[LaserStitcher] Saved a ROSBAG to the file " << bag_name);
		}
		full_scan_pub_.publish(current_scan_);
		ROS_INFO_STREAM("[LaserStitcher] Finished a stitching routine. Final cloud size: " << current_scan_.height * current_scan_.width << ".");

		sensor_msgs::PointCloud2Modifier cloud_modifier_(current_scan_);
		cloud_modifier_.resize(0);
	}
	else if(new_state->data && !is_running_)		// Turning on, after it's been off
	{
		ROS_INFO_STREAM("[LaserStitcher] Beginning a stitching routine.");
	}
	else if(new_state->data)						// Turning on, when it's already on
		ROS_DEBUG_STREAM("[LaserStitcher] Received call to begin stitching, but was already stitching. No change made.");
	else									// Turning off, when it's already off 
		ROS_DEBUG_STREAM("[LaserStitcher] Received call to stop stitching, but was already not stitching. No change made.");

	is_running_ = new_state->data;
}

/* ------------------------- Kill Cloud -------------------------
  Callback to tell system to depopulate and reset the cloud.
    This can also be done automatically when a scan is halted, if the
	  class variable `en_stopped_ is set to TRUE
*/
void LaserStitcher::resetCloud(const std_msgs::Bool::ConstPtr& placeholder)
{ 
	sensor_msgs::PointCloud2Modifier cloud_modifier_(current_scan_);
	cloud_modifier_.resize(0);
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_stitcher");

//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
 //   ros::console::notifyLoggerLevelsChanged();

	LaserStitcher laser_stitcher;
}

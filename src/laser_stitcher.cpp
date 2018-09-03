
#include "laser_stitcher/laser_stitcher.h"

/* ------------------------- Constructor -------------------------
  Attempts to initialize most variables from ros Parameter Server
    Defaults are set for most things, in case yaml isn't loaded
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
 	// Check Whether to Save Outputs
 	bool temp_bool;
 	if( !nh_.param<bool>("laser_stitcher/should_save", temp_bool, false) )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get should_save from parameter server - defaulting to " << temp_bool << ".");
	save_data_ = temp_bool;

	// ------------------ TF Stuff ------------------
	nh_.param<std::string>("laser_stitcher/output_frame", output_frame_, "map");

	// ----------- Basic Processing Stuff -----------
	is_running_ = false; 
	nh_.param<float>("laser_stitcher/wait_time_if_not_running", wait_time_if_not_running_, 0.1);
	nh_.param<bool>("laser_stitcher/scale_intensities", scale_intensities_, false);
	if(scale_intensities_)
		if( !nh_.param<float>("laser_stitcher/intensity_scale_exp", intensity_scale_exp_, 1.4) )
			ROS_WARN_STREAM("[LaserStitcher] Failed to get intensity scaling coefficients, when scaling was requested. Defaulting to a simple inverse square function.");

	ros::Duration(0.50).sleep();
	ROS_INFO_STREAM("[LaserStitcher] Stitcher online and ready.");

	ros::spin();
}

/* ------------------------- Laser Callback -------------------------
  Primary callback node - receives laser_scan data from external source
*/
void LaserStitcher::laserCallback(sensor_msgs::LaserScan scan_in)
{ 
	// Scale intensities based on distance, if necessary
	//   Return intensities decrease as some function of distance regardless of surface type and angle
	//   Here is a simple implementation to scale intensities by some power of their distance (specified in parameter yaml file) 
	if(scale_intensities_)
	{
		for(int i=0; i<scan_in.ranges.size(); i++)
			scan_in.intensities[i] *= pow(scan_in.ranges[i],intensity_scale_exp_);
	}  
	// Actual Transform and Cloud Building
	//   Try/Catch to prevent crashing on TF exception
	try{
		if(is_running_)
		{
			if(!listener_.waitForTransform(scan_in.header.frame_id, output_frame_, scan_in.header.stamp + ros::Duration().fromSec(scan_in.ranges.size()*scan_in.time_increment), ros::Duration(1.0)))
			{
				ROS_ERROR_STREAM("[LaserStitcher] Received laser_scan callback, but failed to get transform between output frame " << output_frame_ << " and scan frame " << scan_in.header.frame_id);
		  		return;
		  	}

		  	sensor_msgs::PointCloud2 new_planar_cloud;
		  	scan_converter_.transformLaserScanToPointCloud(output_frame_, scan_in, new_planar_cloud, listener_);
		  	pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		  	pcl::fromROSMsg(new_planar_cloud, *temp_cloud);
		  	pcl::toROSMsg(*temp_cloud, new_planar_cloud);
		  	planar_cloud_pub_.publish(new_planar_cloud);
		  	ROS_DEBUG_STREAM("[LaserStitcher] Laser scan caught! Publishing a planar scan on topic " << planar_cloud_pub_.getTopic() << " with size " << new_planar_cloud.width*new_planar_cloud.height);

		  	const sensor_msgs::PointCloud2 previous_cloud_state = current_scan_;
		    pcl::concatenatePointCloud(previous_cloud_state, new_planar_cloud, current_scan_);
		    ROS_DEBUG_STREAM("[LaserStitcher] Laser scan stitched! Publishing a partial scan on topic " << partial_scan_pub_.getTopic() << " with size " << current_scan_.width*current_scan_.height);
		    partial_scan_pub_.publish(current_scan_);
	    }
	    else 
	    	ros::Duration(wait_time_if_not_running_).sleep();
	}
	catch(tf2::TransformException e)
	{
		ROS_ERROR_THROTTLE(2, "[LaserStitcher] %s", e.what());
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
			std::string bag_name = "laser_stitcher_" + std::to_string(ros::Time::now().toSec()) + ".bag";
			bag.open(bag_name, rosbag::bagmode::Write);
			bag.write(full_scan_pub_.getTopic(), ros::Time::now(), current_scan_);
			ROS_INFO_STREAM("[LaserStitcher] Saved a ROSBAG to the file " << bag_name);

			pcl::PointCloud<pcl::PointXYZI> temp_cloud;
			pcl::fromROSMsg(current_scan_, temp_cloud);
		    pcl::io::savePCDFileASCII("laser_stitcher_" + std::to_string(ros::Time::now().toSec()) + ".pcd", temp_cloud);
		    ROS_INFO_STREAM("[LaserStitcher] Saved " << temp_cloud.points.size() << " data points to a pcd file");
		}
		full_scan_pub_.publish(current_scan_);
		ROS_INFO_STREAM("[LaserStitcher] Finished a stitching routine. Final cloud size: " << current_scan_.height * current_scan_.width << ".");

		sensor_msgs::PointCloud2Modifier cloud_modifier_(current_scan_);
		cloud_modifier_.resize(0);
	}
	else if(new_state->data && !is_running_)		// Turning on, after it's been off
	{
		ROS_INFO_STREAM("[LaserStitcher] Beginning a stitching routine.");
		pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::toROSMsg(*temp_cloud, current_scan_);
		current_scan_.header.frame_id = output_frame_;
		current_scan_.header.stamp = ros::Time::now();
	}
	else if(new_state->data)						// Turning on, when it's already on
		ROS_WARN_STREAM("[LaserStitcher] Received call to begin stitching, but was already stitching. No change made.");
	else									// Turning off, when it's already off 
		ROS_WARN_STREAM("[LaserStitcher] Received call to stop stitching, but was already not stitching. No change made.");

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
//    ros::console::notifyLoggerLevelsChanged();

	LaserStitcher laser_stitcher;
}

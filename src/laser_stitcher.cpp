
#include "laser_stitcher/laser_stitcher.h"

LaserStitcher::LaserStitcher()
{
	// These default values overwritten if launched with parameters:
	target_frame_ = 				"map";
	std::string laser_topic = 		"hokuyo_scan";
	std::string pointcloud_topic = 	"laser_stitcher/output_cloud";
	std::string finished_topic = 	"laser_stitcher/scanning_state";
	// Attempt to overwrite defaults using 
	if( !nh_.getParam("laser_stitcher/laser_topic", laser_topic) )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get laser topic from parameter server - defaulting to " << laser_topic << ".");
	if( !nh_.getParam("laser_stitcher/laser_topic", target_frame_) )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get target frame from parameter server - defaulting to " << target_frame_ << ".");
	if( !nh_.getParam("laser_stitcher/laser_topic", pointcloud_topic) )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get output topic from parameter server - defaulting to " << pointcloud_topic << ".");
	if( !nh_.getParam("laser_stitcher/laser_topic", finished_topic) )
		ROS_WARN_STREAM("[LaserStitcher] Failed to get scanning-state topic from parameter server - defaulting to " << finished_topic << ".");
	nh_.param<bool>("laser_stitcher/save_data", save_data_, true);
	nh_.param<std::string>("laser_stitcher/bag_name", bag_name_, "stitched_pointcloud");

	tf::TransformListener listener_;
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(laser_topic, 20, &LaserStitcher::laserCallback, this);
	finish_sub_ = nh_.subscribe<std_msgs::Bool>(finished_topic, 20, &LaserStitcher::setScanningState, this);
	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 3);

	is_running_ = false;

	while(ros::ok())
		ros::spinOnce();
}

void LaserStitcher::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	if(is_running_)
	{
		if(!listener_.waitForTransform(scan_in->header.frame_id, target_frame_, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0)))
		{
			ROS_ERROR_STREAM("[LaserStitcher] Received laser_scan callback, but failed to get transform between target " << target_frame_ << " and scan frame " << scan_in->header.frame_id);
	 		return;
	  	}

	  	sensor_msgs::PointCloud2 new_planar_cloud;
	  	scan_converter_.transformLaserScanToPointCloud(target_frame_, *scan_in, new_planar_cloud, listener_);

	  	sensor_msgs::PointCloud2 new_summed_cloud;
	  	const sensor_msgs::PointCloud2 current_summed_cloud = summed_pointcloud_;

	    pcl::concatenatePointCloud(current_summed_cloud, new_planar_cloud, summed_pointcloud_);
	    ROS_DEBUG_STREAM("[LaserStitcher] Laser scan caught and stitched!");
    }
    else 
    	ros::Duration(0.1).sleep();

    //temporary:
    cloud_pub_.publish(summed_pointcloud_);
}

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

void LaserStitcher::checkFrameChange()
{
	
}

int main(int argc, char** argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	ros::init(argc, argv, "laser_stitcher");

if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

	LaserStitcher laser_stitcher;
}
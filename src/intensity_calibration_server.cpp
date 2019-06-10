

#include <laser_stitcher/intensity_calibration.h>
#include <laser_stitcher/intensity_calibration_service.h>

bool calibrateIntensity(laser_stitcher::intensity_calibration_service::Request &req, laser_stitcher::intensity_calibration_service::Response &res)
{
	ROS_INFO_STREAM("[IntensityCalibrationServer] Received call to normalize intensity with input cloud size " << req.input_cloud.width*req.input_cloud.height);
	pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr normalized(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(req.input_cloud, *input);
	pcl::fromROSMsg(res.normalized_cloud, *normalized);

	LaserIntensityCalibration::normalizeCloud(input, normalized, req.lookup_table_name, req.z_offset);

	pcl::toROSMsg(*normalized, res.normalized_cloud);
	res.normalized_cloud.header.frame_id = "map";

	return true;
}



int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "wall_change_tester");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();

  	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    ros::NodeHandle nh;

    ros::ServiceServer normalization_server = nh.advertiseService("normalize_point_cloud_intensity", calibrateIntensity);

    ros::spin();

}





#include <ros/ros.h>
// Bags
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

// Basic PCL Stuff
#include <pcl_conversions/pcl_conversions.h> 			// fromROSMsg(), toROSMsg() 
#include <pcl/kdtree/kdtree.h>

// Plane Segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// CVBridge
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

// Eigen (matrix handling)
#include <Eigen/Dense>

#include <math.h>

namespace LaserIntensityCalibration
{

	float interpRaster(cv_bridge::CvImagePtr image_ptr, int j, int i, float x_frac, float y_frac);
	// Lookup a value f' in image table I for a given point f(x,y)
	//   Might change this to interpolation at some point, but for now this probably works fine
	//   Here, this is used to look up intensity scaling values based on geometry, but could be used for other purposes also
	//   DIMENSIONS is a 4-valued vector containing {X_MIN, Y_MIN, X_MAX, Y_MAX} for the given image
	float rasterLookup(cv_bridge::CvImagePtr image_ptr, float f, float x, float y, std::vector<float> dimensions);
	// Perform plane fit (RANSAC) to get plane normal
	Eigen::Vector3f planeFit(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
	// Load Input Cloud to have its Intensity Normalized
	void loadCloud(std::string path_to_cloud, std::string cloud_topic, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
	// Load Intensity Lookup Table Image using OpenCV
	void loadImage(std::string path_to_image, cv_bridge::CvImagePtr table_ptr);
	// Actually perform normalization
	void normalizeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr normalized, cv_bridge::CvImagePtr table_ptr, float z_offset);
	// Wrapper with loading everything
	void normalizeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::string path_to_cloud, std::string cloud_topic, pcl::PointCloud<pcl::PointXYZI>::Ptr normalized, cv_bridge::CvImagePtr table_ptr, float z_offset);
	// Wrapper which includes lookup table loading
	void normalizeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr normalized, std::string path_to_image, float z_offset);
	// Wrapper which includes cloud and lookup table loading
	void normalizeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::string path_to_cloud, std::string cloud_topic, pcl::PointCloud<pcl::PointXYZI>::Ptr normalized, std::string path_to_image, float z_offset);


};

/*
class LaserIntensityCalibration
{
private:
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr normalized_cloud_;
	cv_bridge::CvImagePtr table_ptr_;

public:
	LaserIntensityCalibration();

	setCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
	setImage(cv_bridge::CvImagePtr table);
	loadCloud(std::string bag_name, std::string bag_topic);
	loadImage(std::string path_to_image);
	setZOffset(float z_off);

	getInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
	getNormalizedCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr normalized);
	getLookupTable(cv_bridge::CvImagePtr table);

	normalizeCloud();
	normalizeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr normalized);
	normalizeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr normalized);
};

*/
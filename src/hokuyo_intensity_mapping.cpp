
// ROS Stuff
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// Rosbag Stuff
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Std Stuff
#include <iostream>
#include <fstream>
#include <istream>

// PCL Stuff
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>

// Seg
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


void processCloud(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_list, std::vector<Eigen::Vector3f> plane_normals, std::vector<float>plane_dists, std::vector<float> *angle_list, std::vector<float> *dist_list, std::vector<float> *wall_depth_error_list, std::vector<float> *depth_error_list)
{	
	for(int i=0; i<cloud_list.size(); i++)    
	{
		for(int j=0; j<cloud_list[i]->points.size(); j++)
		{
			Eigen::Vector3f point_normal;
			Eigen::Vector3f vector_to_point;

			//point_normal << 	output->points[i].normal_x, output->points[i].normal_y, output->points[i].normal_z;
			point_normal = plane_normals[i] / plane_normals[i].dot(plane_normals[i]);	// Check to normalize normals
			vector_to_point << 	cloud_list[i]->points[j].x, 		cloud_list[i]->points[j].y, 		cloud_list[i]->points[j].z;

			float distance;
			distance = sqrt(vector_to_point.dot(vector_to_point));

			float normal_mag; 	// this should hopefully already be 1, but to be safe...
			normal_mag = sqrt(point_normal.dot(point_normal));

			float wall_depth_error;
			wall_depth_error = fabs(vector_to_point.dot(point_normal)) - fabs(plane_dists[i]);

			float depth_error = distance/plane_dists[i] * wall_depth_error;

			float angle;
			angle = acos( point_normal.dot(vector_to_point) / normal_mag / distance );

			if(angle > 1.570796)
				angle = 3.141593 - angle;

			dist_list->push_back(distance);
			angle_list->push_back(angle);
			wall_depth_error_list->push_back(wall_depth_error);
			depth_error_list->push_back(depth_error);
		}
	}
	ROS_INFO_STREAM("[HokuyoIntensityCalibration] Performed cloud processing. Generated " << dist_list->size() << " points.");
}


void findHoughCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, std::vector<Eigen::Vector3f> wall_coeffs, std::vector<float> wall_dists, pcl::PointCloud<pcl::PointXYZI>::Ptr hough_cloud, float min_roll, float max_roll, float roll_increment, float min_pitch, float max_pitch, float roll, float pitch)
{
	// Make vector of roll values, from MIN_ROLL to MAX_ROLL at ROLL_INCREMENT 
	std::vector<float> rolls;
	rolls.push_back(min_roll);
	while(rolls[rolls.size()-1] < max_roll)
		rolls.push_back( rolls[rolls.size()-1] + roll_increment );
	// Generate Hough Output
	for(int i=0; i<input_cloud->points.size(); i++)
	{
		for(int j=0; j<rolls.size(); j++)
		{
						
		}
	}
}


// Input is PointCloud and list of angles and distances, output is .csv file of X, Y, Z, Intensity, Angle, Distance values
void printScan(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud, std::vector<float> angle_list, std::vector<float> dist_list, std::vector<float> wall_depth_error_list, std::vector<float> depth_error_list, std::string output_file_name)
{	
	std::ofstream output_file;
	output_file.open (output_file_name);
	for(int i=0; i<pointcloud->points.size(); i++)
	{
		output_file << pointcloud->points[i].x << ", " << pointcloud->points[i].y << ", " << pointcloud->points[i].z;
		output_file << ", " << pointcloud->points[i].intensity;
		output_file << ", " << angle_list[i];
		output_file << ", " << dist_list[i];
		output_file << ", " << wall_depth_error_list[i];
		output_file << ", " << depth_error_list[i];
		output_file << "\n";
	}
	output_file.close();
	ROS_INFO_STREAM("[HokuyoIntensityCalibration] Finished printing output file with name " << output_file_name);
}
// Input is PointCloud, output is .csv file of X, Y, Z values
void printScan(pcl::PointCloud<pcl::PointXYZ>::Ptr data_cloud, std::string output_file_name)
{
	std::ofstream output_file;
	output_file.open (output_file_name);
	for(int i=0; i<data_cloud->points.size(); i++)
	{
		output_file << data_cloud->points[i].x << ", " << data_cloud->points[i].y << ", " << data_cloud->points[i].z;
		output_file << "\n";
	}
	output_file.close();
	ROS_INFO_STREAM("[HokuyoIntensityCalibration] Finished printing output file with name " << output_file_name);
}
// Input is list of wall coefficients
void printPlanes(std::vector<Eigen::Vector3f> wall_coeffs, std::vector<float> wall_dists, std::string output_file_name)
{
	std::ofstream output_file;
	output_file.open (output_file_name);
	for(int i=0; i<wall_coeffs.size(); i++)
	{
		output_file << (wall_coeffs[i])[0] << ", ";
		output_file << (wall_coeffs[i])[1] << ", ";
		output_file << (wall_coeffs[i])[2] << ", ";
		output_file << wall_dists[i];
		output_file << "\n";
	}
	output_file.close();
	ROS_INFO_STREAM("[HokuyoIntensityCalibration] Finished printing output file with name " << output_file_name);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hokuyo_calibration");

//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//    ros::console::notifyLoggerLevelsChanged();

	ros::NodeHandle nh;

	// Data Container Declarations
	std::vector<sensor_msgs::PointCloud2> input_msg_list;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> input_cloud_list;
	pcl::PointCloud<pcl::PointXYZI>::Ptr entire_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	sensor_msgs::PointCloud2 entire_cloud_msg;
	std::vector<float> angle_list;
	std::vector<float> dist_list;
	std::vector<float> wall_depth_error_list;
	std::vector<float> depth_error_list;


	// -----------------------------------
	// -----------------------------------
	// Load all clouds
	// --- Input Parameters ---
	std::string bag_name_prefix;
	if(!nh.getParam("/hokuyo_intensity_mapping/bag_prefix", bag_name_prefix))
	{
		ROS_ERROR_STREAM("Failed to get bag prefix from parameter server. Exiting...");
		return -1;
	}
	std::vector<std::string> bag_name_list;
	if(!nh.getParam("/hokuyo_intensity_mapping/bag_name_list", bag_name_list))
	{
		ROS_ERROR_STREAM("Failed to get bag name list from parameter server. Exiting...");
		return -1;
	}
	std::string bag_topic;
	nh.param<std::string>("/hokuyo_intensity_mapping/bag_topic", bag_topic, "first_wall");


	// --- Actual Loading ---
	bool filter_inputs, voxelize_inputs;
	float filter_value, input_voxel_leaf_size;
	nh.param<bool>("hokuyo_intensity_mapping/filter_inputs", filter_inputs, true);
	nh.param<float>("hokuyo_intensity_mapping/filter_value", filter_value, 0.0);
	nh.param<bool>("hokuyo_intensity_mapping/voxelize_inputs", voxelize_inputs, true);
	nh.param<float>("hokuyo_intensity_mapping/input_voxel_leaf_size", input_voxel_leaf_size, 0.01);
	for(int i=0; i<bag_name_list.size(); i++)
	{
		sensor_msgs::PointCloud2 input_msg;
		rosbag::Bag bag; 
		std::string current_bag_name = bag_name_prefix + bag_name_list[i] + ".bag"; //boost::lexical_cast<std::string>(bag_name_list[i]) + ".bag";
		ROS_INFO_STREAM("[HokuyoIntensityCalibration] Attempting to load cloud from bag file.");
		ROS_INFO_STREAM("[HokuyoIntensityCalibration]   Bag file " << i << ":  " << current_bag_name << " with topic " << bag_topic);

		bag.open(current_bag_name, rosbag::bagmode::Read);

		std::vector<std::string> topics;
		topics.push_back(bag_topic);
		rosbag::View view_cloud(bag, rosbag::TopicQuery(topics));

		BOOST_FOREACH(rosbag::MessageInstance const m, view_cloud)
	    {
	        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
	        if (cloud_ptr != NULL)
	            input_msg = *cloud_ptr;
	        else
	        	ROS_ERROR_STREAM("[HokuyoIntensityCalibration] Cloud retrieved from bag is null...");
	    }
	    bag.close(); 
	    ROS_INFO_STREAM("[HokuyoIntensityCalibration]   Loaded a cloud of size " << input_msg.height*input_msg.width);		
	    
	    input_msg_list.push_back(input_msg);
	    pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	    pcl::fromROSMsg(input_msg, *new_cloud);
	    
	    if(filter_inputs)
	    {
		    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
		    for(int j=0; j<new_cloud->points.size(); j++)
		    	if(new_cloud->points[j].x > filter_value) filtered_cloud->points.push_back(new_cloud->points[j]);
	    	ROS_INFO_STREAM("[HokuyoIntensityCalibration] Filtered input cloud at value " << filter_value << " from size " << new_cloud->points.size() << " to " << filtered_cloud->points.size());
		    *new_cloud = *filtered_cloud;
	    }
	    if(voxelize_inputs)
	    {
	    	pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	    	pcl::VoxelGrid<pcl::PointXYZI> vg_xyz;
			vg_xyz.setInputCloud(new_cloud);
			float input_voxel_leaf_size;
			nh.param<float>("hokuyo_intensity_mapping/input_voxel_leaf_size", input_voxel_leaf_size, 0.01);
			vg_xyz.setLeafSize(input_voxel_leaf_size, 10, input_voxel_leaf_size);
			// Apply Filter and return Voxelized Data
			vg_xyz.filter(*filtered_cloud);
	    	ROS_INFO_STREAM("[HokuyoIntensityCalibration] Voxelized input cloud at leaf size " << input_voxel_leaf_size << " from size " << new_cloud->points.size() << " to " << filtered_cloud->points.size());
			*new_cloud = *filtered_cloud;
	    }

	    pcl::toROSMsg(*new_cloud, input_msg_list[i]);
	    input_msg_list[i].header.frame_id = "map";
	    
	    input_cloud_list.push_back(new_cloud);
	    *entire_cloud += *new_cloud;

	}
	    ROS_INFO_STREAM("[HokuyoIntensityCalibration] Entire cloud size: " << entire_cloud->points.size());	
	
	float z_offset;
	nh.param<float>("hokuyo_intensity_mapping/z_offset", z_offset, -0.3);
	for(int i=0; i<entire_cloud->points.size(); i++)
		entire_cloud->points[i].z += z_offset;

	pcl::toROSMsg(*entire_cloud, entire_cloud_msg);
	entire_cloud_msg.header.frame_id = "map";

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	std::vector<Eigen::Vector3f> plane_coeffs_list;
	std::vector<float> plane_dists;
	Eigen::Vector3f first_normal;
	for(int i=0; i<input_cloud_list.size(); i++)
	{
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZI> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (10000);
		seg.setDistanceThreshold (0.05);
		seg.setInputCloud (input_cloud_list[i]);
		ROS_INFO_STREAM("[HokuyoIntensityCalibration] Performing planar RANSAC segmentation.");
		seg.segment (*inliers, *coefficients);
		ROS_INFO_STREAM("[HokuyoIntensityCalibration]   Coeffs: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " 
								   << coefficients->values[3]);
		Eigen::Vector3f plane_coeffs;
		plane_coeffs << coefficients->values[0], coefficients->values[1], coefficients->values[2];
		if(i==0) first_normal = plane_coeffs;
		float angle_off = acos(plane_coeffs.dot(first_normal));
		ROS_INFO_STREAM("[HokuyoIntensityCalibration]   Offset: " << angle_off << ", from difference " << plane_coeffs[0]-first_normal[0] << " " << plane_coeffs[1]-first_normal[1] << " " << plane_coeffs[2]-first_normal[2]);
		plane_coeffs_list.push_back(plane_coeffs);
		plane_dists.push_back(coefficients->values[3]);
	}

	// -----------------------------------
	// -----------------------------------
	// Process Clouds
	processCloud(input_cloud_list, plane_coeffs_list, plane_dists, &angle_list, &dist_list, &wall_depth_error_list, &depth_error_list);

	pcl::PointCloud<pcl::PointXYZ>::Ptr intensity_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	for(int i=0; i<entire_cloud->points.size(); i++)
	{
		pcl::PointXYZ point;
		point.x = angle_list[i];
		point.y = dist_list[i];
		point.z = entire_cloud->points[i].intensity / 1000;
		intensity_cloud->points.push_back(point);
	}
	sensor_msgs::PointCloud2 intensity_msg;
	pcl::toROSMsg(*intensity_cloud, intensity_msg);
	intensity_msg.header.frame_id = "map";

	pcl::PointCloud<pcl::PointXYZ>::Ptr error_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	for(int i=0; i<entire_cloud->points.size(); i++)
	{
		pcl::PointXYZ point;
		point.x = angle_list[i];
		point.y = dist_list[i];
		point.z = wall_depth_error_list[i]*10;
		error_cloud->points.push_back(point);
	}
	sensor_msgs::PointCloud2 error_msg;
	pcl::toROSMsg(*error_cloud, error_msg);
	error_msg.header.frame_id = "map";



	pcl::VoxelGrid<pcl::PointXYZ> vg_xyz;
	vg_xyz.setInputCloud(intensity_cloud);
	float angle_pixel_size, distance_pixel_size, intensity_pixel_size;
	nh.param<float>("hokuyo_intensity_mapping/angle_pixel_size", angle_pixel_size, 0.005);
	nh.param<float>("hokuyo_intensity_mapping/distance_pixel_size", distance_pixel_size, 0.005);
	nh.param<float>("hokuyo_intensity_mapping/intensity_pixel_size", intensity_pixel_size, 10000);
	vg_xyz.setLeafSize(angle_pixel_size, distance_pixel_size, intensity_pixel_size);
	// Apply Filter and return Voxelized Data
	pcl::PointCloud<pcl::PointXYZ>::Ptr intensity_cloud_voxelized(new pcl::PointCloud<pcl::PointXYZ>());
	vg_xyz.filter(*intensity_cloud_voxelized);
	sensor_msgs::PointCloud2 intensity_msg_voxelized;
	pcl::toROSMsg(*intensity_cloud_voxelized, intensity_msg_voxelized);
	intensity_msg_voxelized.header.frame_id = "map";
	ROS_INFO_STREAM("[HokuyoIntensityCalibration] Voxelized data cloud from size " << intensity_cloud->points.size() << " to " << intensity_cloud_voxelized->points.size());

	// -----------------------------------
	// -----------------------------------
	// Print Outputs
	std::string output_file_name;
	nh.param<std::string>("hokuyo_intensity_mapping/output_file_name", output_file_name, "/home/conor/ros_data/trees/Velodyne/Intensity Calibration/hokuyo_calibration/data");
	printScan(entire_cloud, angle_list, dist_list, wall_depth_error_list, depth_error_list, output_file_name + ".csv");
	printScan(intensity_cloud_voxelized, output_file_name + "_voxelized.csv");
	printPlanes(plane_coeffs_list, plane_dists, output_file_name + "_planes.csv");
	ROS_INFO_STREAM("[HokuyoIntensityCalibration] Finished all work, saved output files.");

	ros::Publisher entire_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("entire_cloud", 1);
	ros::Publisher intensity_pub = nh.advertise<sensor_msgs::PointCloud2>("intensity_cloud", 1);
	ros::Publisher intensity_voxelized_pub = nh.advertise<sensor_msgs::PointCloud2>("intensity_cloud_voxelized", 1);
	ros::Publisher error_pub = nh.advertise<sensor_msgs::PointCloud2>("error_cloud", 1);

	while(ros::ok())
	{
		entire_cloud_pub.publish(entire_cloud_msg);
		intensity_pub.publish(intensity_msg);
		intensity_voxelized_pub.publish(intensity_msg_voxelized);
		error_pub.publish(error_msg);
		ros::Duration(1.0).sleep();
	}

}




#include <laser_stitcher/intensity_calibration.h>

namespace LaserIntensityCalibration
{


// Double Interpolation in Image
float interpRaster(cv_bridge::CvImagePtr image_ptr, int j, int i, float x_frac, float y_frac)
{
	float f_11 = image_ptr->image.at<uchar>(j,  i);
	float f_12 = image_ptr->image.at<uchar>(j+1,i);
	float f_21 = image_ptr->image.at<uchar>(j,  i+1);
	float f_22 = image_ptr->image.at<uchar>(j+1,i+1);

    if(f_12 == 0)
        return f_11;
    if(f_21 == 0)
        return f_11;
    if(f_22 == 0)
        return f_11;

//    return f_11;

    return (1-x_frac) * (1-y_frac) * f_11 +
           x_frac     * (1-y_frac) * f_21 +
           (1-x_frac) * y_frac     * f_12 +
           x_frac     * y_frac     * f_22;

//	return  ( (1-y_frac)*f_11 + y_frac*f_12 ) * (1-x_frac) +
//			( (1-y_frac)*f_21 + y_frac*f_22 ) * x_frac;
}


// Lookup a value f' in image table I for a given point f(x,y)
//   Might change this to interpolation at some point, but for now this probably works fine
//   Here, this is used to look up intensity scaling values based on geometry, but could be used for other purposes also
//   DIMENSIONS is a 4-valued vector containing {X_MIN, Y_MIN, X_MAX, Y_MAX} for the given image
float rasterLookup(cv_bridge::CvImagePtr image_ptr, float f, float x, float y, std::vector<float> dimensions)
{
	// Find horizontal index
	float x_pos = (x - dimensions[0]) / (dimensions[2]-dimensions[0]) * image_ptr->image.cols;
	if( x_pos >= image_ptr->image.cols )
	{
		ROS_WARN_STREAM_THROTTLE(0.1,"[IntensityCalibration] Warning - angle at " << x << ", " << y << " is out of dimension bounds: " << dimensions[0] << ", " << dimensions[1] << ", " << dimensions[2] << ", " << dimensions[3] << " with val " << x_pos);
		x_pos = image_ptr->image.cols - 2;
	}
	else if( x_pos < 0 )
	{
		ROS_WARN_STREAM_THROTTLE(0.1,"[IntensityCalibration] Warning - angle at " << x << ", " << y << " is out of dimension bounds: " << dimensions[0] << ", " << dimensions[1] << ", " << dimensions[2] << ", " << dimensions[3] << " with val " << x_pos);
		x_pos = 0;
	}

	// Find vertical index
	float y_pos = (y - dimensions[1]) / (dimensions[3]-dimensions[1]) * image_ptr->image.rows;
    bool dist_too_high = false;
	if( y_pos >= image_ptr->image.rows )
	{
		ROS_WARN_STREAM_THROTTLE(0.1,"[IntensityCalibration] Warning - dist at " << x << ", " << y << " is out of dimension bounds: " << dimensions[0] << ", " << dimensions[1] << ", " << dimensions[2] << ", " << dimensions[3] << " with val " << y_pos);
		y_pos = image_ptr->image.rows - 2;
        dist_too_high = true;
	}
	else if( y_pos < 0 )
	{
		ROS_WARN_STREAM_THROTTLE(0.1,"[IntensityCalibration] Warning - dist at " << x << ", " << y << " is out of dimension bounds: " << dimensions[0] << ", " << dimensions[1] << ", " << dimensions[2] << ", " << dimensions[3] << " with val " << y_pos);
		y_pos = 0;
	}

	int i = int(floor(x_pos));
	int j = int(floor(y_pos));
	float x_frac = x_pos - float(i);
	float y_frac = y_pos - float(j);


    float intensity_value = interpRaster(image_ptr, j, i, x_frac, y_frac);
    //ROS_ERROR_STREAM("normalizing... " << int(image_ptr->image.at<uchar>(j,  i)) << " " << int(image_ptr->image.at<uchar>(j+1,i)) << " " << int(image_ptr->image.at<uchar>(j,  i+1)) << " " << int(image_ptr->image.at<uchar>(j+1,i+1)) << " square " << intensity_value << " at " << i << " " << j << ", " << x_frac << " " << y_frac << ", " << x_pos << " " << y_pos);
    //ros::Duration(0.03).sleep();

    if(dist_too_high)
        return intensity_value * pow(dimensions[3],2) / pow(y,2);
    else 
        return intensity_value;
}


// Perform plane fit (RANSAC) to get plane normal
Eigen::Vector3f planeFit(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
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
    seg.setInputCloud (input);
    ROS_INFO_STREAM("[IntensityCalibration] Performing planar RANSAC segmentation.");
    seg.segment (*inliers, *coefficients);
    ROS_INFO_STREAM("[IntensityCalibration]   Coeffs: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " 
                               << coefficients->values[3]);
    Eigen::Vector3f wall_normal;
    wall_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];

    return wall_normal;
}



// Load Input Cloud to have its Intensity Normalized
void loadCloud(std::string path_to_cloud, std::string cloud_topic, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // ----------------------------- Pull in data -----------------------------
    ROS_INFO_STREAM("[IntensityCalibration] Loading clouds from bag files, using bag name: " << path_to_cloud << " and topic name: " << cloud_topic << ".");
    sensor_msgs::PointCloud2 input_msg;
    // Open Bag
    rosbag::Bag input_bag; 
    input_bag.open(path_to_cloud, rosbag::bagmode::Read);
    // Create Topic List
    std::vector<std::string> topics;
    topics.push_back(cloud_topic);
    rosbag::View view_dense(input_bag, rosbag::TopicQuery(topics));
    // Extract Cloud
    BOOST_FOREACH(rosbag::MessageInstance const m, view_dense)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
            input_msg = *cloud_ptr;
        else
          ROS_ERROR_STREAM("[IntensityCalibration] Cloud caught for first cloud is null...");
    }
    input_bag.close();
    ROS_INFO_STREAM("[IntensityCalibration] First cloud size: " << input_msg.height*input_msg.width);

    pcl::fromROSMsg(input_msg, *cloud);
}



// Load Intensity Lookup Table Image using OpenCV
void loadImage(std::string path_to_image, cv_bridge::CvImagePtr table_ptr)
{
    cv::Mat image = cv::imread(path_to_image, CV_LOAD_IMAGE_GRAYSCALE);
    image.copyTo(table_ptr->image);
    sensor_msgs::Image image_msg;
    table_ptr->toImageMsg(image_msg);
    image_msg.encoding = sensor_msgs::image_encodings::MONO8;

    ROS_INFO_STREAM("[IntensityCalibration] Loaded intensity calibration image of size " << table_ptr->image.rows << " by " << table_ptr->image.cols);    
}


// Actually perform normalization
void normalizeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr normalized, cv_bridge::CvImagePtr table_ptr, float z_offset=0.0)
{
    for(int i=0; i<input->points.size(); i++)
        input->points[i].z += z_offset;
    *normalized = *input;

    std::vector<float> dimensions;
    dimensions.push_back(0);
    dimensions.push_back(0);
    dimensions.push_back(1.58);
    dimensions.push_back(4.3);

    Eigen::Vector3f wall_normal = planeFit(input);

    for(int i=0; i<normalized->points.size(); i++)
    {
        Eigen::Vector3f point_vector;
        point_vector << normalized->points[i].x,
                        normalized->points[i].y,
                        normalized->points[i].z;

        float dist = sqrt(point_vector.dot(point_vector));
        
        float angle = acos(point_vector.dot(wall_normal) / dist);

        if(angle > 1.57)
            angle = 3.14159 - angle;

        if(std::isnan(angle))
        {
            ROS_ERROR_STREAM("encountered ang nan at " << point_vector[0] << " " << point_vector[1] << " " << point_vector[2] << " | " <<  wall_normal[0] << " " << wall_normal[1] << " " << wall_normal[2] << " | " << point_vector.dot(wall_normal) << " " << acos(point_vector.dot(wall_normal)) << " " << acos(point_vector.dot(wall_normal))/dist );
            continue;
        }
        if(std::isnan(dist))
        {
            ROS_ERROR_STREAM("encountered dist nan at " << point_vector[0] << " " << point_vector[1] << " " << point_vector[2] << " " << angle << " " << dist);
            continue;
        }

        normalized->points[i].intensity /= rasterLookup(table_ptr, 
                                                        normalized->points[i].intensity,
                                                        angle,
                                                        dist,
                                                        dimensions);
    }
}

// Wrapper which includes cloud loading
void normalizeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::string path_to_cloud, std::string cloud_topic, pcl::PointCloud<pcl::PointXYZI>::Ptr normalized, cv_bridge::CvImagePtr table_ptr, float z_offset=0.0)
{
    // Load Cloud
    //pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>());
    loadCloud(path_to_cloud, cloud_topic, input);
    // Normalize Intensity
    normalizeCloud(input, normalized, table_ptr, z_offset);
}


// Wrapper which includes lookup table loading
void normalizeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr normalized, std::string path_to_image, float z_offset=0.0)
{
    // Load Lookup table
    cv_bridge::CvImagePtr table_ptr(new cv_bridge::CvImage);
    loadImage(path_to_image, table_ptr);
    // Normalize Intensity
    normalizeCloud(input, normalized, table_ptr, z_offset);
}


// Wrapper which includes cloud and lookup table loading
void normalizeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::string path_to_cloud, std::string cloud_topic, pcl::PointCloud<pcl::PointXYZI>::Ptr normalized, std::string path_to_image, float z_offset=0.0)
{
    // Load Cloud Table
    //pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>());
    loadCloud(path_to_cloud, cloud_topic, input);
    // Load Lookup table
    cv_bridge::CvImagePtr table_ptr(new cv_bridge::CvImage);
    loadImage(path_to_image, table_ptr);
    // Normalize Intensity
    normalizeCloud(input, normalized, table_ptr, z_offset);
}




} // LaserIntensityCalibration


/*
int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "wall_change_tester");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();

  	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    ros::NodeHandle nh;

    std::string bag_name, bag_topic;
    nh.param<std::string>("intensity_calibration/bag_name", bag_name, "/home/conor/ros_data/Fake_Walls/Segmented/screwy/1dps.bag");
    nh.param<std::string>("intensity_calibration/bag_topic", bag_topic, "/target_wall");

    std::string lookup_table_name;
    nh.param<std::string>("intensity_calibration/lookup_table_name", lookup_table_name, "/home/conor/Downloads/intensity_lookup_table.png");

    float z_offset;
    nh.param<float>("intensity_calibration/z_offset", z_offset, -0.3);

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr normalized_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    LaserIntensityCalibration::normalizeCloud(input_cloud, bag_name, bag_topic, normalized_cloud, lookup_table_name, z_offset);

    ROS_INFO_STREAM("[IntensityCalibration] Performed calibration on input image size " << input_cloud->points.size() << " to " << normalized_cloud->points.size() << " using image " << lookup_table_name);

    sensor_msgs::PointCloud2 input_msg;
    pcl::toROSMsg(*input_cloud, input_msg);
    input_msg.header.frame_id = "map";    
    sensor_msgs::PointCloud2 normalized_msg;
    pcl::toROSMsg(*normalized_cloud, normalized_msg);
    normalized_msg.header.frame_id = "map";

    ros::Publisher input_pub = nh.advertise<sensor_msgs::PointCloud2>("intensity_calibration/input", 1);
    ros::Publisher normalized_pub = nh.advertise<sensor_msgs::PointCloud2>("intensity_calibration/normalized", 1);
    ros::Publisher table_image_pub = nh.advertise<sensor_msgs::Image>("intensity_calibration/calibration_image", 1);

    while(ros::ok())
    {
    	input_pub.publish(input_msg);
    	normalized_pub.publish(normalized_msg);
    	//table_image_pub.publish(image_msg);
    	ros::Duration(1.0).sleep();
    }
}

*/
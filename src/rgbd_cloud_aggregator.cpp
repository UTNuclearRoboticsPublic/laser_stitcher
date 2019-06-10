
#include <laser_stitcher/laser_stitcher.h>

main(int argc, char** argv)
{
	ros::init(argc, argv, "rgbd_cloud_aggregator");

	ros::NodeHandle nh;

	std::string scanning_state_topic;
	nh.param<std::string>("laser_stitcher/scanning_state_topic", scanning_state_topic, "laser_stitcher/scanning_state");
	ros::Publisher scanning_state_pub = nh.advertise<std_msgs::Bool>(scanning_state_topic, 1);  

	// How long should clouds be aggregated? 
	float wait_time;
	nh.param<float>("laser_stitcher/wait_time", wait_time, 5);

	ROS_INFO_STREAM("[RGBDScanAggregator] Beginning scan aggregation!");

	std_msgs::Bool scanning_state;
	scanning_state.data = true;
	
	ros::Duration(1.0).sleep();
	
	scanning_state_pub.publish(scanning_state);

	ros::Time start_time = ros::Time::now();
	while(ros::Time::now() < start_time + ros::Duration(wait_time))
	{
		//scanning_state_pub.publish(scanning_state);
		ros::spinOnce();
		ros::Duration(wait_time/20).sleep();
	}

	scanning_state.data = false;
	scanning_state_pub.publish(scanning_state);
	ros::spinOnce();

	// Commence operation!
	ROS_INFO_STREAM("[RGBDScanAggregator] Ended scan aggregation.");
	//ros::spin(); 
}
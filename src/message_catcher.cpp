
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>

std::string topic_;
bool finish_;

void topicCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	rosbag::Bag bag;
	bag.open("test.bag", rosbag::bagmode::Write);

	bag.write(topic_, ros::Time::now(), msg);

	bag.close();

	finish_ = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "message_catcher");

	ros::NodeHandle nh;

	nh.param<std::string>("message_catcher/topic", topic_, "front_camera/image_raw");

	ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>(topic_, 1, topicCallback);

	finish_ = false;
	while(!finish_)
	{
		ros::spinOnce();
	}
}
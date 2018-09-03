
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class LIDARFramePub
{
public:
	LIDARFramePub();
private:
	tf2_ros::TransformBroadcaster lidar_frame_broadcaster;
	geometry_msgs::TransformStamped lidar_transform_;
	void jointstateCallback(sensor_msgs::JointState jointstate);
};

LIDARFramePub::LIDARFramePub()
{
	//   Populate header fields of transform
    lidar_transform_.header.frame_id = "base_link";//parent_frame_name_;
  	lidar_transform_.child_frame_id = "hokuyo_lidar_base";//lidar_frame_name_;
  	//   Set Translation (fixed, here)
  	lidar_transform_.transform.translation.x = 0.0;
  	lidar_transform_.transform.translation.y = 0.0;
  	lidar_transform_.transform.translation.z = 19*0.0254;

  	ros::NodeHandle nh;
	ros::Subscriber jointstate_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &LIDARFramePub::jointstateCallback, this);

	ros::spin();
}

void LIDARFramePub::jointstateCallback(sensor_msgs::JointState jointstate)
{
	float pan_angle_ = jointstate.position[0];
	ROS_INFO_STREAM("entered here... " << pan_angle_);
	// Manually build the relevant frame for the LIDAR
	//   Build transform, set frames and time
  	lidar_transform_.header.stamp = jointstate.header.stamp;
  	//   Set Rotation (pan is dynamic)
  	tf2::Quaternion rotation;
  	rotation.setRPY(0.0, -1.5708, pan_angle_);
  	lidar_transform_.transform.rotation.x = rotation.x();
  	lidar_transform_.transform.rotation.y = rotation.y();
  	lidar_transform_.transform.rotation.z = rotation.z();
  	lidar_transform_.transform.rotation.w = rotation.w();
  	//   Broadcast it!
  	lidar_frame_broadcaster.sendTransform(lidar_transform_);
  	ROS_DEBUG_STREAM("Published a transform at: " << lidar_transform_);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arbotix_frame_publisher");

    // Uncomment the following to enable DEBUGGING output
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
      ros::console::notifyLoggerLevelsChanged();

  	LIDARFramePub pub_object;
}
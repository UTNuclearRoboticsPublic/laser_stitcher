#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <stdlib.h> 

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "is_running_true");
     ros::NodeHandle nh;

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);

     //Ceates the publisher, and tells it to publish
     ros::Publisher pub=nh.advertise<std_msgs::Bool>("laser_stitcher/scanning_state", 1);
     while(pub.getNumSubscribers() == 0)
        {rate.sleep();}

     //Sets up the random number generator
     srand(time(0));




            //Declares the message to be sent
            std_msgs::Bool msg;
            msg.data=true;

           //Publish the message
           pub.publish(msg);

          //Delays until it is time to send another message
          rate.sleep();

return 0;
}

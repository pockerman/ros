/**
  * Receiver node for Hello ROS example
  */
#include "ros/ros.h"
#include "std_msgs/String.h"

void print_msg_callback(const std_msgs::String::ConstPtr& msg){

    ROS_INFO("I received: %s ",msg->data.c_str());
}

int main(int argc, char** argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "node_2");

    // this is the actual node handler
    ros::NodeHandle node;

    // this is a subscriber node
    ros::Subscriber sub = node.subscribe("message", 1000, print_msg_callback );
    ros::spin();
    return 0;
}

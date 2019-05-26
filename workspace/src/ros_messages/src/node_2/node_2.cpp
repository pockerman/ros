/**
  * Receiver node for Hello ROS example
  */
#include "ros/ros.h"
#include "ros_messages/example.h"

void print_msg_callback(const ros_messages::example::ConstPtr& msg){

		ROS_INFO("Node name: %s ", msg->message.c_str());
    ROS_INFO("Counter a is: %d ",msg->a);
		ROS_INFO("Counter b is: %d ",msg->b);
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

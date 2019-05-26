/**
  * Publisher node for Hello ROS example
  */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char** argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "node_1");

    // this is the actual node handler
    ros::NodeHandle n;

    // tell the master the name of the topic and the type
    // The name is message and the second parameter is the buffer size

    ros::Publisher pub = n.advertise<std_msgs::String>("message", 1000);

    // set the frequency to send the data to 10Hz
    ros::Rate loop_rate(10);

    while(ros::ok()){

        std_msgs::String msg;
        std::stringstream ss;
        ss << " Hello ROS ";
        msg.data = ss.str();
        //ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

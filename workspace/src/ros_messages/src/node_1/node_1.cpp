/**
  * Publisher node for Hello ROS example
  */

#include "ros/ros.h"
#include "ros_messages/example.h" // the custom message

int main(int argc, char** argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "node_1");

    // this is the actual node handler
    ros::NodeHandle n;

    // tell the master the name of the topic and the type
    // The name is message and the second parameter is the buffer size
    ros::Publisher pub = n.advertise<ros_messages::example>("message", 1000);

    // set the frequency to send the data to 10Hz
    ros::Rate loop_rate(10);
		unsigned int counter = 0;
    while(ros::ok()){

				ros_messages::example msg;
        msg.message = "This is Publisher Node";
				msg.a = counter;
				msg.b = counter + 1;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

				if(counter == 100){

					counter = 0;

				}

				counter++;
    }
    return 0;
}

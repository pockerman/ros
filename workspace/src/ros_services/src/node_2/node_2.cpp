#include "ros/ros.h"
#include "ros_services/word_count.h"
#include "ros_services/word_countRequest.h"



int main(int argc,char **argv){

//the name of the node this must be unique
std::string node_name="Client";
ros::init(argc, argv, node_name);

ros::NodeHandle n;
ros::ServiceClient client = n.serviceClient<ros_services::word_count>("Speak");

ros_services::word_count srv;
srv.request.words ="This is my very long message send to the outter space by Alex";

if(client.call(srv)){
		ROS_INFO("Sum: %ld", (long int)srv.response.count);
}
else{
		ROS_ERROR("Failed to call service Speak");
		return 1;
	}

return 0;

}

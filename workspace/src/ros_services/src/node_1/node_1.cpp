#include "ros/ros.h"
#include "ros_services/word_count.h"
#include "ros_services/word_countRequest.h"
#include <boost/algorithm/string.hpp> //for boost::split
#include <vector>
#include <string>

std::vector<std::string> split_msg(const std::string& msg){

	std::vector<std::string> words;
	boost::split(words, msg, boost::is_any_of(" "));
	return words;

}

//see also this ROS example http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
bool speak(ros_services::word_count::Request& req, ros_services::word_count::Response& res){
	
	std::string msg(req.words);
	auto words = ::split_msg(msg);
	res.count = words.size();
  ROS_INFO("Request words [%s]: ",msg.c_str());
	ROS_INFO("Request count [ %d ]: ",res.count);
	return true;
} 

int main(int argc,char **argv){

	//the name of the node this must be unique
	std::string node_name="Server";

	//initialize ROS with the node name
	ros::init(argc,argv,node_name);

	//create node. This is the handler for the process
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("Speak", speak);
	ROS_INFO("Ready to speak...");

	ros::spin();

return 0;
}

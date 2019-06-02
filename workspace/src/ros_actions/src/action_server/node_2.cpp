#include "ros/ros.h"
#include "ros_actions/StartMotorAction.h"
#include "ros_actions/StartMotorActionFeedback.h"
#include "actionlib/server/simple_action_server.h"

#include <vector>
#include <string>

typedef actionlib::SimpleActionServer<ros_actions::StartMotorAction> Server;

// the feedback we want to send
ros_actions::StartMotorFeedback feedback;

void start_motor(const ros_actions::StartMotorGoalConstPtr& goal, Server* server){

	
  // publish info to the console for the user
	ROS_INFO("Executing start_motor, for motor id  %d with speed %d", goal->motor_id, goal->motor_speed);

	/// check which motor to start
	if(goal->motor_id == 0){

		// get the speed
		auto speed = goal->motor_speed;

		// set the speed and set the feedback message also
		::feedback.feedback_msg = std::move(std::string("Engine with ID 0 started"));
		server->publishFeedback(::feedback);
	}
	else{

		ROS_INFO("The motor id given is %d", goal->motor_id);
	}
}

int main(int argc,char **argv){

	//the name of the node this must be unique
	std::string node_name="ActionServer";

	//initialize ROS with the node name
	ros::init(argc,argv,node_name);

  ros::NodeHandle n;
  Server server(n, "start_motor", boost::bind(&start_motor, _1, &server), false);
  server.start();
  ros::spin();

	return 0;
}

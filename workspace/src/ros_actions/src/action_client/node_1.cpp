#include "ros/ros.h"
#include "ros_actions/StartMotorAction.h"
#include "ros_actions/StartMotorActionGoal.h"
#include "ros_actions/StartMotorGoal.h"
#include "actionlib/client/simple_action_client.h"

#include <boost/algorithm/string.hpp> //for boost::split
#include <vector>
#include <string>

typedef actionlib::SimpleActionClient<ros_actions::StartMotorAction> Client;

int main(int argc,char **argv){

	//the name of the node this must be unique
	std::string node_name="ActionClient";

	//initialize ROS with the node name
	ros::init(argc,argv,node_name);

	// let's tell the robot to start the motor
	// we can't go anywhere without this 

	Client client("start_motor", true); // true -> don't need ros::spin()

	// the goal
	ros_actions::StartMotorGoal goal;
	goal.motor_id = 0; //the zeroth motor to start
	goal.motor_speed = 255; //the speed for the motor

	//send the goal
	client.sendGoal(goal);

	// wait for 5 secs 
	client.waitForResult(ros::Duration(5.0));

	//test for success
	if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		std::cout<<"Motor started..."<<std::endl;
	}
	else{
		std::cout<<"After 5 secs the motor could not be started..."<<std::endl;
	}

return 0;
}

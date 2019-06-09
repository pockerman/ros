# Item: ROS Actions

#### References

1. **```actionlib```:** http://wiki.ros.org/actionlib
2. **```actionlib``` Detailed Description:** http://wiki.ros.org/actionlib/DetailedDescription

## ROS Actions

Item <a href="ros_services.md">ROS Services</a> introduces a request/response mechanism for communication between ROS nodes.
The downside of this approach is that this is a blocking operation meaning there is no progress until the computation
initiated via the request is completed. In some cases, however, if the service takes a long time to execute,  we would like to have the ability to cancel the request during execution or get periodic feedback about how the request is progressing. 
The ```actionlib``` package provides tools to create servers that execute long-running goals that can be preempted. 
It also provides a client interface in order to send requests to the server [1]. 

Just like messages and services, actions are messages that have a similar format like service messages. 
Likewise, action messages are placed in ```./action``` directory of the package that uses the actions.

A well presented discussion about ROS actions can be found in [2]. Thus, in this item we will simply show how to
create a ROS action.

ROS actions involve an ```ActionClient``` and an ```ActionServer```. They communicate via a _ROS Action Protocol_ [1].
The latter is build on top of ROS messages. In order for the client and server to communicate, we need to define a few messages on which they communicate. This is with an action specification. This defines the Goal, Feedback, and Result messages with which clients and servers communicate [1].

#### Goal

The goal can be sent to the ```ActionServer``` by the ```ActionClient```. For example, in the case of moving the base, the goal would be a ```PoseStamped``` message that contains information about where the robot should move to in the world [1]. 

#### Feedback

Feedback provides server implementers a way to tell an ```ActionClient``` about 
the incremental progress of a goal. For moving the base, this might be the robot's current pose along the path.

#### Result
A result is sent from the ```ActionServer``` to the ```ActionClient``` upon completion of the goal. 
This is different than feedback, since it is sent exactly once. 
This is extremely useful when the purpose of the action is to provide some sort of information. 
For move base, the result isn't very important, but it might contain the final pose of the robot.

### Implementing ROS Action

Just like services and messages, the action specification is defined using a ```.action``` file. 
The ```.action``` file has the goal definition, followed by the result definition, followed by the feedback definition. Each section separated by 3 hyphens ```(---)```.

These files are placed in a package's ```./action``` directory.  As an example, consider an action that starts the motor of a robot

```
# define the goal
uint32 motor_id
float32 motor_speed
---

# define the result
uint8 started # zero for success not zero failure
---

# define the feedback message
string msg

``` 

Based on this ```.action``` file,   messages have to be generated in order for the client and server to communicate. 
This generation can be automatically triggered during the make process [1]. Let's see how 

Create a new package and name it ```ros_services```:

```
cd catkin_ws/src
catkin_create_pkg ros_actions std_msgs rospy roscpp
cd ..
catkin_make
```

Create an ```./action``` directory and add the ```StartMotor.action``` file with the following contents

```
# define the goal
uint32 motor_id
float32 motor_speed
---

# define the result
uint8 started # zero for success not zero failure
---

# define the feedback message
string msg

``` 

#### Update ```package.xml``` 

We also need to update our ```package.xml``` file for our new package. Edit the relevant file and add the following two lines

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

#### Update ```CMakeLists.txt```

Next, we need to make a few changes to the ```CMakeLists.txt``` file of our package. 


- Add the actions files

```
## Generate actions in the 'action' folder
add_action_files(
   FILES 
	 StartMotor.action
)

```

- Generate the action messages

```

generate_messages(
   DEPENDENCIES
   actionlib_msgs   std_msgs
 )

catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

```

- Add the executable names and link

```
add_executable(${PROJECT_NAME}_client_node src/action_client/node_1.cpp)
add_executable(${PROJECT_NAME}_server_node src/action_server/node_2.cpp)
target_link_libraries(${PROJECT_NAME}_client_node ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_server_node ${catkin_LIBRARIES})
```

Overall our ```CMakeLists.txt``` file when stripped off comments should look like the following:

```
cmake_minimum_required(VERSION 2.8.3)
project(ros_actions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

add_action_files(
   FILES 
	 StartMotor.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs   std_msgs
 )

catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_client_node src/action_client/node_1.cpp)
add_executable(${PROJECT_NAME}_server_node src/action_server/node_2.cpp)
target_link_libraries(${PROJECT_NAME}_client_node ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_server_node ${catkin_LIBRARIES})

```

### Build 

Now that we have everything set, we nee to build the our package

```
cd catkin_ws
catkin_make
```


The process above will create some header files for us. 
Concretely, navigate to ```catkin_ws/devel/include/ros_actions/``` and you will see that there are seven header files create for us.
These are the headers we need to use in our application.



### Run 

Start the master node

```
roscore
``` 

Open a new terminal and type

```
rosrun ros_actions ros_actions_client_node 
```

Similarly for the server node.


---
**REMARK**

Make sure that the termoinals that execute the two nodes that you executed

```
source /path/to/your/catkin_ws/devel/setup.sh
```

---


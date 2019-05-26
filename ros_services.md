# Item: ROS Services

#### References

1. **ROS services:** http://wiki.ros.org/Services
2. **Writing a Simple Service and Client (C++):** http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

ROS services are also discussed in the videos below

## ROS Services

Item <a href="ros_messages.md">ROS messages</a> discusses ROS messages; a mechanism for node communication in  a ROS based application.
In order to do so, nodes have to subscribe to the topics they are interested in. 
Each topic advertises a message that the subscribed node can read. ROS topics in this sense play the role of a many-to-many one-way communication infrastructure. 

ROS services, which are discussed in this Item, is another way to pass data between nodes in ROS. 
The following videos summarize many of the topics touched herein.

[![](http://img.youtube.com/vi/feXC7aQrkeM/0.jpg)](https://www.youtube.com/watch?v=feXC7aQrkeM "Programming for Robotics (ROS) Course 4")

[![](http://img.youtube.com/vi/bKXahdK2yxA/0.jpg)](https://www.youtube.com/watch?v=bKXahdK2yxA "[ROS Tutorials] Chapter 3.1: ROS Services + programming Wam robot-arm (Python)")

[![](http://img.youtube.com/vi/KnoIJq7n3m4/0.jpg)](https://www.youtube.com/watch?v=KnoIJq7n3m4 "ROS BASICS IN 5 DAYS #7 - ROS Services | Part2")


---
**REMARK**

There is no acknowledgement involved when communicating over topics.

---


ROS services are just synchronous procedure calls which are done remotely. They allow one node to call a function that executes in another node. 
We can define the input and output of the function in a similar way we can define new message types (see Item <a href="ros_messages.md">ROS messages</a>).
The server (which provides the service) specifies a callback to deal with the service request, and advertises the service. 
The client (which calls the service) then accesses this service through a local proxy. 
Hence, ROS services correspond to a request-response communication type. Consequently, ROS services require two message types;  a request and a response.


ROS service calls can be used when the application needs only occasionally to compute something. 
Furthermore, the triggered computation takes a bounded amount of time to complete.  some examples of services
are turning on a sensor or taking a high-resolution picture with a camera.


---
**REMARK**

ROS services block the program flow. What this means is that the program will not proceed any further unitl the ROS service has been completed. Thus, it is desirable to have computations that don't last long is a service callback.

---


### Implementing ROS Service

Request/response communication between nodes is realized with ROS services:

- ROS service server advertises the service
- The service client accesses this service

Just like ROS messages, ROS services are defined in ```*.srv``` files. In addition, they are similar to messages in structure. 
They are defined in ```/srv``` directory in the package that implements them.

Here is a very simple example of a service that takes in an ```int``` and returns a ```double```:

```
int a
---
double b

```

A service description file consists of a request and a response msg type, separated by ```---```. 
Any two ```.msg``` files concatenated together with a ```---``` are a legal service description.  The inputs to the service call come first. 

---

**Remark**

We cannot embed another ```.srv``` inside of a ```.srv```. 

---

Create a new package and name it ```ros_services```:

```
cd catkin_ws/src
catkin_create_pkg ros_services std_msgs rospy roscpp
cd ..
catkin_make
```


Just like messages are stored in the ```msg``` directory, services, are strored in the ```srv``` directory. 
So let's create a ```srv``` dirctory into our package. 
Within the directory, we create the ```word_count.srv``` file which has the following structure

```
string words
---
uint32 count
```


Now we need to build our package again. Let's do this.

#### Update ```package.xml``` 

We also need to update our ```package.xml``` file for our new package. Edit the relevant file and add the following two lines

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```


#### Update ```CMakeLists.txt```

Next, we need to make a few changes to the ```CMakeLists.txt``` file of our package. This is a bit involved but not overly difficult


- add ```message_generation``` to the end of the ```find_package()``` call , so that ```catkin``` knows to
look for the message_generation package:

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	message_generation
)
```

- Tell ```catkin``` which services we’re going to use:

```
add_service_files(
   FILES
   word_count.srv
)

```

- Make sure the ```generate_messages()``` call is uncommented and contains all the dependencies that are needed by our messages:

```
generate_messages(
DEPENDENCIES
std_msgs
)
```

Overall our ```CMakeLists.txt``` file when stripped off comments should look like the following:

```
cmake_minimum_required(VERSION 2.8.3)
project(ros_services)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	message_generation
)

add_service_files(
   FILES
   word_count.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node_1  src/node_1/node_1.cpp)
add_executable(${PROJECT_NAME}_node_2  src/node_2/node_2.cpp)
target_link_libraries(${PROJECT_NAME}_node_1 ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node_2 ${catkin_LIBRARIES})

```

---

**Remark**

Check the code in the ```src/ros_services/node_1``` and ```src/ros_services/node_2``` 

---


### Build 

Now that we have everything set, we nee to build the our package

```
cd catkin_ws
catkin_make
```


The process above will create some header files for us. 
Concretely, navigate to ```catkin_ws/devel/include/ros_services/``` and you will see that there are three header files; ```word_count.h```, ```word_countRequest.h``` and ```word_countResponse.h```.
These are the headers we need to use in our application in order to have access to the service.


### Run 

Start the master node

```
roscore
``` 

Open a new terminal and type

```
rosrun ros_services ros_services_node_1 
```

Similarly for node 2.

---
**REMARK**

Make sure that the termoinals that execute the two nodes that you executed

```
source /path/to/your/catkin_ws/devel/setup.sh
```

---



### ROS Service Commands

Prin t the list of running ROS services in an application

```
rosservice list
```

Call a ROS service

```
rosservice call <service_name> <arguments_req>
```

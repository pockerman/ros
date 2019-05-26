# Item: ROS Messages

#### References

1. **ROS Messages:** http://wiki.ros.org/Messages
2. **```std_msgs```:** http://wiki.ros.org/msg#Field_Types?distro=kinetic
3. **```common_msgs```:** http://wiki.ros.org/common_msgs?distro=kinetic
4. **Creating a ROS msg and srv:** http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv


## ROS Messages

ROS applications are typically based on executables that run on distinct processes. When a process, say ```p_1```, needs data from another one, say ```p_2```,
then these two processes need a way to communicate with each other. This is done via messages. Thus, we need to see how we can create application specific messages in ROS.
 
Item <a href="ros_introduction.md">ROS Introduction</a> mentioned that ROS nodes communicate over topics. 
This however is just an abstraction. A topic acquires or inherits the ROS message type that is to be exchanged between the nodes. 

For example in Item <a href="ros_hello.md">Hello ROS</a> we shows you how to develop a 
simple Publisher/Subscriber application that uses a standard string, namely ```std_msgs::String```, as the exchanged message between the two nodes.
Furthermore, the topic is called simply ```message``` therein. To put things more into perspective, the relevant code is summarized below


```
/// Publisher 


ros::Publisher pub = n.advertise<std_msgs::String>("message", 1000);

...

std_msgs::String msg;
std::stringstream ss;
ss << " Hello ROS ";
msg.data = ss.str();
pub.publish(msg);


/// Subscriber

 ros::Subscriber sub = node.subscribe("message", 1000, print_msg_callback );

```

---

**Remark**

Only one message type can be published to a given topic.

---

Needless to say, robotic applications  typically require more than what is shown above. For example, we may want to communicate joint angles with corresponding joint names, distance sensor info, robot position in 3D and so on. ROS supports derived message types. These are messages  composed of basic message types. Examples of such messages are shown below:


```
Point
------
float x
float y
float z

```

```
Pose
------
- Position
    float x
    float y
    float z
- orientation
    float x
    float y
    float z
    float w

```

```
PointStamped
------
time stamp
string reference_frame
- Position
    float x
    float y
    float z
```

---

**Remark**


The ```std_msgs``` package defines the primitive types.  Arrays of these types, both fixed and variable length, are returned from the lower level communication code. These primitive types are used to build all of the messages used in ROS. These messages are contained in the ```std_msgs``` package and the ```common_msgs``` package.


---




### Creating Custom Messages

Let's now see hot to create custom message types for our own applications. 
Item <a href="ros_introduction.md">ROS Introduction</a> mentions that ROS messages are defined in ```*.msg``` files. 
Message files typically exist in a ROS package with the following naming convention:


```
ros_package_name/msg
```


---

**REMARK**

These message files are compiled into language-specific implementations that can be used in your code. 
This means that even if you are using an interpreted language, such as Python, you need to run ```catkin_make``` 
if you’re going to define your own message types. Otherwise, the language-specific implementation will not be generated, and Python will not be able to find your new message type. Furthermore, if you don’t rerun ```catkin_make``` after you change the message definition, Python will still be using the older version of the message type.

---

Message-definition files are typically quite simple and short. Each line specifies a type
and a field name. Types can be:

- Built-in ROS primitive types
- Message types from other packages, arrays of types (either primitive or from other packages, and either
fixed or variable length)
- The special ```Header``` type.

We will see below that this is easy to develop. One more thing to remeber is that we need to indicate the new message type to the build system. 
We do so via the ```CMakeLists.txt``` of the package that the message is being used. 

Create a new package and name it ```ros_messages```:

```
cd catkin_ws/src
catkin_create_pkg ros_messages std_msgs rospy roscpp
cd ..
catkin_make
```

Our package depends again on the ```std_msgs, rorpy``` and ```roscpp``` packages. We need a ```msg``` directory in our package to store our messages. Let's create it.
In the ```msg``` directory you just created, create a file ```example.msg``` with the following contents

```
string message
int32 a
int32 b

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

- Tell ```catkin``` that we’re going to use messages at runtime, by adding ```message_runtime``` to the ```catkin_package()```:

```
add_message_files(
FILES
example.msg
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
project(ros_messages)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	message_generation
)

add_message_files(
  FILES
   example.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node_1 src/node_1/node_1.cpp)
add_executable(${PROJECT_NAME}_node_2 src/node_2/node_2.cpp)

target_link_libraries(${PROJECT_NAME}_node_1   ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node_2   ${catkin_LIBRARIES})

```

---

**Remark**

Check the code in the ```src/ros_messages/node_1``` and ```src/ros_messages/node_2``` 

---


### Build

Now that we have everything set, we nee to build the our package

```
cd catkin_ws
catkin_make
```

---
**REMARK**

Generated message definitions contain an MD5 checksum. This is used by ROS to make sure that it’s using the correct version of a message. If you modify your message-definition files and run
```catkin_make``` over them, you might also have to run ```catkin_make``` over any code that uses these messages, to make sure that the checksums match up. This is generally more of a problem with C++ than with Python, since the checksums are compiled into the executables. However, it can be an issue with Python with compiled byte code (```.pyc``` files).

---

The above process will create a ```example.h``` header in the ```catkin_ws/devel/include/ros_messages``` directory. This is where the C++ definition, or else the type, of our message
lies. We should include this header to our nodes should we want to use the message.  Checkout the 	```workspace/src/ros_messages/src/node_1/node_1.cpp``` and
```workspace/src/ros_messages/src/node_2/node_2.cpp``` how to use adapt the nodes from Item <a href="ros_hello.md">Hello ROS</a> in order to use our message. 


### Run

Once this is done, let's run the application.

Start the master node

```
roscore
``` 

Open a new terminal and type

```
rosrun ros_messages ros_messages_node_1 
```

Similarly for node 2.

---
**REMARK**

Make sure that the termoinals that execute the two nodes that you executed

```
source /path/to/catkin_ws/devel/setup.sh
```

---



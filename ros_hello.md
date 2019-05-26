# Item: Hello ROS

#### References

1. **Creating a ROS Package:** http://wiki.ros.org/ROS/Tutorials/CreatingPackage


## Hello ROS

Item <a href="ros_create_workspace.md">ROS ```workspaces```, ```packages``` and ```catkin```</a> gives a high level overview of what a ```workspace``` in ROS
is and how to create one using ```catkin```. In a ```workspace``` we typically have the packages related to our robot. Let's see how to 
create our first package. For more information you should have a look at [1].


---

**Remark**

Depending on how you installed ROS you may have to do some sourcing. For example. if you did a source installation then
you have to 

```
source ~/ros_catkin_ws/install_isolated/setup.bash
```

Note also that the path up to ```install_isolated``` may be different for you.

---

A valid ROS package must contain a ```package.xml``` that is ```catkin``` compliant.  This can be done using the ```catkin_create_pkg``` script to create a new package.
Assuming that you already have a valid ```catkin``` workspace, do


```
cd ~/catkin_ws/src
catking_create_pkg hello_ros std_msgs rospy roscpp

```

This will create a ```hello_ros``` directory which contains a ```package.xml``` and a ```CMakeLists.txt```.
These have been partially filled out with the information you gave ```catkin_create_pkg```.
The output after executing the above may look something along the following

```
Created file hello_ros/CMakeLists.txt
Created file hello_ros/package.xml
Created folder hello_ros/include/hello_ros
Created folder hello_ros/src
Successfully created files in /home/david/ros_catkin_ws/my_catkin_ws/src/hello_ros. Please adjust the values in package.xml.

```



```catkin_create_pkg``` requires that you give it a ```package_name``` and optionally a list of dependencies on which that package depends: 

```
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```


---

**Remark**

If you don't have a valid ```catkin``` workspace you can do

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws 
catkin_make
```

For more information see Item <a href="ros_create_workspace.md">ROS ```workspaces```, ```packages``` and ```catkin```</a> and references therein.

--- 



Now that we created our package, we will have to build it.

```
cd ~/catkin_ws
catkin_make

```

We now have to add our workspace to the ROS environment. We do so by

```
source ~/catkin_ws/devel/setup.bash
```

---

**Remark**

We need to add our workspace every time it changes so that the ROS environment is aware of the changes.

---


### Nodes

Let's now create our first two nodes. One node will send the message "Hello ROS" and the other one will print that to the screen

### ROS Tools

We can use various ROS tools to review our package.


```
/// Find out first order dependencies
rospack depends1 hello_ros
roscpp
rospy
std_msgs

```



# Item: Vision Sensors in ROS

#### References

1. **OpenCV:** https://opencv.org/
2. **Video4Linux:** https://en.wikipedia.org/wiki/Video4Linux
3. **```usb_cam```:** http://wiki.ros.org/usb_cam
4. **vision_opencv:** http://wiki.ros.org/vision_opencv
5. **image_transport:** http://wiki.ros.org/image_transport
6. **Writing a Simple Image Publisher (C++):** http://wiki.ros.org/image_transport/Tutorials/PublishingImages

---

**Remark**

The C++ code has been adapted from the following video

[![](http://img.youtube.com/vi/LECg-Gv5xjo/0.jpg)](https://www.youtube.com/watch?v=LECg-Gv5xjo "OpenCV and Python: Find Lanes for a Self-Driving Car (Basics)")

---

## Vision Sensors in ROS

Vision sensors, such as cameras and LiDARs, are perhaps the most effective sensors a robot can have.
In this item, we will go over how to program such types of sensors in ROS. Concretely, in this item we will
discuss how to interact with video cameras and how to use the OpenCV interface available in ROS.

### Interacting With A Web Camera

We can interact with web cameras in ROS using the ```usb_cam``` package. This package  is the ROS driver for Video4Linux (V4L) USB cameras. 
V4L is a collection of device drivers in Linux for real-time video capture from web cameras. 
The ```usb_cam``` ROS package works using V4L devices and publishes the video stream from devices as ROS image messages. 
We can subscribe to it and perform our own processing using it. Let's see how to install the package


---

**Remark** 

In case that you don't have ```Video4Linux``` installed, you can do so by

```
sudo apt-get install v4l-utils
```

---

---

**Remark: Side Note**

As a side note we can find out if ```v4l-utils``` is installed on our box by using

```
dpkg -s v4l-utils
``` 

---

Once you have the ```v4l-utils``` installed, we will install  the ```usb_cam``` package. We do so as follows

```
cd ~/ros_workspace/src
git clone https://github.com/bosch-ros-pkg/usb_cam.git
cd ~/ros_workspace/
catkin_make

``` 

Assuming that everything completed successfully, let's have a last sanity check. 
Connect a webcam you may have available  to your PC  and check whether it is detected. We can do so by
opening a terminal and execute the ```dmesg``` command. This will print the kernel logs. 
The logs can be quite lengthy but in any case try to see if you can identify your web camera. 


---

**Remark: Cheese Webcam Viewer**

Assuming that  your webcam has support in Ubuntu, we can open the video device using a tool called
```Cheese```. This is simply a webcam viewer.


---

#### Test ```usb_cam```

Let's now test whether the ```usb_cam``` package works as expected.


An image in ROS is displayed using the ```image_view``` package. This package subscribes to the topic called ```/usb_cam/image_raw```.

---

**Remark:ROS Topics**


We can get a list of topics in ROS by using

```
rostopic list
```
---


### Using OpenCV

OpenCV is perhaps the most popular, at the time of writing, image processings and computer vision library. 
It has many  popular algorithms for various tasks related to image processing
and computer vision.  OpenCV has bindings for Python and C++. You can checkout the library at https://opencv.org/.


ROS integrates OpenCV via a package called ```vision_opencv```, see [4] for more details.
The ```vision_opencv``` metapackage has two packages. Namely:

- ```cv_bridge:``` This package is responsible for converting the OpenCV image data
type (```cv::Mat```) into ROS ```sensor_msgs/Image.msg``` messages.
- ```image_geometry:``` This package helps us interpret images geometrically. 


Let's write a node that performs lane tracking using OpenCV.


### Lane Tracker With OpenCV

- Create a package called ```ocv_lane_tracker```

```
cd catkin_ws/src
catkin_create_pkg ocv_lane_tracker sensor_msgs cv_bridge roscpp std_msgs image_transport
cd ..
catkin_make
```


Now we need to build our package again. Let's do this.

#### Update ```package.xml``` 

Update the ```package.xml``` file so that it reflects our package


#### Update ```CMakeLists.txt```

We need to update the default generated ```CMakeLists.txt```. This is not much different to what we did previously. The ```CMakeLists.txt```
will look something like the following

```
cmake_minimum_required(VERSION 2.8.3)
project(ocv_lane_tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocv_lane_tracker
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/node.cpp)

# You may have to alter this/ or not needed at all
LINK_DIRECTORIES(OpenCV_LIBRARIES_DIR /usr/local/lib) 
SET(OpenCV_LIBRARIES opencv_highgui )
target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
```

---

**Remark**

OpenCV has some libraries that depending on the application we need to link against.
If you have done a system-wide install of OpenCV these libraries should be in ```/usr/local/lib```.
--- 

---

**Remark**

Check the code in the ```src/ocv_lane_tracker/src/node.cpp```. 

---


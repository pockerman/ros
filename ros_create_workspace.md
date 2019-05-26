# Item: ROS ```workspaces```, ```packages``` and ```catkin```

#### References

1. **ROS ```catkin```:** http://wiki.ros.org/catkin
2. **ROS workspaces:** http://wiki.ros.org/catkin/workspaces
3. **ROS package:** http://wiki.ros.org/Packages
4. **ROS ```CMakeLists.txt```:** http://wiki.ros.org/catkin/CMakeLists.txt


## ```catkin```, ```workspaces``` and ```packages```

This item continues from where Item <a href="ros_introduction.md">Introduction To ROS</a> left over and introduces more
ROS concepts. 

Some of the things outlined in this item are also presented in the video below.

[![](http://img.youtube.com/vi/0BxVPCInS3M/0.jpg)](https://www.youtube.com/watch?v=jYqDnuxTwK8&t=2s "Programming for Robotics (ROS) Course 2")

### ```catkin```

```catkin``` is the ROS build system. This  is a set of tools that ROS uses to generate executable programs, libraries, scripts and interfaces that client code can use.

```catkin``` consists of a set of ```CMake``` macros and customized ```Python``` scripts. 
This is done in order  to provide extra functionality on top of the normal ```CMake``` workflow. 

---

**Remark: ```CMake```**
```CMake``` is a commonly used open  source build system. See https://cmake.org/ for more information. 

---

For the simple ```catkin``` user, all you there is really to know is that there are two files; ```CMakeLists.txt``` and ```package.xml```.
We need to add some specific information into these two files in order to have things generated and work properly. 
We then call the various ```catkin``` tools to generate the directories and files we are going to use as we develop our software for our robot.


### workspaces

Our ROS code should live in  ```workspace``` for this code. A ```workspace``` is simply a directory that organizes ROS project files. Let's see how we
can generate such a ```workspace```. 

We can do so by using ```catkin```. Typically, a ```catkin```  generated ```workspace``` contains the following directories:

- ```src```
- ```devel```
- ```build```
- ```install```

Let's see how we can create a ```catkin``` workspace and initialize it. 
Fisrt make sure that you have added the system-wide ROS setup script to our ```~/.bashrc``` file. If you haven't you can do so by

```
source /opt/ros/<YOUR_ROS_DIST>/setup.bash
```

We can now create the ```catkin``` workspace and initialize it.


```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

This creates a ```workspace``` directory called ```catkin_ws``` (although you can call it however you like), with a ```src``` directory inside it for your code. The ```catkin_init_workspace``` command creates a ```CMakeLists.txt``` file for you in the ```src``` directory, where you invoked it. Next, we’re going to create some other ```workspace``` files. We do so by executing

```

cd ~/catkin_ws 
catkin_make
```

---

**Remark**


If you are familiar with ```CMake``` the above command breaks down to the following

```
mkdir build
cd build
cmake ..
make
make install  # (optionally)
```

---

Executing ```catkin_make``` will generate a lot of output. When it’s finished,
we will have two new directories: 

- ```build```
- ```devel```. 

```build``` is where ```catkin``` is going to store the results of some of its work, like libraries and executable programs if you use C++. 


```devel``` contains a number of files and directories, the most interesting of which are the
```setup``` files. Running these configures your system to use this ```workspace```, and the code that’s (going to be) contained inside it. 
Assuming you’re using the default command-line shell ( bash ) and are still in the top-level directory of your workspace, you can do this with:


```
source devel/setup.bash
```

---
**REMARK**

We can have multiple ROS ```workspace```s. However, we can only work in one of them at any one time. 

---

### ```packages``` 
 
ROS software is organized into packages (this includes code that we write or code we want to use and others have written), each of which contains some combination of code, data, and documentation. Concretely, ROS packages contain


- source code
- launch files
- configuration files
- message definitions
- data
- documentation


Packages sit inside workspaces, in the ```src``` directory. Each package directory must
include a ```CMakeLists.txt``` file and a ```package.xml``` file that describes the contents of the
package and how catkin should interact with it. We can create a new package by using:

```
cd ~/catkin_ws/src
catkin_create_pkg my_awesome_code rospy
```

This changes the directory to ```src``` (where packages live) and invokes ```catkin_create_pkg``` to make the new package called ```my_awesome_code``` , which depends on the (already existing) ```rospy``` package. If your new package depends on other existing packages, you can also list them on the command line. We’ll talk about package
dependencies later.

The ```catkin_create_pkg``` command makes a directory with the same name as the new
package, ```my_awesome_code``` with a ```CMakeLists.txt``` file, a ```package.xml``` file, and a ```src``` and an ```include``` directories in it. 
The ```package.xml``` file contains a bunch of metadata about your new package.

A package may depend on another package(s). If this is the case, then package declares the package(s) it depends on as dependencies. 


---

**Remark**

 According to the documentation, for a package to be considered a ```catkin``` package it must meet a few requirements: 

- The package must contain a ```catkin``` compliant ```package.xml``` file 
- The package must contain a ```CMakeLists.txt``` which uses ```catkin```
- Each package must have its own folder

The ```package.xml``` file provides meta information about the package. 
Furthermore,  a ```catkin``` metapackage must have the relevant boilerplate ```CMakeLists.txt``` file. 
The last requirement means that no nested packages nor multiple packages sharing the same directory. 

---

#### ```CMakeLists.txt``` and ```package.xml```

We have already mentioned that each package directory must
include a ```CMakeLists.txt``` file and a ```package.xml``` file that describes the contents of the
package and how catkin should interact with it. 

In particular, ```package.xml``` defines the properties of the package:

- package name
- version number
- author(s)
- license
- dependencies

```CMakeLists.txt``` is the input to the CMake build system. 

#### Installing a package

Sometimes the functionality our application requires has already been implemented. 
In this case all we have to do is to install the required package(s) and use it in our code. We can do so by using

```
cd <path_to_folder_with_ROS_package(s)>
rosdep install <package_name>

```

Alternatively, we can also use

```
cd <path_to_ROS_workspace/src>
rosdep install --from-paths . ignore-src -y 

```

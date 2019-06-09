# Item: Install ROS

#### References

1. **Ubuntu install of ROS Melodic:** http://wiki.ros.org/Installation/Ubuntu
2. **Repositories/Ubuntu:** https://help.ubuntu.com/community/Repositories/Ubuntu
3. **Installing from source:** http://wiki.ros.org/melodic/Installation/Source

## Install ROS

In this item we will show you how to install ROS on Ubuntu 18.04 LTS. There are two options in order to install
ROSS

- Using Ubuntu software manager ```apt``` (this is the recommended approach)
- Install from source.


### Install Using ```apt```

The instaructions are a brief summary from [1] so you can also check therein if you have any problems.

1. Configure your Ubuntu repositories 

Configure your Ubuntu repositories to allow _restricted_,  _universe_ and _multiverse_. You can follow [2] for this step. 

2. Setup ```sources.list```

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

3. Setup keys

```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

4. Installation

```
sudo apt update
```
- Desktop-Full Installation

```
sudo apt install ros-melodic-desktop-full
```

Before you can use ROS, you will need to initialize ```rosdep```. ```rosdep``` enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.

```
sudo rosdep init
rosdep update
```

### Environment setup

It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched: 

```

echo "source /opt/ros/<YOUR_ROS_DIST>/setup.bash" >> ~/.bashrc
source ~/.bashrc

```


To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. 
For example, ```rosinstall``` is a frequently used command-line tool that enables you to easily download 
many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run: 

```
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```


Finally, we can print information about the ROS environment by

```
printenv | grep ROS
```

A possible output may be

```
ROS_ROOT=/opt/ros/<YOUR_ROS_DIST>/share/ros
ROS_PACKAGE_PATH=/opt/ros/<YOUR_ROS_DIST>/share
ROS_MASTER_URI=http://localhost:11311
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=<YOUR_ROS_DIST>
ROS_ETC_DIR=/opt/ros/<YOUR_ROS_DIST>/etc/ros
```

### Install ROS From Source

We can also install ROS from source as described in [3] and summarized below.

1. Installing dependencies

```
sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
```

2. Initializing ```rosdep```

```
sudo rosdep init
rosdep update
```

3. Create a catkin Workspace

```
mkdir ~/ros_catkin_ws
$ cd ~/ros_catkin_ws
```

4.  Fetch the core packages

```
rosinstall_generator desktop_full --rosdistro <YOUR_ROS_DIST> --deps --tar > <YOUR_ROS_DIST>-desktop-full.rosinstall
```

5. Install core packages with ```wstool```

```
wstool init -j8 src <YOUR_ROS_DIST>-desktop-full.rosinstall
```

6. Resolve dependencies

```
rosdep install --from-paths src --ignore-src --rosdistro <YOUR_ROS_DIST> -y
```

7. Build the ```catkin``` workspace

```
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
```




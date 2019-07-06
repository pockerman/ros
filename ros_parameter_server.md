# Item: Parameter Server

#### References

1. **Parameter Server:** http://wiki.ros.org/Parameter%20Server
2. **```rosparam:```** http://wiki.ros.org/rosparam

## Parameter Server

In this item we will go over the ROS parameter server. You can follow the videos below to learn more

[![](http://img.youtube.com/vi/zSYZD7lnT3E/0.jpg)](https://www.youtube.com/watch?v=zSYZD7lnT3E "[ROS in 5 mins] 012 - What is ROS Parameter Server?")
[![](http://img.youtube.com/vi/m9FjgacE2OY/0.jpg)](https://www.youtube.com/watch?v=m9FjgacE2OY "[ROS in 5 mins] 053 - How to load params on the Parameter Server")


According to [1]: A parameter server is a shared, multi-variate dictionary that is accessible via network APIs. 
Nodes use this server to store and retrieve parameters at runtime. 
As it is not designed for high-performance, it is best used for static, non-binary data such as configuration parameters. 
It is meant to be globally viewable so that tools  can easily inspect the configuration state of the system and modify if necessary. 


So  a parameter server is simply an entity that allows us to store and retrive 
parameters relevant to our robot.

ROS uses a certain convention in order to name parameters 
Concretely, ROS parameters have a hierarchy that matches the namespaces used for 
topics and nodes. The reason behind this hierarchy is to protect parameter names 
from colliding. Moreover, the hierarchical scheme also allows parameters to 
be accessed individually or as a tree [1]. 

Let's see some examples taken from [1]

```
/camera/left/name: leftcamera
/camera/left/exposure: 1
/camera/right/name: rightcamera
/camera/right/exposure: 1.1

```

The parameter ```/camera/left/name``` has the value ```leftcamera```. 
You can also get the value for ```/camera/left```, which is the dictionary. 
You can also get the value for ```/camera```, which has a 
dictionary of dictionaries representation of the parameter tree:  

```
left: { name: leftcamera, exposure: 1 }
right: { name: rightcamera, exposure: 1.1 }

```

### Parameter Types

The Parameter Server uses XMLRPC data types for parameter values, which include [1]:

- 32-bit integers
- booleans
- strings
- doubles
- iso8601 dates
- lists
- base64-encoded binary data 

The Parameter Server represents ROS namespaces as dictionaries. 
For example, imagine you set the following three parameters: 

```
/gains/P = 10.0
/gains/I = 1.0
/gains/D = 0.1

```

You can either read them back separately, i.e. 
retrieving ```/gains/P``` would return ```10.0```, or you can retrieve 
```/gains```, which would return a dictionary: 

```
{ 'P': 10.0, 'I': 1.0, 'D' : 0.1 }
```

### ```rosparam```

The ```rosparam``` command-line tool enables you to query and set parameters on the Parameter Server using YAML syntax [1].

```rosparam``` contains the ```rosparam``` command-line tool for getting and setting ROS Parameters on the Parameter Server using YAML-encoded files. 
 This library is intended for internal use only. ```rosparam``` can be invoked within a ```roslaunch``` file [2].

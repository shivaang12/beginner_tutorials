# ROS beginner Tutorial
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/shivang/beginner_tutorials/blob/master/LICENSE)

This repository is a part of the ENPM808X - introduction to ROS assignment.

## Dependencies 
The dependencies of this repository are:

```
* Ubuntu 16.04
* ROS Kinetic Kame
```

After ensuring that you have Ubuntu 16.04, follow below steps for ROS installation. More information can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
```

Now intializing of ROS

```
$ rosdep init
$ rosdep update 
```

To setup ROS environment:

```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Creating ROS Workspace:

```
$ cd <path where workspace needs to be created>
$ mkdir -p <name of workspace>
$ cd <workspace>
<workspace>$ catkin_make
```

After any updates to workspace, `source <path to workspace>/devel/setup.bash` commands needs to be executed. To avoid this every time, it can be added to *.bashrc* file.

## Building

The code can be built by cloning the repository:
```
<home>$ cd <workspace>/src
<workspace>/src$ git clone https://github.com/shivaang12/beginner_tutorials.git
<workspace>/src$ cd ..
<workspace>$ catkin_make 
```
## Running the code

This week's tutorials deal with *services* and *launch* files. 

### Launch files

Launch files are very useful when multiple nodes needs to be started from within the package. It can also be used to set the arguments of the nodes from within the launch files. In this package the file *launch_change_text.launch* does that. There will be difference in the published message in the nodes which are being launched from the launch file and nodes which are started individually using *rosrun* commands.

### Service files

*Services* are also a useful tool of ROS. Publisher/subscriber communication model is very useful for continuous communication, but server/client communication has its own uses. Server/client model is advantageous when one has to execute certain process as need arises by requesting the *service*. This package has a service called *text_change* which changes the message being published. It takes an input the string which needs to be published and responses with a *bool* value.

### Running using *rosrun*
Please ensure that *rosmaster* is running before executing following steps. *rosmaster* can be started by following command.
```
<home>$ roscore
```

To start the *publisher* node follow the steps given below:
```
<home>$ rosrun beginner_tutorials pulisher_node
```

*subscriber* node can be started by following commands:
```
<home>$ rosrun beginner_tutorials subscriber_node
```

Starting the nodes using the steps above, prints out `Welcome to ENPM808X, Fall 2018!` on the console.

### Running using *roslaunch* 
To start both the nodes with a single command, *launch* file can be created and used to launch both the nodes. To *launch* both nodes execute following command. No need to have *roscore* running prior to running this:
```
<home>$ roslaunch beginner_tutorials launch_change_text.launch 
```

*roscore* starts when the file is launched if it is not running. The command above will launch the node with default string in the launch file. To pass the custom string to launch file use the following command:
```
<home>$ roslaunch beginner_tutorials launch_change_text.launch str:="ENPM808X-is-an-online-course."
```

Note: parameter to launch file, the string should be a single cannot contain whitespaces.

### Calling the service 
Service can also be called from the *terminal* when both the nodes are running (refereing to this package). This is necessary because *publisher* node is the server for service while *subscriber* node is client of the service. To call the service both server and client should be run properly.
```
<home>$ rosservice call /text_change <string to be published>
```

For this package, it can be done as follows:
```
<home>$ rosservice call /text_change "This is America."
```


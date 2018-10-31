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

The code contains only has one node which publishes and subscribes to *chatter* topic. To run the code, ensure *rosmaster* is running. If it is not running, it can be started in a new terminal by executing `$ roscore`.
Execute following command start ROS core service:
```
$ roscore
```
Then type the following in new terminal to run the node.
```
$ rosrun beginner_tutorials beginner_tutorials_node
```

The output of the code is looks like follows:
```
[ INFO] [1539919785.735576754]: The incoming stream is:
ENPM808X, Fall 2018!
[ INFO] [1539919785.835575162]: The incoming stream is:
ENPM808X, Fall 2018!
[ INFO] [1539919785.935575044]: The incoming stream is:
ENPM808X, Fall 2018!
```


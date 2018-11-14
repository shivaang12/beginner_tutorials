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
<workspace>/src$ cd beginner_tutorials
<workspace>/src$ git checkout Week11_HW
<workspace>/src$ cd ..
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
<home>$ rosservice call /change_text <string to be published>
```

For this package, it can be done as follows:
```
<home>$ rosservice call /change_text "This is America."
```

### TF transform
TF transforms are the way to define coordinate transforms between various frames in ROS. The publisher node publishes a non-zero transformation between `world` and `publisher` frames. The easiest to check if the transforms are being broadcasted between the two frames is using `tf_echo` command. To get the more detailed information about transforms between all the frames and how they are connected, `view_frames` or `rqt_tf_tree` command is used.

Ensure that `publisher_node` is already running. It can be started using the instructions given in the earlier sections. Now to see the transform being broadcasted, execute the following command:
```
<home>$ rosrun tf tf_echo /publiher /world
``` 
Here, the first frame is the parent frame while the second frame is the child frame. The output after executing the above command looks like below:
```
At time 1542150435.708
- Translation: [1.998, 1.003, -1.000]
- Rotation: in Quaternion [0.000, 0.000, 1.000, -0.001]
            in RPY (radian) [0.000, 0.000, -3.140]
            in RPY (degree) [0.000, 0.000, -179.909]
At time 1542150436.708
- Translation: [1.998, 1.003, -1.000]
- Rotation: in Quaternion [0.000, 0.000, 1.000, -0.001]
            in RPY (radian) [0.000, 0.000, -3.140]
            in RPY (degree) [0.000, 0.000, -179.909]
At time 1542150437.708
- Translation: [1.998, 1.003, -1.000]
- Rotation: in Quaternion [0.000, 0.000, 1.000, -0.001]
            in RPY (radian) [0.000, 0.000, -3.140]
            in RPY (degree) [0.000, 0.000, -179.909]
```
To generate a PDF containing the connections between various frames assuming publisher node is already running,
```
<home>$ rosrun rqt_tf_tree rqt_tf_tree
```

Or use following,
```
<home>$ rosrun tf view_frames
```
## Testing using rostest/gtest 
Integration testing is very important to ensure that the newly created or modified modules does not break the running version of the code. This module has two unit tests which ensure that `change_text` service is running and that the transform being boradcasted is as expected. 

There are two ways to run the tests, one way is by using `catkin_make` and another way by using `rostest`. Before running the tests, ensure that `catkin_make` is invoked to build the units that are not being tested. The tests can be run using either of the following commands:
#### Using catkin_make 
```
<home>$ cd <path to directory>/catkin_ws
<workspace>$ catkin_make run_tests_beginner_tutorials
```

#### Using rostest
```
<home>$ rostest beginner_tutorials publisher_test.launch
```
The output might look like this:
```
[Testcase: testpublisherTest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-publisherTest/serviceTest][passed]
[beginner_tutorials.rosunit-publisherTest/transformTest][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/shivang/.ros/log/rostest-shivang-Latitude-E5270-10147.log
```

## Recording/Playing rosbag
`rosbag` allows to record all the data that is being published across all the nodes. Bag files are very useful when one wants to debug what is happening on the robot. However, rosbag does not record calls to services as well as responses of the services. 
It can be recorded from the commandline using `rosbag record -a`. And the recorded bag file can be played using `rosbag play <path to bag file>`.

Before proceeding to play the bag file, ensure that subscriber node is running as per the instructions given the earlier sections. Now to play the `publisher.bag` file located in `results/`:
```
<home>$ rosbag play catkin_ws/src/beginner_tutorials/results/publisher.bag
```

This command will start playing the recorded bag file and the subscriber node which was running previously will be able to listen to the data that is being published. One can check the transform being broadcasted using the instructions given before. 

`launch_change_text.launch` launch file of this package accepts a flag `record` that can be used to record the bag file which will record all the data that is being published on the `chatter` topic. It will save the file in `results` directory. This can be done using following commands:
```
<home>$ roslaunch beginner_tutorials launch_change_text.launch record:=true
```
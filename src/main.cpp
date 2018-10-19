/**
 * @file    main.cpp
 * @author  Shivang Patel
 * @copyright MIT License (c) 2018 Shivang Patel
 *
 * @brief DESCRIPTION
 * This is main file which runs the nodes. This file initializes ROS and 
 * creates the objects for the Publisher and Subscriber classes.
 *
 */
#include <string>
#include "publisher.hpp"
#include "subscriber.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "main_node");
  // Initiate the string to be published
  std::string str = "ENPM808X, Fall 2018!";
  // Create the Subscriber object
  Subscriber sub;
  // Create the Publisher object
  Publisher pub;
  // Publish the message
  pub.publish(str);

  return 0;
}
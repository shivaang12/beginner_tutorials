/**
 * @file    pub_main.cpp
 * @author  Shivang Patel
 * @copyright MIT License (c) 2018 Shivang Patel
 *
 * @brief DESCRIPTION
 * Main file for the publisher node. Initializes node, initializes the string
 * being published, starts the server for the service change_text, and publish
 * the string.
 *
 */

#include <string>
#include "publisher.hpp"

int main(int argc, char **argv) {
  // Create publisher node
  ros::init(argc, argv, "publisher");
  // Inform that the publisher node is started
  ROS_INFO_STREAM("Initializing node: publisher...");
  // Initiate the string to be published
  std::string str = "Welcome to ENPM808X, Fall 2018!";
  if (argc > 1) {
    // Initialize the message with string coming from launch file
    str = argv[1];
  }
  // Node handle for publisher node
  ros::NodeHandle nh;
  // Create the Publisher object
  Publisher pub(str);
  // Create server for service change_text
  ros::ServiceServer server =
      nh.advertiseService("change_text", &Publisher::changeText, &pub);
  // Inform that service is started
  ROS_INFO_STREAM("Service started: change_text");

  // Publish the message
  pub.publish();

  return 0;
}

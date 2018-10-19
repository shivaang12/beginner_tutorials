/**
 * @file    publisher.cpp
 * @author  Shivang Patel
 * @copyright MIT License (c) 2018 Shivang Patel
 *
 * @brief DESCRIPTION
 * Class implementation for Publisher
 *
 */
#include "publisher.hpp"
#include <string>

Publisher::Publisher() {
  publisher_ = nh_.advertise<std_msgs::String>("chatter", 1000);
}

void Publisher::publish(const std::string& str) {
  // Set the rate at which messages are to be published
  ros::Rate loop_rate(10);
  // Create a message variable of std_msgs;:String type
  std_msgs::String msg;
  // Continuously publish messages
  while (ros::ok()) {
    // Assign the string to message
    msg.data = str;
    // Publish the message
    publisher_.publish(msg);
    // Spin in the callbacks
    ros::spinOnce();
    // Sleep for some time
    loop_rate.sleep();
  }
}
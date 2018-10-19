/**
 * @file    subscriber.cpp
 * @author  Shivang Patel
 * @copyright MIT License (c) 2018 Shivang Patel
 *
 * @brief DESCRIPTION
 * Class implementation for Subscriber
 *
 */

#include "subscriber.hpp"
/**
 * @brief      Constructs the object of the class.
 */
Subscriber::Subscriber() {
  // Subscribe to the "chatter" topic
  subscriber_ =
      nh_.subscribe("chatter", 1000, &Subscriber::subscriberCallback, this);
}
/**
 * @brief      Subsriber callback
 *
 */
void Subscriber::subscriberCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("The incoming stream is: " << std::endl << msg->data);
}
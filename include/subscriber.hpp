/**
 * @file    subscriber.hpp
 * @author  Shivang Patel
 * @copyright MIT License (c) 2018 Shivang Patel
 *
 * @brief DESCRIPTION
 * Class for subscribing to chatter topic.
 *
 */

#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief      Class that subscribs to chatter topic
 */
class Subscriber {
 public:
  /**
   * @brief      Constructor of the class
   */
  Subscriber();
  /**
   * @brief      Callback function for the subscriber.
   *
   * @param[in]  msg:   String to publish.
   *
   * @return     void:  Return nothing.
   */
  void subscriberCallback(const std_msgs::String::ConstPtr& msg);

 private:
  ros::NodeHandle nh_;          ///< Node handle for the subscriber node
  ros::Subscriber subscriber_;  ///< Subscriber object
};
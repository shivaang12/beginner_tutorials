/**
 * @file    publisher.hpp
 * @author  Shivang Patel
 * @copyright MIT License (c) 2018 Shivang Patel
 *
 * @brief DESCRIPTION
 * Class for publishing to chatter topic.
 *
 */

#pragma once

#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief      Class for publishing to chatter topic
 */
class Publisher {
 public:
  /**
   * @brief      Constructor of the class
   */
  Publisher();

  void publish(const std::string& str);

 private:
  ros::NodeHandle nh_;        ///< Node handle for the publisher node
  ros::Publisher publisher_;  ///< Publisher object
  std::string text_;          ///< Message to be published
};

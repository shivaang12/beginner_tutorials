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
#include "beginner_tutorials/text_change.h"

/**
 * @brief      Class for publishing to chatter topic
 */
class Publisher {
 public:
  /**
   * @brief      Constructor of the class
   */
  Publisher(const std::string& str);
  /**
   * @brief      This method publishes the passed string to chatter topic
   *
   * @param[in]  str:   String to be published
   *
   * @return     void: Return nothing
   */
  auto publish() -> void;
  /**
   * @brief      A service callback function. Changes the text on chatter topic
   *
   * @param      request:  Request of Service
   * @param      resp:     Response of Service
   *
   * @return     bool:     Returns true if service callback was successful
   */
  auto changeText(beginner_tutorials::text_change::Request& request,
                  beginner_tutorials::text_change::Response& resp) -> bool;

 private:
  ros::NodeHandle nh_;        ///< Node handle for the publisher node
  ros::Publisher publisher_;  ///< Publisher object
  std::string text_;          ///< Message to be published
};

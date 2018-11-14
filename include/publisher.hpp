// MIT License

// Copyright (c) 2018 Shivang Patel

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
#include "beginner_tutorials/text_change.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"

/**
 * @brief      Class for publishing to chatter topic
 */
class Publisher {
 public:
    /**
     * @brief      Constructor of the class
     */
    explicit Publisher(const std::string &str);
    /**
     * @brief      This method publishes the passed string to chatter topic
     *
     * @param[in]  str:   String to be published
     *
     * @return     void: Return nothing
     */
    auto publish() -> void;
    /**
     * @brief      A service callback function. Changes the text on chatter
     * topic
     *
     * @param      request:  Request of Service
     * @param      resp:     Response of Service
     *
     * @return     bool:     Returns true if service callback was successful
     */
    auto changeText(beginner_tutorials::text_change::Request &request,
                    beginner_tutorials::text_change::Response &resp)
        -> bool;

 private:
    ros::NodeHandle nh_;        ///< Node handle for the publisher node
    ros::Publisher publisher_;  ///< Publisher object
    std::string text_;          ///< Message to be published
};

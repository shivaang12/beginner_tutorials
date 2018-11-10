// MIT License

// Copyright (c) 2018 Shivang Patel

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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

Publisher::Publisher(const std::string& str) : text_(str) {
  publisher_ = nh_.advertise<std_msgs::String>("chatter", 1000);
}

auto Publisher::publish() -> void {
  // Set the rate publishing
  ros::Rate loop_rate(10);
  // Create a message variable of std_msgs::String type
  std_msgs::String msg;
  // Continuously publish messages
  while (ros::ok()) {
    // Check the number of subscribers and if it is 0, generate warning
    if (publisher_.getNumSubscribers() == 0) {
      ROS_WARN_STREAM("No subscribers on the topic.");
    } else {
      ROS_INFO_STREAM(
          "Number of subscribers are: " << publisher_.getNumSubscribers());
    }
    // Assign the string to message
    msg.data = text_;
    // Publish the message
    publisher_.publish(msg);
    // Sleep for some time
    loop_rate.sleep();
    // Spin in the callbacks
    ros::spinOnce();
  }
}

auto Publisher::changeText(
    beginner_tutorials::text_change::Request& request,
    beginner_tutorials::text_change::Response& resp) -> bool {
    // Assign the string received from the service
    text_ = request.text;
    // Response of the service
    resp.response = true;
    // Warn that the message being published is changed
    ROS_WARN_STREAM("Changing the message being published...");

    return true;
}

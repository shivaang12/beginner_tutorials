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
 * @file    subscriber_main.cpp
 * @author  Shivang Patel
 * @copyright MIT License (c) 2018 Shivang Patel
 *
 * @brief DESCRIPTION
 * Main code for the subscriber node. Initialize the node, log the information,
 * and set callbacks.
 *
 */
#include <string>
#include "beginner_tutorials/text_change.h"
#include "subscriber.hpp"

int main(int argc, char **argv) {
  // Create publisher node
  ros::init(argc, argv, "subscriber");
  // Inform that node is initialized
  ROS_INFO_STREAM("Initializing node: subscriber...");
  // Node handle for the subscriber node
  ros::NodeHandle nh;
  // Create service client for service change_text
  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::text_change>("change_text");
  // Log fatal error if client is not created
  if (!client.exists()) {
    ROS_FATAL_STREAM("Service is not available!");
  } else {
    ROS_DEBUG_STREAM("Service is running.");
  }
  // Log information until server is not available
  if (ros::service::waitForService("change_text", 10000)) {
    ROS_DEBUG_STREAM("Server is available.");
  } else {
    ROS_ERROR_STREAM("Waiting for server to run.");
  }

  // Create the Publisher object
  Subscriber sub;
  // Spin in the callback
  ros::spin();

  return 0;
}

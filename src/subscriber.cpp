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
 * @brief      Subscriber callback
 *
 */
void Subscriber::subscriberCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("The incoming stream is: " << std::endl << msg->data);
}

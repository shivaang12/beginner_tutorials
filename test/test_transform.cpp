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
 * @file    test_transform.cpp
 * @author  Shivang Patel
 * @copyright MIT License (c) 2018 Shivang Patel
 *
 * @brief DESCRIPTION
 * Unit tests for the transforms being published.
 *
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include "publisher.hpp"

/**
 * @brief      Unit test for testing the published transform between world frame
 * and publisher frame
 *
 */

TEST(TESTSuite, transformTest) {
    // Create TF listener object
    tf::TransformListener listener;
    // Create transform object to store the published transform
    tf::StampedTransform transform;
    // Wait till the transform is published
    if (listener.waitForTransform("world", "talk", ros::Time(0),
                                  ros::Duration(100))) {
        // Get the value of the published transform
        listener.lookupTransform("world", "talk", ros::Time(0), transform);

        // Check the values of the published transform
        EXPECT_EQ(2.0, transform.getOrigin().x());
        EXPECT_EQ(1.0, transform.getOrigin().y());
        EXPECT_EQ(1.0, transform.getOrigin().z());
    }
}

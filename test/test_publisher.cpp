// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file main.cpp
 * @author Sanchit Tanwar (sanchittanwar75@gmail.com)
 * @brief Rostest to test the talker node.
 * @version 0.1
 * @date 2022-12-13
 * 
 */
#include <gtest/gtest.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TestTalker : public testing::Test {
 protected:
     rclcpp::Node::SharedPtr node_;
};

TEST_F(TestTalker, test_num_of_publishers) {
  node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub = node_->create_publisher<std_msgs::msg::String>
                    ("topic", 4.0);

  auto num_pub = node_->count_publishers("topic");
  EXPECT_EQ(1, static_cast<int>(num_pub));
}

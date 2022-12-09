
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/custom_message.hpp"

using namespace std::chrono_literals;



/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher()
  : Node("publisher_node"), count_(0) {
    // set the logger level to DEBUG from Info
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                    "Getting frequency parameter value");

    auto pubfreq_info = rcl_interfaces::msg::ParameterDescriptor();
    pubfreq_info.description =
                "Frequency value for the publisher";
    this->declare_parameter("pubfreq", 2.0, pubfreq_info);
    auto pubfreq = this->get_parameter("pubfreq")
                  .get_parameter_value().get<std::float_t>();


    publisher_ = this->create_publisher<std_msgs::msg::String>
                 ("topic", 10);
    // calculate publisher freq
    auto time = std::chrono::milliseconds(
            static_cast<int>(1000/pubfreq));

    timer_ = this->create_wall_timer(
      time, std::bind(&MinimalPublisher::timer_callback, this));

    auto serviceCallbackPtr = std::bind(&MinimalPublisher::modify_message,
                    this, std::placeholders::_1, std::placeholders::_2);

    service_ = create_service<beginner_tutorials::srv::CustomMessage>(
                    "publish_custom_message", serviceCallbackPtr);
    }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hi this is Sanchit"+
    std::to_string(count_++)+ " time.";
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    publisher_->publish(message);
  }

  void modify_message(
    const std::shared_ptr<beginner_tutorials::srv::CustomMessage::Request>
                                                  request,
    std::shared_ptr<beginner_tutorials::srv::CustomMessage::Response>
                       response) {
    response->divye = request->sanchit+
             " No request message!!!";
    RCLCPP_INFO_STREAM(this->get_logger(),
                  "Request message: "<< request->sanchit);
    RCLCPP_INFO_STREAM(this->get_logger(),
                  "Response message: "<< response->divye);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::CustomMessage>::SharedPtr service_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

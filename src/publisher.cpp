
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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

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

/**
 * @brief To publish a static frame to /tf_static topic
 * 
 */
class StaticFramePublisher : public rclcpp::Node {
 public:
    explicit StaticFramePublisher(char * transforms[])
    : Node("publisher_node") {
    tf_static_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->create_transform(transforms);
    }

 private:
    void create_transform(char * transforms[]) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "talker";
    t.child_frame_id = transforms[1];

    t.transform.translation.x = atof(transforms[2]);
    t.transform.translation.y = atof(transforms[3]);
    t.transform.translation.z = atof(transforms[4]);
    tf2::Quaternion q;
    q.setRPY(
      atof(transforms[5]),
      atof(transforms[6]),
      atof(transforms[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};


int main(int argc, char * argv[]) {
if (argc ==1) {
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<MinimalPublisher>());
rclcpp::shutdown();
return 0;
} else {
auto logger = rclcpp::get_logger("logger");

if (argc != 8) {
  RCLCPP_INFO(
    logger, "Invalid number of parameters");
  return 1;
}

if (strcmp(argv[1], "talker") == 0) {
  RCLCPP_INFO(logger, "static turtle name cannot be 'talker'");
  return 2;
}


rclcpp::init(argc, argv);
RCLCPP_INFO(logger,
                "Tf2 Static Brodcaster started");
rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
rclcpp::shutdown();
return 3;
}
}

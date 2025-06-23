// Copyright 2025 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"

// Include the message type we will subscribe to
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

// Use std::placeholders to bind the callback function
using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode()
  : Node("subscriber_node")
  {
    // Create a subscription to the "/counter" topic
    // This subscription will receive messages of type std_msgs::msg::Int32
    // with a queue size of 10. The callback function will be called
    // whenever a new message is received on this topic.
    // The callback function is bound to the member function 'callback' of this class.
    // The '_1' placeholder is used to pass the received message to the callback.
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/counter", 10, std::bind(&SubscriberNode::callback, this, _1));
  }

private:
  // The callback function that will be called when a new message is received
  void callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->data);
  }

  // Declare the subscription as a shared pointer
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SubscriberNode>();

  // Spin the node to keep it alive and process callbacks
  // This will keep the node running and allow it to publish messages.
  rclcpp::spin(node);

  // Shutdown the ROS 2 system when done
  // This will clean up resources and stop the node gracefully.
  rclcpp::shutdown();

  return 0;
}
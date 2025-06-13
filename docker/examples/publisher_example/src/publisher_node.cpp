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

// Include the message type we will publish
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode()
  : Node("publisher_node"), counter_(0)
  {
    // Create a publisher that publishes messages of type std_msgs::msg::Int32
    // on the topic "/counter" with a queue size of 10.
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("/counter", 10);

    // Create a timer that calls the step function every 500 milliseconds.
    // The step function will publish a message with an incrementing counter.
    timer_ = this->create_wall_timer(500ms, std::bind(&PublisherNode::step, this));
  }

  // The step function is called by the timer and publishes a message.
  void step()
  {
    // Create a new message of type std_msgs::msg::Int32
    std_msgs::msg::Int32 message;

    // Set the data field of the message to the current counter value
    message.data = counter_++;

    // Log the message being published
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);

    // Publish the message to the topic
    publisher_->publish(message);
  }

private:
  // Declare the publisher and timer as shared pointers
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  // Declare the timer that will call the step function periodically
  rclcpp::TimerBase::SharedPtr timer_;

  // Counter to keep track of the number of messages published
  int counter_ = 0;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PublisherNode>();

  // Spin the node to keep it alive and process callbacks
  // This will keep the node running and allow it to publish messages.
  rclcpp::spin(node);

  // Shutdown the ROS 2 system when done
  // This will clean up resources and stop the node gracefully.
  rclcpp::shutdown();

  return 0;
}
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
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BumpGoNode : public rclcpp::Node
{
public:
  BumpGoNode()
  : Node("bumpgo_node"), state_(FORWARD)
  {
    // Declare the speed publisher, with the topic and queue

    // Declare the subscriber of the laser sensor, indicating the topic and which callback function you associate it with.

    timer_ = create_wall_timer(
      500ms, std::bind(&BumpGoNode::step, this));
  }

  // Function to handle the robot's movement logic
  void step()
  {
    geometry_msgs::msg::Twist velocity_msg;
  
    switch (state_) {
      case FORWARD:
        // Fill velocity_msg with the speeds you consider appropriate
        // IF obstacle_detected < OBSTACLE_DISTANCE:
        //   Change state to BACK;
        //   Take a timestamp of when you transition from forward to back 
        break;
      case BACK:
        // Fill velocity_msg with the speeds you consider appropriate
        // Check if the time elapsed since the last timestamp is greater than BACKING_TIME:
        //   Change state to TURN;
        //   Take a timestamp of when you transition from back to turn
        break;
      case TURN:
        // Fill velocity_msg with the speeds you consider appropriate
        // Check if the time elapsed since the last timestamp is greater than TURNING_TIME:
        //   Change state to FORWARD;
        break;
    }

    // Publish the speed
    // ...
  }

private:

  // Function to update the timestamp
  void time()
  {
    time_stamp_ = this->now();
  }

  // Function to check if a certain duration has elapsed since the last timestamp
  bool time_elapsed(const rclcpp::Duration & duration) const
  {
    return (this->now() - time_stamp_) > duration;
  }

  // Callback for laser scan messages
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    (void)msg;  // This line is set to remove the unused variable warning. Delete it when you implement the callback.

    // Process the laser scan data to determine the minimum distance
    // min_laser_distance_ = (...);

    // ...
  }

  // Constants for timing
  const rclcpp::Duration TURNING_TIME {2s};
  const rclcpp::Duration BACKING_TIME {2s};
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  // Constants for robot movement
  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_ANGULAR = 0.3f;
  static constexpr float OBSTACLE_DISTANCE = 0.5f;

  // State machine states
  int state_;
  static const int FORWARD = 0;
  static const int BACK = 1;
  static const int TURN = 2;

  // Member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  float min_laser_distance_ = std::numeric_limits<float>::max();
  rclcpp::Time time_stamp_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BumpGoNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
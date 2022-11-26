// Copyright 2021 ROBOTIS CO., LTD.
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

#ifndef VELOCITY_INTERFACE_NODE_
#define VELOCITY_INTERFACE_NODE_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_velocity.hpp"


void setupDynamixelPositionMode(uint8_t dxl_id);


class VelocityInterfaceNode : public rclcpp::Node
{
public:
  using SetVelocity = dynamixel_sdk_custom_interfaces::msg::SetVelocity;

  VelocityInterfaceNode();
  virtual ~VelocityInterfaceNode();

private:
  rclcpp::Subscription<SetVelocity>::SharedPtr set_velocity_subscriber_;
  rclcpp::TimerBase::SharedPtr no_velocity_command_timer_;


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr present_position_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_homing_position_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_lighter_service_;

  void timer_callback();
  void no_velocity_command_timer_callback();

  void trigger_lighter_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>, 
    std::shared_ptr<std_srvs::srv::Trigger::Response>);
  bool isLighterOn = false;

  void reset_coming_position_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>, 
    std::shared_ptr<std_srvs::srv::Trigger::Response>);
    
    uint8_t present_id;
  int present_position;
};

#endif  // VELOCITY_INTERFACE_NODE_

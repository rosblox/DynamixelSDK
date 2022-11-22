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

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples velocity_interface_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /velocity_interface_node/set_velocity dynamixel_sdk_custom_interfaces/SetVelocity "{id: 1, velocity: 1000}"
// $ ros2 topic echo /velocity_interface_node/status"
//
// Author: Max Polzin
*******************************************************************************/

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_velocity.hpp"

#include "rclcpp/rclcpp.hpp"

#include "velocity_interface_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_VELOCITY_LIMIT 44
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Protocol parameter
#define PARAM_VELOCITY_CONTROL_MODE 1
#define PARAM_TORQUE_ENABLE_FALSE 0
#define PARAM_TORQUE_ENABLE_TRUE 1
#define PARAM_VELOCITY_LIMIT_MAX 255


// Default setting
#define BAUDRATE 2000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/U2D2"


using namespace std::chrono_literals;

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_velocity = 0;
int dxl_comm_result = COMM_TX_FAIL;

VelocityInterfaceNode::VelocityInterfaceNode()
: Node("velocity_interface_node")
{
  RCLCPP_INFO(this->get_logger(), "Run velocity interface node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  

  set_velocity_subscriber_ =
    this->create_subscription<SetVelocity>(
    "set_velocity",
    QOS_RKL10V,
    [this](const SetVelocity::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;


      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      uint32_t goal_velocity = (unsigned int)msg->velocity;  // Convert int32 -> uint32

      // Write Goal Velocity (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id,
        ADDR_GOAL_VELOCITY,
        goal_velocity,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      }

      no_velocity_command_timer_->reset();
    }
    );

    no_velocity_command_timer_ = this->create_wall_timer(2500ms, std::bind(&VelocityInterfaceNode::no_velocity_command_timer_callback, this));

    present_position_publisher_ = this->create_publisher<std_msgs::msg::Int32>("present_position", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&VelocityInterfaceNode::timer_callback, this));

}


VelocityInterfaceNode::~VelocityInterfaceNode(){

}

void VelocityInterfaceNode::no_velocity_command_timer_callback(){

  uint8_t dxl_error = 0;

  // Write Goal Velocity (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result =
  packetHandler->write4ByteTxRx(
    portHandler,
    present_id,
    ADDR_GOAL_VELOCITY,
    0,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
  }

}


void VelocityInterfaceNode::timer_callback()
{

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.

  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler,
    present_id,
    ADDR_PRESENT_POSITION,
    reinterpret_cast<uint32_t *>(&present_position),
    &dxl_error
  );

  auto message = std_msgs::msg::Int32();
  message.data = present_position;
  present_position_publisher_->publish(message);
}


void setupDynamixel(uint8_t dxl_id)
{
  // Use Velocity Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    PARAM_VELOCITY_CONTROL_MODE,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("velocity_interface_node"), "Failed to set Velocity Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("velocity_interface_node"), "Succeeded to set Velocity Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    PARAM_TORQUE_ENABLE_TRUE,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("velocity_interface_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("velocity_interface_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("velocity_interface_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("velocity_interface_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("velocity_interface_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("velocity_interface_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto velocity_interface_node = std::make_shared<VelocityInterfaceNode>();
  rclcpp::spin(velocity_interface_node);
  rclcpp::shutdown();

  // // Disable Torque of DYNAMIXEL
  // packetHandler->write1ByteTxRx(
  //   portHandler,
  //   BROADCAST_ID,
  //   ADDR_TORQUE_ENABLE,
  //   PARAM_TORQUE_ENABLE_FALSE,
  //   &dxl_error
  // );

  return 0;
}

// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <netdb.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "robco_hw/robco_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robcomm/robcomm.hpp"




namespace robco_hw
{
hardware_interface::CallbackReturn RobcoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  joint_pos_commands_.resize(info_.joints.size(), 0.0);
  joint_vel_commands_.resize(info_.joints.size(), 0.0);
  joint_pos_states_.resize(info_.joints.size(), 0.0);
  joint_vel_states_.resize(info_.joints.size(), 0.0);
  joint_acc_states_.resize(info_.joints.size(), 0.0);
  // joint_pos_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // joint_vel_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // joint_pos_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // joint_vel_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // joint_acc_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

 // 设置默认参数值
  robot_ip_ = "192.168.3.1";
  local_rx_port_ = 25001;
  remote_tx_port_ = 25000;
  robot_init_timeout_ = 3;

  RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"), 
              "Attempting to connect to robot at %s (rx %d tx %d)...",
              robot_ip_.c_str(), local_rx_port_, remote_tx_port_);

  robot_.connect(robot_ip_, local_rx_port_, remote_tx_port_);
  
  RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"), 
              "Waiting for robot to initialize (timeout is %d seconds)...", robot_init_timeout_);
  for (int i = 0; i < 100 * robot_init_timeout_ && !robot_.is_initialized(); i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  if (!robot_.is_initialized())
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobcoHardwareInterface"), 
                 "Timeout waiting for robot to initialize");
    return CallbackReturn::ERROR;
  }

  robot_.receive();

  RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"), 
              "%d joints found", robot_.get_joint_count());
  RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"), 
              "Robco robot interface initialized.");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobcoHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
  
 
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobcoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
      // TODO(anyone): insert correct interfaces
      state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_pos_states_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_vel_states_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_acc_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobcoHardwareInterface::export_command_interfaces()
{
  // TODO(anyone): insert correct interfaces
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size() * 2);
  position_command_interface_names_.reserve(info_.joints.size());
  velocity_command_interface_names_.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_pos_commands_[i]));
      position_command_interface_names_.push_back(command_interfaces.back().get_name());
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_vel_commands_[i]));
      velocity_command_interface_names_.push_back(command_interfaces.back().get_name());
     }

  return command_interfaces;
}


hardware_interface::CallbackReturn RobcoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands
  // 设置机器人为操作状态
  robot_.set_state(robcomm::ROBOT_STATE_CMD_OPERATIONAL);
  
  if (!robot_.is_initialized())
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobcoHardwareInterface"), "Robot not initialized!");
    return CallbackReturn::ERROR;
  }

  // 设置关节初始状态
  // auto joint_angles = robot_.getJointAngles();
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_pos_states_[i] = 0.0;
    joint_pos_commands_[i] = 0.0;
    joint_vel_commands_[i] = 0.0; // 初始化速度命令为0
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobcoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands
  robot_.set_state(robcomm::ROBOT_STATE_CMD_DISABLED);

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobcoHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states
  // 读取机器人状态
  // robot_.receive();
  // auto joint_angles = robot_.getJointAngles();
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_pos_states_[i] = joint_pos_commands_[i];
    joint_vel_states_[i] = joint_vel_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobcoHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): write robot's commands'

  robot_.jog_joints(joint_vel_commands_);
  return hardware_interface::return_type::OK;

  return hardware_interface::return_type::OK;
}

}  // namespace robco_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robco_hw::RobcoHardwareInterface, hardware_interface::SystemInterface)

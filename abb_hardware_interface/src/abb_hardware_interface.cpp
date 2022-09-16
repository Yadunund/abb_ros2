// Copyright 2020 ROS2-Control Development Team
// Modifications Copyright 2022 PickNik Inc
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

#include <abb_hardware_interface/abb_hardware_interface.hpp>
#include <abb_hardware_interface/utilities.hpp>

#include <iostream>

using namespace std::chrono_literals;

namespace abb_hardware_interface
{
static constexpr size_t NUM_CONNECTION_TRIES = 100;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ABBSystemPositionOnlyHardware");

CallbackReturn ABBSystemPositionOnlyHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const auto rws_port = stoi(info_.hardware_parameters["rws_port"]);
  const auto rws_ip = info_.hardware_parameters["rws_ip"];

  if (rws_ip == "None")
  {
    RCLCPP_FATAL(LOGGER, "RWS IP not specified");
    return CallbackReturn::ERROR;
  }

  // Get robot controller description from RWS
  // abb::robot::RWSManager rws_manager(rws_ip, rws_port, "Default User", "robotics");
  // const auto robot_controller_description_ =
  //     abb::robot::utilities::establishRWSConnection(rws_manager, "IRB1200", true);

  // Description of ABB robot controller.
  abb::robot::RobotControllerDescription description{};

  auto SetJoint = [](
    abb::robot::StandardizedJoint* p_joint,
    const std::string name,
    const double lower_bound,
    const double upper_bound)
  {
    p_joint->set_standardized_name(name);
    p_joint->set_rotating_move(true);
    p_joint->set_lower_joint_bound(M_PI * (lower_bound) / 180.0);
    p_joint->set_upper_joint_bound(M_PI * (upper_bound) / 180.0);
  };

  // Add header.
  auto header{description.mutable_header()};
  header->mutable_robot_ware_version()->set_major_number(7);
  header->mutable_robot_ware_version()->set_minor_number(3);
  header->mutable_robot_ware_version()->set_patch_number(2);

  // Add system indicators.
  auto system_indicators{description.mutable_system_indicators()};
  system_indicators->mutable_options()->set_egm(true);

  // Add single mechanical units group.
  auto mug{description.add_mechanical_units_groups()};
  mug->set_name("");

  // Add single robot to mechanical units group.
  auto robot{mug->mutable_robot()};
  robot->set_type(abb::robot::MechanicalUnit_Type_TCP_ROBOT);
  robot->set_axes_total(6);
  robot->set_mode(abb::robot::MechanicalUnit_Mode_ACTIVATED);

  // std::string robot_controller_id{""};
  // Add joints to robot.
  // auto id{robot_controller_id.empty() ? "" : robot_controller_id + "_"};
  auto joint_name{"joint_"};
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(1), -180.0, 180.0);
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(2), -95.0, 155.0);
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(3), -210.0, 65.0);
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(4), -230.0, 230.0);
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(5), -130.0, 130.0);
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(6), -400.0, 400.0);

  robot_controller_description_ = std::move(description);
  RCLCPP_INFO_STREAM(LOGGER, "Robot controller description:\n"
                                 << abb::robot::summaryText(robot_controller_description_));

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                   joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  // Configure EGM
  RCLCPP_INFO(LOGGER, "Configuring EGM interface...");

  // Initialize motion data from robot controller description
  try
  {
    abb::robot::initializeMotionData(motion_data_, robot_controller_description_);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize motion data from robot controller description");
    return CallbackReturn::ERROR;
  }

  // Create channel configuration for each mechanical unit group
  std::vector<abb::robot::EGMManager::ChannelConfiguration> channel_configurations;
  for (const auto& group : robot_controller_description_.mechanical_units_groups())
  {
    try
    {
      const auto egm_port = stoi(info_.hardware_parameters[group.name() + "egm_port"]);
      const auto channel_configuration =
          abb::robot::EGMManager::ChannelConfiguration{ static_cast<uint16_t>(egm_port), group };
      channel_configurations.emplace_back(channel_configuration);
      RCLCPP_INFO_STREAM(LOGGER,
                         "Configuring EGM for mechanical unit group " << group.name().c_str() << " on port " << egm_port);
    }
    catch (std::invalid_argument& e)
    {
      RCLCPP_FATAL_STREAM(LOGGER, "EGM port for mechanical unit group \"" << group.name()
                                                                          << "\" not specified in hardware parameters");
      return CallbackReturn::ERROR;
    }
  }
  try
  {
    egm_manager_ = std::make_unique<abb::robot::EGMManager>(channel_configurations);
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize EGM connection");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ABBSystemPositionOnlyHardware::export_state_interfaces()
{
  std::cout << "[export state]" << std::endl;

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        std::cout << "[export state]" <<  joint.name << std::endl;
        // TODO(seng): Consider changing joint names in robot description to match what comes
        // from the ABB robot description to avoid needing to strip the prefix here
        // const auto pos = joint.name.find("joint");
        // const auto joint_name = joint.name.substr(pos);
        const auto& joint_name = joint.name;
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &joint.state.position));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &joint.state.velocity));
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ABBSystemPositionOnlyHardware::export_command_interfaces()
{
    std::cout << "[export command] has " << motion_data_.groups.size() << " groups" << std::endl;

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& group : motion_data_.groups)
  {
    std::cout <<"Group name: " << group.name << "has " << group.units.size() << " units" << std::endl;
    for (auto& unit : group.units)
    {
      std::cout << "unit name " << unit.name << " has " << unit.joints.size() << " joint" << std::endl;
      for (auto& joint : unit.joints)
      {
        // TODO(seng): Consider changing joint names in robot description to match what comes
        // from the ABB robot description to avoid needing to strip the prefix here
        // const auto pos = joint.name.find("joint");
        // const auto joint_name = joint.name.substr(pos);
        const auto& joint_name = joint.name;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &joint.command.position));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &joint.command.velocity));
      }
    }
  }

  return command_interfaces;
}

CallbackReturn ABBSystemPositionOnlyHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  size_t counter = 0;
  RCLCPP_INFO(LOGGER, "Connecting to robot...");
  while (rclcpp::ok() && ++counter < NUM_CONNECTION_TRIES)
  {
    // Wait for a message on any of the configured EGM channels.
    if (egm_manager_->waitForMessage(500))
    {
      RCLCPP_INFO(LOGGER, "Connected to robot");
      break;
    }

    RCLCPP_INFO(LOGGER, "Not connected to robot...");
    if (counter == NUM_CONNECTION_TRIES)
    {
      RCLCPP_ERROR(LOGGER, "Failed to connect to robot");
      return CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(500ms);
  }

  egm_manager_->read(motion_data_);

  RCLCPP_INFO(LOGGER, "ros2_control hardware interface was successfully started!");

  return CallbackReturn::SUCCESS;
}

return_type ABBSystemPositionOnlyHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  egm_manager_->read(motion_data_);
  return return_type::OK;
}

return_type ABBSystemPositionOnlyHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  egm_manager_->write(motion_data_);
  return return_type::OK;
}

}  // namespace abb_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(abb_hardware_interface::ABBSystemPositionOnlyHardware, hardware_interface::SystemInterface)

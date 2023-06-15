// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gwpspider_interfaces:msg/GripperCommand.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__MSG__DETAIL__GRIPPER_COMMAND__BUILDER_HPP_
#define GWPSPIDER_INTERFACES__MSG__DETAIL__GRIPPER_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gwpspider_interfaces/msg/detail/gripper_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gwpspider_interfaces
{

namespace msg
{

namespace builder
{

class Init_GripperCommand_command
{
public:
  explicit Init_GripperCommand_command(::gwpspider_interfaces::msg::GripperCommand & msg)
  : msg_(msg)
  {}
  ::gwpspider_interfaces::msg::GripperCommand command(::gwpspider_interfaces::msg::GripperCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gwpspider_interfaces::msg::GripperCommand msg_;
};

class Init_GripperCommand_leg
{
public:
  Init_GripperCommand_leg()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperCommand_command leg(::gwpspider_interfaces::msg::GripperCommand::_leg_type arg)
  {
    msg_.leg = std::move(arg);
    return Init_GripperCommand_command(msg_);
  }

private:
  ::gwpspider_interfaces::msg::GripperCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::gwpspider_interfaces::msg::GripperCommand>()
{
  return gwpspider_interfaces::msg::builder::Init_GripperCommand_leg();
}

}  // namespace gwpspider_interfaces

#endif  // GWPSPIDER_INTERFACES__MSG__DETAIL__GRIPPER_COMMAND__BUILDER_HPP_

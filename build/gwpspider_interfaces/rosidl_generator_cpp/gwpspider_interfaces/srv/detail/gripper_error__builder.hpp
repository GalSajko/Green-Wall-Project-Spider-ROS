// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gwpspider_interfaces:srv/GripperError.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__GRIPPER_ERROR__BUILDER_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__GRIPPER_ERROR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gwpspider_interfaces/srv/detail/gripper_error__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gwpspider_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripperError_Request_leg_index
{
public:
  explicit Init_GripperError_Request_leg_index(::gwpspider_interfaces::srv::GripperError_Request & msg)
  : msg_(msg)
  {}
  ::gwpspider_interfaces::srv::GripperError_Request leg_index(::gwpspider_interfaces::srv::GripperError_Request::_leg_index_type arg)
  {
    msg_.leg_index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gwpspider_interfaces::srv::GripperError_Request msg_;
};

class Init_GripperError_Request_monitor
{
public:
  Init_GripperError_Request_monitor()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperError_Request_leg_index monitor(::gwpspider_interfaces::srv::GripperError_Request::_monitor_type arg)
  {
    msg_.monitor = std::move(arg);
    return Init_GripperError_Request_leg_index(msg_);
  }

private:
  ::gwpspider_interfaces::srv::GripperError_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::gwpspider_interfaces::srv::GripperError_Request>()
{
  return gwpspider_interfaces::srv::builder::Init_GripperError_Request_monitor();
}

}  // namespace gwpspider_interfaces


namespace gwpspider_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripperError_Response_success
{
public:
  Init_GripperError_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gwpspider_interfaces::srv::GripperError_Response success(::gwpspider_interfaces::srv::GripperError_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gwpspider_interfaces::srv::GripperError_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::gwpspider_interfaces::srv::GripperError_Response>()
{
  return gwpspider_interfaces::srv::builder::Init_GripperError_Response_success();
}

}  // namespace gwpspider_interfaces

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__GRIPPER_ERROR__BUILDER_HPP_

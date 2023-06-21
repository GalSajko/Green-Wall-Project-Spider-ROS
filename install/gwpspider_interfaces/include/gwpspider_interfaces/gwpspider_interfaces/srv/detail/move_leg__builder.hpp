// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gwpspider_interfaces:srv/MoveLeg.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_LEG__BUILDER_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_LEG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gwpspider_interfaces/srv/detail/move_leg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gwpspider_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveLeg_Request_duration
{
public:
  explicit Init_MoveLeg_Request_duration(::gwpspider_interfaces::srv::MoveLeg_Request & msg)
  : msg_(msg)
  {}
  ::gwpspider_interfaces::srv::MoveLeg_Request duration(::gwpspider_interfaces::srv::MoveLeg_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveLeg_Request msg_;
};

class Init_MoveLeg_Request_trajectory_type
{
public:
  explicit Init_MoveLeg_Request_trajectory_type(::gwpspider_interfaces::srv::MoveLeg_Request & msg)
  : msg_(msg)
  {}
  Init_MoveLeg_Request_duration trajectory_type(::gwpspider_interfaces::srv::MoveLeg_Request::_trajectory_type_type arg)
  {
    msg_.trajectory_type = std::move(arg);
    return Init_MoveLeg_Request_duration(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveLeg_Request msg_;
};

class Init_MoveLeg_Request_goal_position
{
public:
  explicit Init_MoveLeg_Request_goal_position(::gwpspider_interfaces::srv::MoveLeg_Request & msg)
  : msg_(msg)
  {}
  Init_MoveLeg_Request_trajectory_type goal_position(::gwpspider_interfaces::srv::MoveLeg_Request::_goal_position_type arg)
  {
    msg_.goal_position = std::move(arg);
    return Init_MoveLeg_Request_trajectory_type(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveLeg_Request msg_;
};

class Init_MoveLeg_Request_leg
{
public:
  Init_MoveLeg_Request_leg()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveLeg_Request_goal_position leg(::gwpspider_interfaces::srv::MoveLeg_Request::_leg_type arg)
  {
    msg_.leg = std::move(arg);
    return Init_MoveLeg_Request_goal_position(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveLeg_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::gwpspider_interfaces::srv::MoveLeg_Request>()
{
  return gwpspider_interfaces::srv::builder::Init_MoveLeg_Request_leg();
}

}  // namespace gwpspider_interfaces


namespace gwpspider_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveLeg_Response_success
{
public:
  Init_MoveLeg_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gwpspider_interfaces::srv::MoveLeg_Response success(::gwpspider_interfaces::srv::MoveLeg_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveLeg_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::gwpspider_interfaces::srv::MoveLeg_Response>()
{
  return gwpspider_interfaces::srv::builder::Init_MoveLeg_Response_success();
}

}  // namespace gwpspider_interfaces

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_LEG__BUILDER_HPP_

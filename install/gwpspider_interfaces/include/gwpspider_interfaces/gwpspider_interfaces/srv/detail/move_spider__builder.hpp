// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gwpspider_interfaces:srv/MoveSpider.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_SPIDER__BUILDER_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_SPIDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gwpspider_interfaces/srv/detail/move_spider__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gwpspider_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveSpider_Request_spider_pose
{
public:
  explicit Init_MoveSpider_Request_spider_pose(::gwpspider_interfaces::srv::MoveSpider_Request & msg)
  : msg_(msg)
  {}
  ::gwpspider_interfaces::srv::MoveSpider_Request spider_pose(::gwpspider_interfaces::srv::MoveSpider_Request::_spider_pose_type arg)
  {
    msg_.spider_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveSpider_Request msg_;
};

class Init_MoveSpider_Request_duration
{
public:
  explicit Init_MoveSpider_Request_duration(::gwpspider_interfaces::srv::MoveSpider_Request & msg)
  : msg_(msg)
  {}
  Init_MoveSpider_Request_spider_pose duration(::gwpspider_interfaces::srv::MoveSpider_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_MoveSpider_Request_spider_pose(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveSpider_Request msg_;
};

class Init_MoveSpider_Request_origin
{
public:
  explicit Init_MoveSpider_Request_origin(::gwpspider_interfaces::srv::MoveSpider_Request & msg)
  : msg_(msg)
  {}
  Init_MoveSpider_Request_duration origin(::gwpspider_interfaces::srv::MoveSpider_Request::_origin_type arg)
  {
    msg_.origin = std::move(arg);
    return Init_MoveSpider_Request_duration(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveSpider_Request msg_;
};

class Init_MoveSpider_Request_trajectory_type
{
public:
  explicit Init_MoveSpider_Request_trajectory_type(::gwpspider_interfaces::srv::MoveSpider_Request & msg)
  : msg_(msg)
  {}
  Init_MoveSpider_Request_origin trajectory_type(::gwpspider_interfaces::srv::MoveSpider_Request::_trajectory_type_type arg)
  {
    msg_.trajectory_type = std::move(arg);
    return Init_MoveSpider_Request_origin(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveSpider_Request msg_;
};

class Init_MoveSpider_Request_goal_positions
{
public:
  explicit Init_MoveSpider_Request_goal_positions(::gwpspider_interfaces::srv::MoveSpider_Request & msg)
  : msg_(msg)
  {}
  Init_MoveSpider_Request_trajectory_type goal_positions(::gwpspider_interfaces::srv::MoveSpider_Request::_goal_positions_type arg)
  {
    msg_.goal_positions = std::move(arg);
    return Init_MoveSpider_Request_trajectory_type(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveSpider_Request msg_;
};

class Init_MoveSpider_Request_legs
{
public:
  Init_MoveSpider_Request_legs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveSpider_Request_goal_positions legs(::gwpspider_interfaces::srv::MoveSpider_Request::_legs_type arg)
  {
    msg_.legs = std::move(arg);
    return Init_MoveSpider_Request_goal_positions(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveSpider_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::gwpspider_interfaces::srv::MoveSpider_Request>()
{
  return gwpspider_interfaces::srv::builder::Init_MoveSpider_Request_legs();
}

}  // namespace gwpspider_interfaces


namespace gwpspider_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveSpider_Response_success
{
public:
  Init_MoveSpider_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gwpspider_interfaces::srv::MoveSpider_Response success(::gwpspider_interfaces::srv::MoveSpider_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gwpspider_interfaces::srv::MoveSpider_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::gwpspider_interfaces::srv::MoveSpider_Response>()
{
  return gwpspider_interfaces::srv::builder::Init_MoveSpider_Response_success();
}

}  // namespace gwpspider_interfaces

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_SPIDER__BUILDER_HPP_

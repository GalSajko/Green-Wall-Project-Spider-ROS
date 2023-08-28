// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gwpspider_interfaces:srv/GetOffsetsToChargingPosition.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__GET_OFFSETS_TO_CHARGING_POSITION__BUILDER_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__GET_OFFSETS_TO_CHARGING_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gwpspider_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetOffsetsToChargingPosition_Request_current_pins
{
public:
  Init_GetOffsetsToChargingPosition_Request_current_pins()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request current_pins(::gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request::_current_pins_type arg)
  {
    msg_.current_pins = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request>()
{
  return gwpspider_interfaces::srv::builder::Init_GetOffsetsToChargingPosition_Request_current_pins();
}

}  // namespace gwpspider_interfaces


namespace gwpspider_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetOffsetsToChargingPosition_Response_offsets
{
public:
  Init_GetOffsetsToChargingPosition_Response_offsets()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response offsets(::gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response::_offsets_type arg)
  {
    msg_.offsets = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response>()
{
  return gwpspider_interfaces::srv::builder::Init_GetOffsetsToChargingPosition_Response_offsets();
}

}  // namespace gwpspider_interfaces

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__GET_OFFSETS_TO_CHARGING_POSITION__BUILDER_HPP_

// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gwpspider_interfaces:srv/GetOffsetsToChargingPosition.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__GET_OFFSETS_TO_CHARGING_POSITION__TRAITS_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__GET_OFFSETS_TO_CHARGING_POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'current_pins'
#include "std_msgs/msg/detail/int16_multi_array__traits.hpp"

namespace gwpspider_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetOffsetsToChargingPosition_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_pins
  {
    out << "current_pins: ";
    to_flow_style_yaml(msg.current_pins, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetOffsetsToChargingPosition_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_pins
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_pins:\n";
    to_block_style_yaml(msg.current_pins, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetOffsetsToChargingPosition_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace gwpspider_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gwpspider_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  gwpspider_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gwpspider_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request & msg)
{
  return gwpspider_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request>()
{
  return "gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request";
}

template<>
inline const char * name<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request>()
{
  return "gwpspider_interfaces/srv/GetOffsetsToChargingPosition_Request";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Int16MultiArray>::value> {};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Int16MultiArray>::value> {};

template<>
struct is_message<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'offsets'
#include "std_msgs/msg/detail/float32_multi_array__traits.hpp"

namespace gwpspider_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetOffsetsToChargingPosition_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: offsets
  {
    out << "offsets: ";
    to_flow_style_yaml(msg.offsets, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetOffsetsToChargingPosition_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: offsets
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offsets:\n";
    to_block_style_yaml(msg.offsets, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetOffsetsToChargingPosition_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace gwpspider_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use gwpspider_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  gwpspider_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gwpspider_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response & msg)
{
  return gwpspider_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response>()
{
  return "gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response";
}

template<>
inline const char * name<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response>()
{
  return "gwpspider_interfaces/srv/GetOffsetsToChargingPosition_Response";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Float32MultiArray>::value> {};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Float32MultiArray>::value> {};

template<>
struct is_message<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<gwpspider_interfaces::srv::GetOffsetsToChargingPosition>()
{
  return "gwpspider_interfaces::srv::GetOffsetsToChargingPosition";
}

template<>
inline const char * name<gwpspider_interfaces::srv::GetOffsetsToChargingPosition>()
{
  return "gwpspider_interfaces/srv/GetOffsetsToChargingPosition";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::GetOffsetsToChargingPosition>
  : std::integral_constant<
    bool,
    has_fixed_size<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request>::value &&
    has_fixed_size<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response>::value
  >
{
};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::GetOffsetsToChargingPosition>
  : std::integral_constant<
    bool,
    has_bounded_size<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request>::value &&
    has_bounded_size<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response>::value
  >
{
};

template<>
struct is_service<gwpspider_interfaces::srv::GetOffsetsToChargingPosition>
  : std::true_type
{
};

template<>
struct is_service_request<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request>
  : std::true_type
{
};

template<>
struct is_service_response<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__GET_OFFSETS_TO_CHARGING_POSITION__TRAITS_HPP_

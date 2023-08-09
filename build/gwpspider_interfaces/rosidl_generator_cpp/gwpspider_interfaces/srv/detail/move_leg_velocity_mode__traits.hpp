// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gwpspider_interfaces:srv/MoveLegVelocityMode.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_LEG_VELOCITY_MODE__TRAITS_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_LEG_VELOCITY_MODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "gwpspider_interfaces/srv/detail/move_leg_velocity_mode__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'legs_ids'
#include "std_msgs/msg/detail/int8_multi_array__traits.hpp"
// Member 'velocity_mode_direction'
#include "std_msgs/msg/detail/float32_multi_array__traits.hpp"

namespace gwpspider_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveLegVelocityMode_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: legs_ids
  {
    out << "legs_ids: ";
    to_flow_style_yaml(msg.legs_ids, out);
    out << ", ";
  }

  // member: velocity_mode_direction
  {
    out << "velocity_mode_direction: ";
    to_flow_style_yaml(msg.velocity_mode_direction, out);
    out << ", ";
  }

  // member: threshold_type
  {
    out << "threshold_type: ";
    rosidl_generator_traits::value_to_yaml(msg.threshold_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveLegVelocityMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: legs_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "legs_ids:\n";
    to_block_style_yaml(msg.legs_ids, out, indentation + 2);
  }

  // member: velocity_mode_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_mode_direction:\n";
    to_block_style_yaml(msg.velocity_mode_direction, out, indentation + 2);
  }

  // member: threshold_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "threshold_type: ";
    rosidl_generator_traits::value_to_yaml(msg.threshold_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveLegVelocityMode_Request & msg, bool use_flow_style = false)
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
  const gwpspider_interfaces::srv::MoveLegVelocityMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  gwpspider_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gwpspider_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const gwpspider_interfaces::srv::MoveLegVelocityMode_Request & msg)
{
  return gwpspider_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gwpspider_interfaces::srv::MoveLegVelocityMode_Request>()
{
  return "gwpspider_interfaces::srv::MoveLegVelocityMode_Request";
}

template<>
inline const char * name<gwpspider_interfaces::srv::MoveLegVelocityMode_Request>()
{
  return "gwpspider_interfaces/srv/MoveLegVelocityMode_Request";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::MoveLegVelocityMode_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::MoveLegVelocityMode_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<gwpspider_interfaces::srv::MoveLegVelocityMode_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace gwpspider_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveLegVelocityMode_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveLegVelocityMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveLegVelocityMode_Response & msg, bool use_flow_style = false)
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
  const gwpspider_interfaces::srv::MoveLegVelocityMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  gwpspider_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gwpspider_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const gwpspider_interfaces::srv::MoveLegVelocityMode_Response & msg)
{
  return gwpspider_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gwpspider_interfaces::srv::MoveLegVelocityMode_Response>()
{
  return "gwpspider_interfaces::srv::MoveLegVelocityMode_Response";
}

template<>
inline const char * name<gwpspider_interfaces::srv::MoveLegVelocityMode_Response>()
{
  return "gwpspider_interfaces/srv/MoveLegVelocityMode_Response";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::MoveLegVelocityMode_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::MoveLegVelocityMode_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<gwpspider_interfaces::srv::MoveLegVelocityMode_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<gwpspider_interfaces::srv::MoveLegVelocityMode>()
{
  return "gwpspider_interfaces::srv::MoveLegVelocityMode";
}

template<>
inline const char * name<gwpspider_interfaces::srv::MoveLegVelocityMode>()
{
  return "gwpspider_interfaces/srv/MoveLegVelocityMode";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::MoveLegVelocityMode>
  : std::integral_constant<
    bool,
    has_fixed_size<gwpspider_interfaces::srv::MoveLegVelocityMode_Request>::value &&
    has_fixed_size<gwpspider_interfaces::srv::MoveLegVelocityMode_Response>::value
  >
{
};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::MoveLegVelocityMode>
  : std::integral_constant<
    bool,
    has_bounded_size<gwpspider_interfaces::srv::MoveLegVelocityMode_Request>::value &&
    has_bounded_size<gwpspider_interfaces::srv::MoveLegVelocityMode_Response>::value
  >
{
};

template<>
struct is_service<gwpspider_interfaces::srv::MoveLegVelocityMode>
  : std::true_type
{
};

template<>
struct is_service_request<gwpspider_interfaces::srv::MoveLegVelocityMode_Request>
  : std::true_type
{
};

template<>
struct is_service_response<gwpspider_interfaces::srv::MoveLegVelocityMode_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_LEG_VELOCITY_MODE__TRAITS_HPP_

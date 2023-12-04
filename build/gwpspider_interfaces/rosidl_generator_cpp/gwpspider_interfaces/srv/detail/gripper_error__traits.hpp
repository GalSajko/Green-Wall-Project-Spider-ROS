// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gwpspider_interfaces:srv/GripperError.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__GRIPPER_ERROR__TRAITS_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__GRIPPER_ERROR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "gwpspider_interfaces/srv/detail/gripper_error__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace gwpspider_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GripperError_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: monitor
  {
    out << "monitor: ";
    rosidl_generator_traits::value_to_yaml(msg.monitor, out);
    out << ", ";
  }

  // member: leg_index
  {
    out << "leg_index: ";
    rosidl_generator_traits::value_to_yaml(msg.leg_index, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperError_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: monitor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "monitor: ";
    rosidl_generator_traits::value_to_yaml(msg.monitor, out);
    out << "\n";
  }

  // member: leg_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "leg_index: ";
    rosidl_generator_traits::value_to_yaml(msg.leg_index, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperError_Request & msg, bool use_flow_style = false)
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
  const gwpspider_interfaces::srv::GripperError_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  gwpspider_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gwpspider_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const gwpspider_interfaces::srv::GripperError_Request & msg)
{
  return gwpspider_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gwpspider_interfaces::srv::GripperError_Request>()
{
  return "gwpspider_interfaces::srv::GripperError_Request";
}

template<>
inline const char * name<gwpspider_interfaces::srv::GripperError_Request>()
{
  return "gwpspider_interfaces/srv/GripperError_Request";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::GripperError_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::GripperError_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<gwpspider_interfaces::srv::GripperError_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace gwpspider_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GripperError_Response & msg,
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
  const GripperError_Response & msg,
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

inline std::string to_yaml(const GripperError_Response & msg, bool use_flow_style = false)
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
  const gwpspider_interfaces::srv::GripperError_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  gwpspider_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gwpspider_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const gwpspider_interfaces::srv::GripperError_Response & msg)
{
  return gwpspider_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gwpspider_interfaces::srv::GripperError_Response>()
{
  return "gwpspider_interfaces::srv::GripperError_Response";
}

template<>
inline const char * name<gwpspider_interfaces::srv::GripperError_Response>()
{
  return "gwpspider_interfaces/srv/GripperError_Response";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::GripperError_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::GripperError_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<gwpspider_interfaces::srv::GripperError_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<gwpspider_interfaces::srv::GripperError>()
{
  return "gwpspider_interfaces::srv::GripperError";
}

template<>
inline const char * name<gwpspider_interfaces::srv::GripperError>()
{
  return "gwpspider_interfaces/srv/GripperError";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::GripperError>
  : std::integral_constant<
    bool,
    has_fixed_size<gwpspider_interfaces::srv::GripperError_Request>::value &&
    has_fixed_size<gwpspider_interfaces::srv::GripperError_Response>::value
  >
{
};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::GripperError>
  : std::integral_constant<
    bool,
    has_bounded_size<gwpspider_interfaces::srv::GripperError_Request>::value &&
    has_bounded_size<gwpspider_interfaces::srv::GripperError_Response>::value
  >
{
};

template<>
struct is_service<gwpspider_interfaces::srv::GripperError>
  : std::true_type
{
};

template<>
struct is_service_request<gwpspider_interfaces::srv::GripperError_Request>
  : std::true_type
{
};

template<>
struct is_service_response<gwpspider_interfaces::srv::GripperError_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__GRIPPER_ERROR__TRAITS_HPP_

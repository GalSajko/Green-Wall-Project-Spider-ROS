// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gwpspider_interfaces:srv/MoveSpider.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_SPIDER__TRAITS_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_SPIDER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "gwpspider_interfaces/srv/detail/move_spider__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'legs'
#include "std_msgs/msg/detail/int16_multi_array__traits.hpp"
// Member 'goal_positions'
// Member 'spider_pose'
#include "std_msgs/msg/detail/float32_multi_array__traits.hpp"

namespace gwpspider_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveSpider_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: legs
  {
    out << "legs: ";
    to_flow_style_yaml(msg.legs, out);
    out << ", ";
  }

  // member: goal_positions
  {
    out << "goal_positions: ";
    to_flow_style_yaml(msg.goal_positions, out);
    out << ", ";
  }

  // member: spider_pose
  {
    out << "spider_pose: ";
    to_flow_style_yaml(msg.spider_pose, out);
    out << ", ";
  }

  // member: trajectory_type
  {
    out << "trajectory_type: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_type, out);
    out << ", ";
  }

  // member: duration
  {
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveSpider_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: legs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "legs:\n";
    to_block_style_yaml(msg.legs, out, indentation + 2);
  }

  // member: goal_positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_positions:\n";
    to_block_style_yaml(msg.goal_positions, out, indentation + 2);
  }

  // member: spider_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spider_pose:\n";
    to_block_style_yaml(msg.spider_pose, out, indentation + 2);
  }

  // member: trajectory_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trajectory_type: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_type, out);
    out << "\n";
  }

  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveSpider_Request & msg, bool use_flow_style = false)
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
  const gwpspider_interfaces::srv::MoveSpider_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  gwpspider_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gwpspider_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const gwpspider_interfaces::srv::MoveSpider_Request & msg)
{
  return gwpspider_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gwpspider_interfaces::srv::MoveSpider_Request>()
{
  return "gwpspider_interfaces::srv::MoveSpider_Request";
}

template<>
inline const char * name<gwpspider_interfaces::srv::MoveSpider_Request>()
{
  return "gwpspider_interfaces/srv/MoveSpider_Request";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::MoveSpider_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::MoveSpider_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<gwpspider_interfaces::srv::MoveSpider_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace gwpspider_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveSpider_Response & msg,
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
  const MoveSpider_Response & msg,
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

inline std::string to_yaml(const MoveSpider_Response & msg, bool use_flow_style = false)
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
  const gwpspider_interfaces::srv::MoveSpider_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  gwpspider_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gwpspider_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const gwpspider_interfaces::srv::MoveSpider_Response & msg)
{
  return gwpspider_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gwpspider_interfaces::srv::MoveSpider_Response>()
{
  return "gwpspider_interfaces::srv::MoveSpider_Response";
}

template<>
inline const char * name<gwpspider_interfaces::srv::MoveSpider_Response>()
{
  return "gwpspider_interfaces/srv/MoveSpider_Response";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::MoveSpider_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::MoveSpider_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<gwpspider_interfaces::srv::MoveSpider_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<gwpspider_interfaces::srv::MoveSpider>()
{
  return "gwpspider_interfaces::srv::MoveSpider";
}

template<>
inline const char * name<gwpspider_interfaces::srv::MoveSpider>()
{
  return "gwpspider_interfaces/srv/MoveSpider";
}

template<>
struct has_fixed_size<gwpspider_interfaces::srv::MoveSpider>
  : std::integral_constant<
    bool,
    has_fixed_size<gwpspider_interfaces::srv::MoveSpider_Request>::value &&
    has_fixed_size<gwpspider_interfaces::srv::MoveSpider_Response>::value
  >
{
};

template<>
struct has_bounded_size<gwpspider_interfaces::srv::MoveSpider>
  : std::integral_constant<
    bool,
    has_bounded_size<gwpspider_interfaces::srv::MoveSpider_Request>::value &&
    has_bounded_size<gwpspider_interfaces::srv::MoveSpider_Response>::value
  >
{
};

template<>
struct is_service<gwpspider_interfaces::srv::MoveSpider>
  : std::true_type
{
};

template<>
struct is_service_request<gwpspider_interfaces::srv::MoveSpider_Request>
  : std::true_type
{
};

template<>
struct is_service_response<gwpspider_interfaces::srv::MoveSpider_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_SPIDER__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gwpspider_interfaces:srv/MoveLegVelocityMode.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_LEG_VELOCITY_MODE__STRUCT_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_LEG_VELOCITY_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'legs_ids'
#include "std_msgs/msg/detail/int8_multi_array__struct.hpp"
// Member 'velocity_mode_direction'
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gwpspider_interfaces__srv__MoveLegVelocityMode_Request __attribute__((deprecated))
#else
# define DEPRECATED__gwpspider_interfaces__srv__MoveLegVelocityMode_Request __declspec(deprecated)
#endif

namespace gwpspider_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveLegVelocityMode_Request_
{
  using Type = MoveLegVelocityMode_Request_<ContainerAllocator>;

  explicit MoveLegVelocityMode_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : legs_ids(_init),
    velocity_mode_direction(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->threshold_type = "";
    }
  }

  explicit MoveLegVelocityMode_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : legs_ids(_alloc, _init),
    velocity_mode_direction(_alloc, _init),
    threshold_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->threshold_type = "";
    }
  }

  // field types and members
  using _legs_ids_type =
    std_msgs::msg::Int8MultiArray_<ContainerAllocator>;
  _legs_ids_type legs_ids;
  using _velocity_mode_direction_type =
    std_msgs::msg::Float32MultiArray_<ContainerAllocator>;
  _velocity_mode_direction_type velocity_mode_direction;
  using _threshold_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _threshold_type_type threshold_type;

  // setters for named parameter idiom
  Type & set__legs_ids(
    const std_msgs::msg::Int8MultiArray_<ContainerAllocator> & _arg)
  {
    this->legs_ids = _arg;
    return *this;
  }
  Type & set__velocity_mode_direction(
    const std_msgs::msg::Float32MultiArray_<ContainerAllocator> & _arg)
  {
    this->velocity_mode_direction = _arg;
    return *this;
  }
  Type & set__threshold_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->threshold_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gwpspider_interfaces__srv__MoveLegVelocityMode_Request
    std::shared_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gwpspider_interfaces__srv__MoveLegVelocityMode_Request
    std::shared_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveLegVelocityMode_Request_ & other) const
  {
    if (this->legs_ids != other.legs_ids) {
      return false;
    }
    if (this->velocity_mode_direction != other.velocity_mode_direction) {
      return false;
    }
    if (this->threshold_type != other.threshold_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveLegVelocityMode_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveLegVelocityMode_Request_

// alias to use template instance with default allocator
using MoveLegVelocityMode_Request =
  gwpspider_interfaces::srv::MoveLegVelocityMode_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gwpspider_interfaces


#ifndef _WIN32
# define DEPRECATED__gwpspider_interfaces__srv__MoveLegVelocityMode_Response __attribute__((deprecated))
#else
# define DEPRECATED__gwpspider_interfaces__srv__MoveLegVelocityMode_Response __declspec(deprecated)
#endif

namespace gwpspider_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveLegVelocityMode_Response_
{
  using Type = MoveLegVelocityMode_Response_<ContainerAllocator>;

  explicit MoveLegVelocityMode_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit MoveLegVelocityMode_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gwpspider_interfaces__srv__MoveLegVelocityMode_Response
    std::shared_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gwpspider_interfaces__srv__MoveLegVelocityMode_Response
    std::shared_ptr<gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveLegVelocityMode_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveLegVelocityMode_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveLegVelocityMode_Response_

// alias to use template instance with default allocator
using MoveLegVelocityMode_Response =
  gwpspider_interfaces::srv::MoveLegVelocityMode_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gwpspider_interfaces

namespace gwpspider_interfaces
{

namespace srv
{

struct MoveLegVelocityMode
{
  using Request = gwpspider_interfaces::srv::MoveLegVelocityMode_Request;
  using Response = gwpspider_interfaces::srv::MoveLegVelocityMode_Response;
};

}  // namespace srv

}  // namespace gwpspider_interfaces

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_LEG_VELOCITY_MODE__STRUCT_HPP_

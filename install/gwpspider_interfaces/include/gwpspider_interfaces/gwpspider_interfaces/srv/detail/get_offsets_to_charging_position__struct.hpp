// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gwpspider_interfaces:srv/GetOffsetsToChargingPosition.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__GET_OFFSETS_TO_CHARGING_POSITION__STRUCT_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__GET_OFFSETS_TO_CHARGING_POSITION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'current_pins'
#include "std_msgs/msg/detail/int16_multi_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request __attribute__((deprecated))
#else
# define DEPRECATED__gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request __declspec(deprecated)
#endif

namespace gwpspider_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetOffsetsToChargingPosition_Request_
{
  using Type = GetOffsetsToChargingPosition_Request_<ContainerAllocator>;

  explicit GetOffsetsToChargingPosition_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_pins(_init)
  {
    (void)_init;
  }

  explicit GetOffsetsToChargingPosition_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_pins(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _current_pins_type =
    std_msgs::msg::Int16MultiArray_<ContainerAllocator>;
  _current_pins_type current_pins;

  // setters for named parameter idiom
  Type & set__current_pins(
    const std_msgs::msg::Int16MultiArray_<ContainerAllocator> & _arg)
  {
    this->current_pins = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request
    std::shared_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request
    std::shared_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetOffsetsToChargingPosition_Request_ & other) const
  {
    if (this->current_pins != other.current_pins) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetOffsetsToChargingPosition_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetOffsetsToChargingPosition_Request_

// alias to use template instance with default allocator
using GetOffsetsToChargingPosition_Request =
  gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gwpspider_interfaces


// Include directives for member types
// Member 'offsets'
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response __attribute__((deprecated))
#else
# define DEPRECATED__gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response __declspec(deprecated)
#endif

namespace gwpspider_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetOffsetsToChargingPosition_Response_
{
  using Type = GetOffsetsToChargingPosition_Response_<ContainerAllocator>;

  explicit GetOffsetsToChargingPosition_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : offsets(_init)
  {
    (void)_init;
  }

  explicit GetOffsetsToChargingPosition_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : offsets(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _offsets_type =
    std_msgs::msg::Float32MultiArray_<ContainerAllocator>;
  _offsets_type offsets;

  // setters for named parameter idiom
  Type & set__offsets(
    const std_msgs::msg::Float32MultiArray_<ContainerAllocator> & _arg)
  {
    this->offsets = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response
    std::shared_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response
    std::shared_ptr<gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetOffsetsToChargingPosition_Response_ & other) const
  {
    if (this->offsets != other.offsets) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetOffsetsToChargingPosition_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetOffsetsToChargingPosition_Response_

// alias to use template instance with default allocator
using GetOffsetsToChargingPosition_Response =
  gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gwpspider_interfaces

namespace gwpspider_interfaces
{

namespace srv
{

struct GetOffsetsToChargingPosition
{
  using Request = gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Request;
  using Response = gwpspider_interfaces::srv::GetOffsetsToChargingPosition_Response;
};

}  // namespace srv

}  // namespace gwpspider_interfaces

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__GET_OFFSETS_TO_CHARGING_POSITION__STRUCT_HPP_

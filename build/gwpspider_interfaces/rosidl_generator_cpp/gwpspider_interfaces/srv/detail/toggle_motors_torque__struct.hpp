// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gwpspider_interfaces:srv/ToggleMotorsTorque.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__TOGGLE_MOTORS_TORQUE__STRUCT_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__TOGGLE_MOTORS_TORQUE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'motors_ids'
#include "std_msgs/msg/detail/int8_multi_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gwpspider_interfaces__srv__ToggleMotorsTorque_Request __attribute__((deprecated))
#else
# define DEPRECATED__gwpspider_interfaces__srv__ToggleMotorsTorque_Request __declspec(deprecated)
#endif

namespace gwpspider_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ToggleMotorsTorque_Request_
{
  using Type = ToggleMotorsTorque_Request_<ContainerAllocator>;

  explicit ToggleMotorsTorque_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motors_ids(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
    }
  }

  explicit ToggleMotorsTorque_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motors_ids(_alloc, _init),
    command(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
    }
  }

  // field types and members
  using _motors_ids_type =
    std_msgs::msg::Int8MultiArray_<ContainerAllocator>;
  _motors_ids_type motors_ids;
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;

  // setters for named parameter idiom
  Type & set__motors_ids(
    const std_msgs::msg::Int8MultiArray_<ContainerAllocator> & _arg)
  {
    this->motors_ids = _arg;
    return *this;
  }
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gwpspider_interfaces__srv__ToggleMotorsTorque_Request
    std::shared_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gwpspider_interfaces__srv__ToggleMotorsTorque_Request
    std::shared_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToggleMotorsTorque_Request_ & other) const
  {
    if (this->motors_ids != other.motors_ids) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToggleMotorsTorque_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToggleMotorsTorque_Request_

// alias to use template instance with default allocator
using ToggleMotorsTorque_Request =
  gwpspider_interfaces::srv::ToggleMotorsTorque_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gwpspider_interfaces


#ifndef _WIN32
# define DEPRECATED__gwpspider_interfaces__srv__ToggleMotorsTorque_Response __attribute__((deprecated))
#else
# define DEPRECATED__gwpspider_interfaces__srv__ToggleMotorsTorque_Response __declspec(deprecated)
#endif

namespace gwpspider_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ToggleMotorsTorque_Response_
{
  using Type = ToggleMotorsTorque_Response_<ContainerAllocator>;

  explicit ToggleMotorsTorque_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ToggleMotorsTorque_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gwpspider_interfaces__srv__ToggleMotorsTorque_Response
    std::shared_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gwpspider_interfaces__srv__ToggleMotorsTorque_Response
    std::shared_ptr<gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToggleMotorsTorque_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToggleMotorsTorque_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToggleMotorsTorque_Response_

// alias to use template instance with default allocator
using ToggleMotorsTorque_Response =
  gwpspider_interfaces::srv::ToggleMotorsTorque_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gwpspider_interfaces

namespace gwpspider_interfaces
{

namespace srv
{

struct ToggleMotorsTorque
{
  using Request = gwpspider_interfaces::srv::ToggleMotorsTorque_Request;
  using Response = gwpspider_interfaces::srv::ToggleMotorsTorque_Response;
};

}  // namespace srv

}  // namespace gwpspider_interfaces

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__TOGGLE_MOTORS_TORQUE__STRUCT_HPP_

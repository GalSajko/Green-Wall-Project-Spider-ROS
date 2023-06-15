// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gwpspider_interfaces:srv/MoveGripper.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_GRIPPER__STRUCT_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_GRIPPER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'instructions'
#include "gwpspider_interfaces/msg/detail/gripper_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__gwpspider_interfaces__srv__MoveGripper_Request __attribute__((deprecated))
#else
# define DEPRECATED__gwpspider_interfaces__srv__MoveGripper_Request __declspec(deprecated)
#endif

namespace gwpspider_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveGripper_Request_
{
  using Type = MoveGripper_Request_<ContainerAllocator>;

  explicit MoveGripper_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : instructions(_init)
  {
    (void)_init;
  }

  explicit MoveGripper_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : instructions(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _instructions_type =
    gwpspider_interfaces::msg::GripperCommand_<ContainerAllocator>;
  _instructions_type instructions;

  // setters for named parameter idiom
  Type & set__instructions(
    const gwpspider_interfaces::msg::GripperCommand_<ContainerAllocator> & _arg)
  {
    this->instructions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gwpspider_interfaces__srv__MoveGripper_Request
    std::shared_ptr<gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gwpspider_interfaces__srv__MoveGripper_Request
    std::shared_ptr<gwpspider_interfaces::srv::MoveGripper_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveGripper_Request_ & other) const
  {
    if (this->instructions != other.instructions) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveGripper_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveGripper_Request_

// alias to use template instance with default allocator
using MoveGripper_Request =
  gwpspider_interfaces::srv::MoveGripper_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gwpspider_interfaces


#ifndef _WIN32
# define DEPRECATED__gwpspider_interfaces__srv__MoveGripper_Response __attribute__((deprecated))
#else
# define DEPRECATED__gwpspider_interfaces__srv__MoveGripper_Response __declspec(deprecated)
#endif

namespace gwpspider_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveGripper_Response_
{
  using Type = MoveGripper_Response_<ContainerAllocator>;

  explicit MoveGripper_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit MoveGripper_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gwpspider_interfaces__srv__MoveGripper_Response
    std::shared_ptr<gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gwpspider_interfaces__srv__MoveGripper_Response
    std::shared_ptr<gwpspider_interfaces::srv::MoveGripper_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveGripper_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveGripper_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveGripper_Response_

// alias to use template instance with default allocator
using MoveGripper_Response =
  gwpspider_interfaces::srv::MoveGripper_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gwpspider_interfaces

namespace gwpspider_interfaces
{

namespace srv
{

struct MoveGripper
{
  using Request = gwpspider_interfaces::srv::MoveGripper_Request;
  using Response = gwpspider_interfaces::srv::MoveGripper_Response;
};

}  // namespace srv

}  // namespace gwpspider_interfaces

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__MOVE_GRIPPER__STRUCT_HPP_

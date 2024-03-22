// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gwpspider_interfaces:srv/GripperError.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__GRIPPER_ERROR__STRUCT_HPP_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__GRIPPER_ERROR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__gwpspider_interfaces__srv__GripperError_Request __attribute__((deprecated))
#else
# define DEPRECATED__gwpspider_interfaces__srv__GripperError_Request __declspec(deprecated)
#endif

namespace gwpspider_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GripperError_Request_
{
  using Type = GripperError_Request_<ContainerAllocator>;

  explicit GripperError_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->monitor = false;
      this->leg_index = 0l;
    }
  }

  explicit GripperError_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->monitor = false;
      this->leg_index = 0l;
    }
  }

  // field types and members
  using _monitor_type =
    bool;
  _monitor_type monitor;
  using _leg_index_type =
    int32_t;
  _leg_index_type leg_index;

  // setters for named parameter idiom
  Type & set__monitor(
    const bool & _arg)
  {
    this->monitor = _arg;
    return *this;
  }
  Type & set__leg_index(
    const int32_t & _arg)
  {
    this->leg_index = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gwpspider_interfaces__srv__GripperError_Request
    std::shared_ptr<gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gwpspider_interfaces__srv__GripperError_Request
    std::shared_ptr<gwpspider_interfaces::srv::GripperError_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripperError_Request_ & other) const
  {
    if (this->monitor != other.monitor) {
      return false;
    }
    if (this->leg_index != other.leg_index) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripperError_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripperError_Request_

// alias to use template instance with default allocator
using GripperError_Request =
  gwpspider_interfaces::srv::GripperError_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gwpspider_interfaces


#ifndef _WIN32
# define DEPRECATED__gwpspider_interfaces__srv__GripperError_Response __attribute__((deprecated))
#else
# define DEPRECATED__gwpspider_interfaces__srv__GripperError_Response __declspec(deprecated)
#endif

namespace gwpspider_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GripperError_Response_
{
  using Type = GripperError_Response_<ContainerAllocator>;

  explicit GripperError_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit GripperError_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gwpspider_interfaces__srv__GripperError_Response
    std::shared_ptr<gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gwpspider_interfaces__srv__GripperError_Response
    std::shared_ptr<gwpspider_interfaces::srv::GripperError_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripperError_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripperError_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripperError_Response_

// alias to use template instance with default allocator
using GripperError_Response =
  gwpspider_interfaces::srv::GripperError_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gwpspider_interfaces

namespace gwpspider_interfaces
{

namespace srv
{

struct GripperError
{
  using Request = gwpspider_interfaces::srv::GripperError_Request;
  using Response = gwpspider_interfaces::srv::GripperError_Response;
};

}  // namespace srv

}  // namespace gwpspider_interfaces

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__GRIPPER_ERROR__STRUCT_HPP_

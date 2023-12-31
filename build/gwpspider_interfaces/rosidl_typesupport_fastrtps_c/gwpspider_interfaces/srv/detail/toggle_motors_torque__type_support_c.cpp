// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from gwpspider_interfaces:srv/ToggleMotorsTorque.idl
// generated code does not contain a copyright notice
#include "gwpspider_interfaces/srv/detail/toggle_motors_torque__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "gwpspider_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "gwpspider_interfaces/srv/detail/toggle_motors_torque__struct.h"
#include "gwpspider_interfaces/srv/detail/toggle_motors_torque__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // command
#include "rosidl_runtime_c/string_functions.h"  // command
#include "std_msgs/msg/detail/int8_multi_array__functions.h"  // motors_ids

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gwpspider_interfaces
size_t get_serialized_size_std_msgs__msg__Int8MultiArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gwpspider_interfaces
size_t max_serialized_size_std_msgs__msg__Int8MultiArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gwpspider_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Int8MultiArray)();


using _ToggleMotorsTorque_Request__ros_msg_type = gwpspider_interfaces__srv__ToggleMotorsTorque_Request;

static bool _ToggleMotorsTorque_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ToggleMotorsTorque_Request__ros_msg_type * ros_message = static_cast<const _ToggleMotorsTorque_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: motors_ids
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Int8MultiArray
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->motors_ids, cdr))
    {
      return false;
    }
  }

  // Field name: command
  {
    const rosidl_runtime_c__String * str = &ros_message->command;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _ToggleMotorsTorque_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ToggleMotorsTorque_Request__ros_msg_type * ros_message = static_cast<_ToggleMotorsTorque_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: motors_ids
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Int8MultiArray
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->motors_ids))
    {
      return false;
    }
  }

  // Field name: command
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->command.data) {
      rosidl_runtime_c__String__init(&ros_message->command);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->command,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'command'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_gwpspider_interfaces
size_t get_serialized_size_gwpspider_interfaces__srv__ToggleMotorsTorque_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ToggleMotorsTorque_Request__ros_msg_type * ros_message = static_cast<const _ToggleMotorsTorque_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name motors_ids

  current_alignment += get_serialized_size_std_msgs__msg__Int8MultiArray(
    &(ros_message->motors_ids), current_alignment);
  // field.name command
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->command.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _ToggleMotorsTorque_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_gwpspider_interfaces__srv__ToggleMotorsTorque_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_gwpspider_interfaces
size_t max_serialized_size_gwpspider_interfaces__srv__ToggleMotorsTorque_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: motors_ids
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_std_msgs__msg__Int8MultiArray(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: command
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _ToggleMotorsTorque_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_gwpspider_interfaces__srv__ToggleMotorsTorque_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ToggleMotorsTorque_Request = {
  "gwpspider_interfaces::srv",
  "ToggleMotorsTorque_Request",
  _ToggleMotorsTorque_Request__cdr_serialize,
  _ToggleMotorsTorque_Request__cdr_deserialize,
  _ToggleMotorsTorque_Request__get_serialized_size,
  _ToggleMotorsTorque_Request__max_serialized_size
};

static rosidl_message_type_support_t _ToggleMotorsTorque_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ToggleMotorsTorque_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gwpspider_interfaces, srv, ToggleMotorsTorque_Request)() {
  return &_ToggleMotorsTorque_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "gwpspider_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "gwpspider_interfaces/srv/detail/toggle_motors_torque__struct.h"
// already included above
// #include "gwpspider_interfaces/srv/detail/toggle_motors_torque__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ToggleMotorsTorque_Response__ros_msg_type = gwpspider_interfaces__srv__ToggleMotorsTorque_Response;

static bool _ToggleMotorsTorque_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ToggleMotorsTorque_Response__ros_msg_type * ros_message = static_cast<const _ToggleMotorsTorque_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

static bool _ToggleMotorsTorque_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ToggleMotorsTorque_Response__ros_msg_type * ros_message = static_cast<_ToggleMotorsTorque_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_gwpspider_interfaces
size_t get_serialized_size_gwpspider_interfaces__srv__ToggleMotorsTorque_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ToggleMotorsTorque_Response__ros_msg_type * ros_message = static_cast<const _ToggleMotorsTorque_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ToggleMotorsTorque_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_gwpspider_interfaces__srv__ToggleMotorsTorque_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_gwpspider_interfaces
size_t max_serialized_size_gwpspider_interfaces__srv__ToggleMotorsTorque_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: success
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _ToggleMotorsTorque_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_gwpspider_interfaces__srv__ToggleMotorsTorque_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ToggleMotorsTorque_Response = {
  "gwpspider_interfaces::srv",
  "ToggleMotorsTorque_Response",
  _ToggleMotorsTorque_Response__cdr_serialize,
  _ToggleMotorsTorque_Response__cdr_deserialize,
  _ToggleMotorsTorque_Response__get_serialized_size,
  _ToggleMotorsTorque_Response__max_serialized_size
};

static rosidl_message_type_support_t _ToggleMotorsTorque_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ToggleMotorsTorque_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gwpspider_interfaces, srv, ToggleMotorsTorque_Response)() {
  return &_ToggleMotorsTorque_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "gwpspider_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "gwpspider_interfaces/srv/toggle_motors_torque.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t ToggleMotorsTorque__callbacks = {
  "gwpspider_interfaces::srv",
  "ToggleMotorsTorque",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gwpspider_interfaces, srv, ToggleMotorsTorque_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gwpspider_interfaces, srv, ToggleMotorsTorque_Response)(),
};

static rosidl_service_type_support_t ToggleMotorsTorque__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &ToggleMotorsTorque__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gwpspider_interfaces, srv, ToggleMotorsTorque)() {
  return &ToggleMotorsTorque__handle;
}

#if defined(__cplusplus)
}
#endif

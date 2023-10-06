// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from gwpspider_interfaces:srv/GetOffsetsToChargingPosition.idl
// generated code does not contain a copyright notice
#include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "gwpspider_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__struct.h"
#include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__functions.h"
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

#include "std_msgs/msg/detail/int16_multi_array__functions.h"  // current_pins

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gwpspider_interfaces
size_t get_serialized_size_std_msgs__msg__Int16MultiArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gwpspider_interfaces
size_t max_serialized_size_std_msgs__msg__Int16MultiArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gwpspider_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Int16MultiArray)();


using _GetOffsetsToChargingPosition_Request__ros_msg_type = gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request;

static bool _GetOffsetsToChargingPosition_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetOffsetsToChargingPosition_Request__ros_msg_type * ros_message = static_cast<const _GetOffsetsToChargingPosition_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: current_pins
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Int16MultiArray
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->current_pins, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _GetOffsetsToChargingPosition_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetOffsetsToChargingPosition_Request__ros_msg_type * ros_message = static_cast<_GetOffsetsToChargingPosition_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: current_pins
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Int16MultiArray
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->current_pins))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_gwpspider_interfaces
size_t get_serialized_size_gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetOffsetsToChargingPosition_Request__ros_msg_type * ros_message = static_cast<const _GetOffsetsToChargingPosition_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name current_pins

  current_alignment += get_serialized_size_std_msgs__msg__Int16MultiArray(
    &(ros_message->current_pins), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _GetOffsetsToChargingPosition_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_gwpspider_interfaces
size_t max_serialized_size_gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request(
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

  // member: current_pins
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_std_msgs__msg__Int16MultiArray(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _GetOffsetsToChargingPosition_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetOffsetsToChargingPosition_Request = {
  "gwpspider_interfaces::srv",
  "GetOffsetsToChargingPosition_Request",
  _GetOffsetsToChargingPosition_Request__cdr_serialize,
  _GetOffsetsToChargingPosition_Request__cdr_deserialize,
  _GetOffsetsToChargingPosition_Request__get_serialized_size,
  _GetOffsetsToChargingPosition_Request__max_serialized_size
};

static rosidl_message_type_support_t _GetOffsetsToChargingPosition_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetOffsetsToChargingPosition_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition_Request)() {
  return &_GetOffsetsToChargingPosition_Request__type_support;
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
// #include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__struct.h"
// already included above
// #include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__functions.h"
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

#include "std_msgs/msg/detail/float32_multi_array__functions.h"  // offsets

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gwpspider_interfaces
size_t get_serialized_size_std_msgs__msg__Float32MultiArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gwpspider_interfaces
size_t max_serialized_size_std_msgs__msg__Float32MultiArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gwpspider_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Float32MultiArray)();


using _GetOffsetsToChargingPosition_Response__ros_msg_type = gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response;

static bool _GetOffsetsToChargingPosition_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetOffsetsToChargingPosition_Response__ros_msg_type * ros_message = static_cast<const _GetOffsetsToChargingPosition_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: offsets
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Float32MultiArray
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->offsets, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _GetOffsetsToChargingPosition_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetOffsetsToChargingPosition_Response__ros_msg_type * ros_message = static_cast<_GetOffsetsToChargingPosition_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: offsets
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Float32MultiArray
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->offsets))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_gwpspider_interfaces
size_t get_serialized_size_gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetOffsetsToChargingPosition_Response__ros_msg_type * ros_message = static_cast<const _GetOffsetsToChargingPosition_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name offsets

  current_alignment += get_serialized_size_std_msgs__msg__Float32MultiArray(
    &(ros_message->offsets), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _GetOffsetsToChargingPosition_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_gwpspider_interfaces
size_t max_serialized_size_gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response(
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

  // member: offsets
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_std_msgs__msg__Float32MultiArray(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _GetOffsetsToChargingPosition_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetOffsetsToChargingPosition_Response = {
  "gwpspider_interfaces::srv",
  "GetOffsetsToChargingPosition_Response",
  _GetOffsetsToChargingPosition_Response__cdr_serialize,
  _GetOffsetsToChargingPosition_Response__cdr_deserialize,
  _GetOffsetsToChargingPosition_Response__get_serialized_size,
  _GetOffsetsToChargingPosition_Response__max_serialized_size
};

static rosidl_message_type_support_t _GetOffsetsToChargingPosition_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetOffsetsToChargingPosition_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition_Response)() {
  return &_GetOffsetsToChargingPosition_Response__type_support;
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
#include "gwpspider_interfaces/srv/get_offsets_to_charging_position.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GetOffsetsToChargingPosition__callbacks = {
  "gwpspider_interfaces::srv",
  "GetOffsetsToChargingPosition",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition_Response)(),
};

static rosidl_service_type_support_t GetOffsetsToChargingPosition__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GetOffsetsToChargingPosition__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition)() {
  return &GetOffsetsToChargingPosition__handle;
}

#if defined(__cplusplus)
}
#endif

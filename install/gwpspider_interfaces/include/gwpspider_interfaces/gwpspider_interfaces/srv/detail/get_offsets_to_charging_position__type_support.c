// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from gwpspider_interfaces:srv/GetOffsetsToChargingPosition.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__rosidl_typesupport_introspection_c.h"
#include "gwpspider_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__functions.h"
#include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__struct.h"


// Include directives for member types
// Member `current_pins`
#include "std_msgs/msg/int16_multi_array.h"
// Member `current_pins`
#include "std_msgs/msg/detail/int16_multi_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__init(message_memory);
}

void gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_fini_function(void * message_memory)
{
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_message_member_array[1] = {
  {
    "current_pins",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request, current_pins),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_message_members = {
  "gwpspider_interfaces__srv",  // message namespace
  "GetOffsetsToChargingPosition_Request",  // message name
  1,  // number of fields
  sizeof(gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request),
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_message_member_array,  // message members
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_message_type_support_handle = {
  0,
  &gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gwpspider_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition_Request)() {
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int16MultiArray)();
  if (!gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_message_type_support_handle.typesupport_identifier) {
    gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Request__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__rosidl_typesupport_introspection_c.h"
// already included above
// #include "gwpspider_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__functions.h"
// already included above
// #include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__struct.h"


// Include directives for member types
// Member `offsets`
#include "std_msgs/msg/float32_multi_array.h"
// Member `offsets`
#include "std_msgs/msg/detail/float32_multi_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__init(message_memory);
}

void gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_fini_function(void * message_memory)
{
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_message_member_array[1] = {
  {
    "offsets",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response, offsets),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_message_members = {
  "gwpspider_interfaces__srv",  // message namespace
  "GetOffsetsToChargingPosition_Response",  // message name
  1,  // number of fields
  sizeof(gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response),
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_message_member_array,  // message members
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_message_type_support_handle = {
  0,
  &gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gwpspider_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition_Response)() {
  gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32MultiArray)();
  if (!gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_message_type_support_handle.typesupport_identifier) {
    gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gwpspider_interfaces__srv__GetOffsetsToChargingPosition_Response__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "gwpspider_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "gwpspider_interfaces/srv/detail/get_offsets_to_charging_position__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers gwpspider_interfaces__srv__detail__get_offsets_to_charging_position__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_service_members = {
  "gwpspider_interfaces__srv",  // service namespace
  "GetOffsetsToChargingPosition",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // gwpspider_interfaces__srv__detail__get_offsets_to_charging_position__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Request_message_type_support_handle,
  NULL  // response message
  // gwpspider_interfaces__srv__detail__get_offsets_to_charging_position__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_Response_message_type_support_handle
};

static rosidl_service_type_support_t gwpspider_interfaces__srv__detail__get_offsets_to_charging_position__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_service_type_support_handle = {
  0,
  &gwpspider_interfaces__srv__detail__get_offsets_to_charging_position__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gwpspider_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition)() {
  if (!gwpspider_interfaces__srv__detail__get_offsets_to_charging_position__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_service_type_support_handle.typesupport_identifier) {
    gwpspider_interfaces__srv__detail__get_offsets_to_charging_position__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)gwpspider_interfaces__srv__detail__get_offsets_to_charging_position__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gwpspider_interfaces, srv, GetOffsetsToChargingPosition_Response)()->data;
  }

  return &gwpspider_interfaces__srv__detail__get_offsets_to_charging_position__rosidl_typesupport_introspection_c__GetOffsetsToChargingPosition_service_type_support_handle;
}

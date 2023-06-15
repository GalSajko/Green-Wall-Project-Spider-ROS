// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from gwpspider_interfaces:srv/InitBno.idl
// generated code does not contain a copyright notice

#ifndef GWPSPIDER_INTERFACES__SRV__DETAIL__INIT_BNO__STRUCT_H_
#define GWPSPIDER_INTERFACES__SRV__DETAIL__INIT_BNO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/InitBno in the package gwpspider_interfaces.
typedef struct gwpspider_interfaces__srv__InitBno_Request
{
  rosidl_runtime_c__String command;
} gwpspider_interfaces__srv__InitBno_Request;

// Struct for a sequence of gwpspider_interfaces__srv__InitBno_Request.
typedef struct gwpspider_interfaces__srv__InitBno_Request__Sequence
{
  gwpspider_interfaces__srv__InitBno_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gwpspider_interfaces__srv__InitBno_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/InitBno in the package gwpspider_interfaces.
typedef struct gwpspider_interfaces__srv__InitBno_Response
{
  bool success;
} gwpspider_interfaces__srv__InitBno_Response;

// Struct for a sequence of gwpspider_interfaces__srv__InitBno_Response.
typedef struct gwpspider_interfaces__srv__InitBno_Response__Sequence
{
  gwpspider_interfaces__srv__InitBno_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gwpspider_interfaces__srv__InitBno_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GWPSPIDER_INTERFACES__SRV__DETAIL__INIT_BNO__STRUCT_H_

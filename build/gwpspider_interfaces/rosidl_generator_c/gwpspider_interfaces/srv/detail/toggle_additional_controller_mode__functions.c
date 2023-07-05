// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from gwpspider_interfaces:srv/ToggleAdditionalControllerMode.idl
// generated code does not contain a copyright notice
#include "gwpspider_interfaces/srv/detail/toggle_additional_controller_mode__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `mode`
// Member `command`
#include "rosidl_runtime_c/string_functions.h"

bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__init(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__init(&msg->mode)) {
    gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__fini(msg);
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__init(&msg->command)) {
    gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__fini(msg);
    return false;
  }
  return true;
}

void
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__fini(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request * msg)
{
  if (!msg) {
    return;
  }
  // mode
  rosidl_runtime_c__String__fini(&msg->mode);
  // command
  rosidl_runtime_c__String__fini(&msg->command);
}

bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__are_equal(const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request * lhs, const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mode), &(rhs->mode)))
  {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  return true;
}

bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__copy(
  const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request * input,
  gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__copy(
      &(input->mode), &(output->mode)))
  {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  return true;
}

gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request *
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request * msg = (gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request *)allocator.allocate(sizeof(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request));
  bool success = gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__destroy(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence__init(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request * data = NULL;

  if (size) {
    data = (gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request *)allocator.zero_allocate(size, sizeof(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence__fini(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence *
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence * array = (gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence *)allocator.allocate(sizeof(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence__destroy(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence__are_equal(const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence * lhs, const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence__copy(
  const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence * input,
  gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request * data =
      (gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__init(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__fini(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__are_equal(const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response * lhs, const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__copy(
  const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response * input,
  gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response *
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response * msg = (gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response *)allocator.allocate(sizeof(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response));
  bool success = gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__destroy(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence__init(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response * data = NULL;

  if (size) {
    data = (gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response *)allocator.zero_allocate(size, sizeof(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence__fini(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence *
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence * array = (gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence *)allocator.allocate(sizeof(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence__destroy(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence__are_equal(const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence * lhs, const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence__copy(
  const gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence * input,
  gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response * data =
      (gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!gwpspider_interfaces__srv__ToggleAdditionalControllerMode_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

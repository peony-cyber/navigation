// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vel_forwarder:msg/GimbalAngles.idl
// generated code does not contain a copyright notice
#include "vel_forwarder/msg/detail/gimbal_angles__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
vel_forwarder__msg__GimbalAngles__init(vel_forwarder__msg__GimbalAngles * msg)
{
  if (!msg) {
    return false;
  }
  // yaw
  // pitch
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    vel_forwarder__msg__GimbalAngles__fini(msg);
    return false;
  }
  return true;
}

void
vel_forwarder__msg__GimbalAngles__fini(vel_forwarder__msg__GimbalAngles * msg)
{
  if (!msg) {
    return;
  }
  // yaw
  // pitch
  // header
  std_msgs__msg__Header__fini(&msg->header);
}

bool
vel_forwarder__msg__GimbalAngles__are_equal(const vel_forwarder__msg__GimbalAngles * lhs, const vel_forwarder__msg__GimbalAngles * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  return true;
}

bool
vel_forwarder__msg__GimbalAngles__copy(
  const vel_forwarder__msg__GimbalAngles * input,
  vel_forwarder__msg__GimbalAngles * output)
{
  if (!input || !output) {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  // pitch
  output->pitch = input->pitch;
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  return true;
}

vel_forwarder__msg__GimbalAngles *
vel_forwarder__msg__GimbalAngles__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__GimbalAngles * msg = (vel_forwarder__msg__GimbalAngles *)allocator.allocate(sizeof(vel_forwarder__msg__GimbalAngles), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vel_forwarder__msg__GimbalAngles));
  bool success = vel_forwarder__msg__GimbalAngles__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vel_forwarder__msg__GimbalAngles__destroy(vel_forwarder__msg__GimbalAngles * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vel_forwarder__msg__GimbalAngles__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vel_forwarder__msg__GimbalAngles__Sequence__init(vel_forwarder__msg__GimbalAngles__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__GimbalAngles * data = NULL;

  if (size) {
    data = (vel_forwarder__msg__GimbalAngles *)allocator.zero_allocate(size, sizeof(vel_forwarder__msg__GimbalAngles), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vel_forwarder__msg__GimbalAngles__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vel_forwarder__msg__GimbalAngles__fini(&data[i - 1]);
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
vel_forwarder__msg__GimbalAngles__Sequence__fini(vel_forwarder__msg__GimbalAngles__Sequence * array)
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
      vel_forwarder__msg__GimbalAngles__fini(&array->data[i]);
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

vel_forwarder__msg__GimbalAngles__Sequence *
vel_forwarder__msg__GimbalAngles__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__GimbalAngles__Sequence * array = (vel_forwarder__msg__GimbalAngles__Sequence *)allocator.allocate(sizeof(vel_forwarder__msg__GimbalAngles__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vel_forwarder__msg__GimbalAngles__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vel_forwarder__msg__GimbalAngles__Sequence__destroy(vel_forwarder__msg__GimbalAngles__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vel_forwarder__msg__GimbalAngles__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vel_forwarder__msg__GimbalAngles__Sequence__are_equal(const vel_forwarder__msg__GimbalAngles__Sequence * lhs, const vel_forwarder__msg__GimbalAngles__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vel_forwarder__msg__GimbalAngles__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vel_forwarder__msg__GimbalAngles__Sequence__copy(
  const vel_forwarder__msg__GimbalAngles__Sequence * input,
  vel_forwarder__msg__GimbalAngles__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vel_forwarder__msg__GimbalAngles);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vel_forwarder__msg__GimbalAngles * data =
      (vel_forwarder__msg__GimbalAngles *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vel_forwarder__msg__GimbalAngles__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vel_forwarder__msg__GimbalAngles__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vel_forwarder__msg__GimbalAngles__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

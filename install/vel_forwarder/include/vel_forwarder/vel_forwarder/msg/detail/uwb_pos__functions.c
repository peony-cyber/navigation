// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vel_forwarder:msg/UWBPos.idl
// generated code does not contain a copyright notice
#include "vel_forwarder/msg/detail/uwb_pos__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vel_forwarder__msg__UWBPos__init(vel_forwarder__msg__UWBPos * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
vel_forwarder__msg__UWBPos__fini(vel_forwarder__msg__UWBPos * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

bool
vel_forwarder__msg__UWBPos__are_equal(const vel_forwarder__msg__UWBPos * lhs, const vel_forwarder__msg__UWBPos * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
vel_forwarder__msg__UWBPos__copy(
  const vel_forwarder__msg__UWBPos * input,
  vel_forwarder__msg__UWBPos * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

vel_forwarder__msg__UWBPos *
vel_forwarder__msg__UWBPos__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__UWBPos * msg = (vel_forwarder__msg__UWBPos *)allocator.allocate(sizeof(vel_forwarder__msg__UWBPos), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vel_forwarder__msg__UWBPos));
  bool success = vel_forwarder__msg__UWBPos__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vel_forwarder__msg__UWBPos__destroy(vel_forwarder__msg__UWBPos * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vel_forwarder__msg__UWBPos__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vel_forwarder__msg__UWBPos__Sequence__init(vel_forwarder__msg__UWBPos__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__UWBPos * data = NULL;

  if (size) {
    data = (vel_forwarder__msg__UWBPos *)allocator.zero_allocate(size, sizeof(vel_forwarder__msg__UWBPos), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vel_forwarder__msg__UWBPos__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vel_forwarder__msg__UWBPos__fini(&data[i - 1]);
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
vel_forwarder__msg__UWBPos__Sequence__fini(vel_forwarder__msg__UWBPos__Sequence * array)
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
      vel_forwarder__msg__UWBPos__fini(&array->data[i]);
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

vel_forwarder__msg__UWBPos__Sequence *
vel_forwarder__msg__UWBPos__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__UWBPos__Sequence * array = (vel_forwarder__msg__UWBPos__Sequence *)allocator.allocate(sizeof(vel_forwarder__msg__UWBPos__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vel_forwarder__msg__UWBPos__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vel_forwarder__msg__UWBPos__Sequence__destroy(vel_forwarder__msg__UWBPos__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vel_forwarder__msg__UWBPos__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vel_forwarder__msg__UWBPos__Sequence__are_equal(const vel_forwarder__msg__UWBPos__Sequence * lhs, const vel_forwarder__msg__UWBPos__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vel_forwarder__msg__UWBPos__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vel_forwarder__msg__UWBPos__Sequence__copy(
  const vel_forwarder__msg__UWBPos__Sequence * input,
  vel_forwarder__msg__UWBPos__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vel_forwarder__msg__UWBPos);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vel_forwarder__msg__UWBPos * data =
      (vel_forwarder__msg__UWBPos *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vel_forwarder__msg__UWBPos__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vel_forwarder__msg__UWBPos__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vel_forwarder__msg__UWBPos__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

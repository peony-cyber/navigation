// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vel_forwarder:msg/Health.idl
// generated code does not contain a copyright notice
#include "vel_forwarder/msg/detail/health__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vel_forwarder__msg__Health__init(vel_forwarder__msg__Health * msg)
{
  if (!msg) {
    return false;
  }
  // hero
  // engineer
  // infantry1
  // infantry2
  // reserve
  // sentry
  return true;
}

void
vel_forwarder__msg__Health__fini(vel_forwarder__msg__Health * msg)
{
  if (!msg) {
    return;
  }
  // hero
  // engineer
  // infantry1
  // infantry2
  // reserve
  // sentry
}

bool
vel_forwarder__msg__Health__are_equal(const vel_forwarder__msg__Health * lhs, const vel_forwarder__msg__Health * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // hero
  if (lhs->hero != rhs->hero) {
    return false;
  }
  // engineer
  if (lhs->engineer != rhs->engineer) {
    return false;
  }
  // infantry1
  if (lhs->infantry1 != rhs->infantry1) {
    return false;
  }
  // infantry2
  if (lhs->infantry2 != rhs->infantry2) {
    return false;
  }
  // reserve
  if (lhs->reserve != rhs->reserve) {
    return false;
  }
  // sentry
  if (lhs->sentry != rhs->sentry) {
    return false;
  }
  return true;
}

bool
vel_forwarder__msg__Health__copy(
  const vel_forwarder__msg__Health * input,
  vel_forwarder__msg__Health * output)
{
  if (!input || !output) {
    return false;
  }
  // hero
  output->hero = input->hero;
  // engineer
  output->engineer = input->engineer;
  // infantry1
  output->infantry1 = input->infantry1;
  // infantry2
  output->infantry2 = input->infantry2;
  // reserve
  output->reserve = input->reserve;
  // sentry
  output->sentry = input->sentry;
  return true;
}

vel_forwarder__msg__Health *
vel_forwarder__msg__Health__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__Health * msg = (vel_forwarder__msg__Health *)allocator.allocate(sizeof(vel_forwarder__msg__Health), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vel_forwarder__msg__Health));
  bool success = vel_forwarder__msg__Health__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vel_forwarder__msg__Health__destroy(vel_forwarder__msg__Health * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vel_forwarder__msg__Health__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vel_forwarder__msg__Health__Sequence__init(vel_forwarder__msg__Health__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__Health * data = NULL;

  if (size) {
    data = (vel_forwarder__msg__Health *)allocator.zero_allocate(size, sizeof(vel_forwarder__msg__Health), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vel_forwarder__msg__Health__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vel_forwarder__msg__Health__fini(&data[i - 1]);
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
vel_forwarder__msg__Health__Sequence__fini(vel_forwarder__msg__Health__Sequence * array)
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
      vel_forwarder__msg__Health__fini(&array->data[i]);
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

vel_forwarder__msg__Health__Sequence *
vel_forwarder__msg__Health__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__Health__Sequence * array = (vel_forwarder__msg__Health__Sequence *)allocator.allocate(sizeof(vel_forwarder__msg__Health__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vel_forwarder__msg__Health__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vel_forwarder__msg__Health__Sequence__destroy(vel_forwarder__msg__Health__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vel_forwarder__msg__Health__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vel_forwarder__msg__Health__Sequence__are_equal(const vel_forwarder__msg__Health__Sequence * lhs, const vel_forwarder__msg__Health__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vel_forwarder__msg__Health__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vel_forwarder__msg__Health__Sequence__copy(
  const vel_forwarder__msg__Health__Sequence * input,
  vel_forwarder__msg__Health__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vel_forwarder__msg__Health);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vel_forwarder__msg__Health * data =
      (vel_forwarder__msg__Health *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vel_forwarder__msg__Health__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vel_forwarder__msg__Health__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vel_forwarder__msg__Health__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vel_forwarder:msg/GameData.idl
// generated code does not contain a copyright notice
#include "vel_forwarder/msg/detail/game_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vel_forwarder__msg__GameData__init(vel_forwarder__msg__GameData * msg)
{
  if (!msg) {
    return false;
  }
  // gamecode
  // ammoleft
  // timeleft
  // selfhealth
  // exteventdata
  return true;
}

void
vel_forwarder__msg__GameData__fini(vel_forwarder__msg__GameData * msg)
{
  if (!msg) {
    return;
  }
  // gamecode
  // ammoleft
  // timeleft
  // selfhealth
  // exteventdata
}

bool
vel_forwarder__msg__GameData__are_equal(const vel_forwarder__msg__GameData * lhs, const vel_forwarder__msg__GameData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // gamecode
  if (lhs->gamecode != rhs->gamecode) {
    return false;
  }
  // ammoleft
  if (lhs->ammoleft != rhs->ammoleft) {
    return false;
  }
  // timeleft
  if (lhs->timeleft != rhs->timeleft) {
    return false;
  }
  // selfhealth
  if (lhs->selfhealth != rhs->selfhealth) {
    return false;
  }
  // exteventdata
  if (lhs->exteventdata != rhs->exteventdata) {
    return false;
  }
  return true;
}

bool
vel_forwarder__msg__GameData__copy(
  const vel_forwarder__msg__GameData * input,
  vel_forwarder__msg__GameData * output)
{
  if (!input || !output) {
    return false;
  }
  // gamecode
  output->gamecode = input->gamecode;
  // ammoleft
  output->ammoleft = input->ammoleft;
  // timeleft
  output->timeleft = input->timeleft;
  // selfhealth
  output->selfhealth = input->selfhealth;
  // exteventdata
  output->exteventdata = input->exteventdata;
  return true;
}

vel_forwarder__msg__GameData *
vel_forwarder__msg__GameData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__GameData * msg = (vel_forwarder__msg__GameData *)allocator.allocate(sizeof(vel_forwarder__msg__GameData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vel_forwarder__msg__GameData));
  bool success = vel_forwarder__msg__GameData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vel_forwarder__msg__GameData__destroy(vel_forwarder__msg__GameData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vel_forwarder__msg__GameData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vel_forwarder__msg__GameData__Sequence__init(vel_forwarder__msg__GameData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__GameData * data = NULL;

  if (size) {
    data = (vel_forwarder__msg__GameData *)allocator.zero_allocate(size, sizeof(vel_forwarder__msg__GameData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vel_forwarder__msg__GameData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vel_forwarder__msg__GameData__fini(&data[i - 1]);
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
vel_forwarder__msg__GameData__Sequence__fini(vel_forwarder__msg__GameData__Sequence * array)
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
      vel_forwarder__msg__GameData__fini(&array->data[i]);
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

vel_forwarder__msg__GameData__Sequence *
vel_forwarder__msg__GameData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vel_forwarder__msg__GameData__Sequence * array = (vel_forwarder__msg__GameData__Sequence *)allocator.allocate(sizeof(vel_forwarder__msg__GameData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vel_forwarder__msg__GameData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vel_forwarder__msg__GameData__Sequence__destroy(vel_forwarder__msg__GameData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vel_forwarder__msg__GameData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vel_forwarder__msg__GameData__Sequence__are_equal(const vel_forwarder__msg__GameData__Sequence * lhs, const vel_forwarder__msg__GameData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vel_forwarder__msg__GameData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vel_forwarder__msg__GameData__Sequence__copy(
  const vel_forwarder__msg__GameData__Sequence * input,
  vel_forwarder__msg__GameData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vel_forwarder__msg__GameData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vel_forwarder__msg__GameData * data =
      (vel_forwarder__msg__GameData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vel_forwarder__msg__GameData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vel_forwarder__msg__GameData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vel_forwarder__msg__GameData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

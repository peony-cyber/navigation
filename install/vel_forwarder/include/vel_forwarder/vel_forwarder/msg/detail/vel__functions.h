// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from vel_forwarder:msg/Vel.idl
// generated code does not contain a copyright notice

#ifndef VEL_FORWARDER__MSG__DETAIL__VEL__FUNCTIONS_H_
#define VEL_FORWARDER__MSG__DETAIL__VEL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "vel_forwarder/msg/rosidl_generator_c__visibility_control.h"

#include "vel_forwarder/msg/detail/vel__struct.h"

/// Initialize msg/Vel message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * vel_forwarder__msg__Vel
 * )) before or use
 * vel_forwarder__msg__Vel__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
bool
vel_forwarder__msg__Vel__init(vel_forwarder__msg__Vel * msg);

/// Finalize msg/Vel message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
void
vel_forwarder__msg__Vel__fini(vel_forwarder__msg__Vel * msg);

/// Create msg/Vel message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * vel_forwarder__msg__Vel__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
vel_forwarder__msg__Vel *
vel_forwarder__msg__Vel__create();

/// Destroy msg/Vel message.
/**
 * It calls
 * vel_forwarder__msg__Vel__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
void
vel_forwarder__msg__Vel__destroy(vel_forwarder__msg__Vel * msg);

/// Check for msg/Vel message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
bool
vel_forwarder__msg__Vel__are_equal(const vel_forwarder__msg__Vel * lhs, const vel_forwarder__msg__Vel * rhs);

/// Copy a msg/Vel message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
bool
vel_forwarder__msg__Vel__copy(
  const vel_forwarder__msg__Vel * input,
  vel_forwarder__msg__Vel * output);

/// Initialize array of msg/Vel messages.
/**
 * It allocates the memory for the number of elements and calls
 * vel_forwarder__msg__Vel__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
bool
vel_forwarder__msg__Vel__Sequence__init(vel_forwarder__msg__Vel__Sequence * array, size_t size);

/// Finalize array of msg/Vel messages.
/**
 * It calls
 * vel_forwarder__msg__Vel__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
void
vel_forwarder__msg__Vel__Sequence__fini(vel_forwarder__msg__Vel__Sequence * array);

/// Create array of msg/Vel messages.
/**
 * It allocates the memory for the array and calls
 * vel_forwarder__msg__Vel__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
vel_forwarder__msg__Vel__Sequence *
vel_forwarder__msg__Vel__Sequence__create(size_t size);

/// Destroy array of msg/Vel messages.
/**
 * It calls
 * vel_forwarder__msg__Vel__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
void
vel_forwarder__msg__Vel__Sequence__destroy(vel_forwarder__msg__Vel__Sequence * array);

/// Check for msg/Vel message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
bool
vel_forwarder__msg__Vel__Sequence__are_equal(const vel_forwarder__msg__Vel__Sequence * lhs, const vel_forwarder__msg__Vel__Sequence * rhs);

/// Copy an array of msg/Vel messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_vel_forwarder
bool
vel_forwarder__msg__Vel__Sequence__copy(
  const vel_forwarder__msg__Vel__Sequence * input,
  vel_forwarder__msg__Vel__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VEL_FORWARDER__MSG__DETAIL__VEL__FUNCTIONS_H_

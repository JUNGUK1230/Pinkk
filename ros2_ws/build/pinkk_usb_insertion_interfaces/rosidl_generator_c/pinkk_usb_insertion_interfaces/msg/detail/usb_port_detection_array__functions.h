// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_detection_array.h"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__FUNCTIONS_H_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "pinkk_usb_insertion_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection_array__struct.h"

/// Initialize msg/UsbPortDetectionArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray
 * )) before or use
 * pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * msg);

/// Finalize msg/UsbPortDetectionArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
void
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * msg);

/// Create msg/UsbPortDetectionArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__create(void);

/// Destroy msg/UsbPortDetectionArray message.
/**
 * It calls
 * pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
void
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__destroy(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * msg);

/// Check for msg/UsbPortDetectionArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__are_equal(const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * lhs, const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * rhs);

/// Copy a msg/UsbPortDetectionArray message.
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
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__copy(
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * input,
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
const rosidl_type_hash_t *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
const rosidl_runtime_c__type_description__TypeSource *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/UsbPortDetectionArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__init(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * array, size_t size);

/// Finalize array of msg/UsbPortDetectionArray messages.
/**
 * It calls
 * pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
void
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__fini(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * array);

/// Create array of msg/UsbPortDetectionArray messages.
/**
 * It allocates the memory for the array and calls
 * pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__create(size_t size);

/// Destroy array of msg/UsbPortDetectionArray messages.
/**
 * It calls
 * pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
void
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__destroy(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * array);

/// Check for msg/UsbPortDetectionArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__are_equal(const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * lhs, const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * rhs);

/// Copy an array of msg/UsbPortDetectionArray messages.
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
ROSIDL_GENERATOR_C_PUBLIC_pinkk_usb_insertion_interfaces
bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__copy(
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * input,
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__FUNCTIONS_H_

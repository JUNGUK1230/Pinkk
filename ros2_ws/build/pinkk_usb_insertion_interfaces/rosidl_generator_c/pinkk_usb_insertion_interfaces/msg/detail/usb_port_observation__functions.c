// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortObservation.idl
// generated code does not contain a copyright notice
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_observation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `detection_id`
// Member `rejection_reason`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `keypoints`
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__functions.h"

bool
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__init(pinkk_usb_insertion_interfaces__msg__UsbPortObservation * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(msg);
    return false;
  }
  // detection_id
  if (!rosidl_runtime_c__String__init(&msg->detection_id)) {
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(msg);
    return false;
  }
  // keypoints
  for (size_t i = 0; i < 4; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__Keypoint2D__init(&msg->keypoints[i])) {
      pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(msg);
      return false;
    }
  }
  // object_confidence
  // reprojection_error_px
  // depth_m
  // valid
  // rejection_reason
  if (!rosidl_runtime_c__String__init(&msg->rejection_reason)) {
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(msg);
    return false;
  }
  return true;
}

void
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(pinkk_usb_insertion_interfaces__msg__UsbPortObservation * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detection_id
  rosidl_runtime_c__String__fini(&msg->detection_id);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // keypoints
  for (size_t i = 0; i < 4; ++i) {
    pinkk_usb_insertion_interfaces__msg__Keypoint2D__fini(&msg->keypoints[i]);
  }
  // object_confidence
  // reprojection_error_px
  // depth_m
  // valid
  // rejection_reason
  rosidl_runtime_c__String__fini(&msg->rejection_reason);
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__are_equal(const pinkk_usb_insertion_interfaces__msg__UsbPortObservation * lhs, const pinkk_usb_insertion_interfaces__msg__UsbPortObservation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // detection_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->detection_id), &(rhs->detection_id)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // keypoints
  for (size_t i = 0; i < 4; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__Keypoint2D__are_equal(
        &(lhs->keypoints[i]), &(rhs->keypoints[i])))
    {
      return false;
    }
  }
  // object_confidence
  if (lhs->object_confidence != rhs->object_confidence) {
    return false;
  }
  // reprojection_error_px
  if (lhs->reprojection_error_px != rhs->reprojection_error_px) {
    return false;
  }
  // depth_m
  if (lhs->depth_m != rhs->depth_m) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  // rejection_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->rejection_reason), &(rhs->rejection_reason)))
  {
    return false;
  }
  return true;
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__copy(
  const pinkk_usb_insertion_interfaces__msg__UsbPortObservation * input,
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // detection_id
  if (!rosidl_runtime_c__String__copy(
      &(input->detection_id), &(output->detection_id)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // keypoints
  for (size_t i = 0; i < 4; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__Keypoint2D__copy(
        &(input->keypoints[i]), &(output->keypoints[i])))
    {
      return false;
    }
  }
  // object_confidence
  output->object_confidence = input->object_confidence;
  // reprojection_error_px
  output->reprojection_error_px = input->reprojection_error_px;
  // depth_m
  output->depth_m = input->depth_m;
  // valid
  output->valid = input->valid;
  // rejection_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->rejection_reason), &(output->rejection_reason)))
  {
    return false;
  }
  return true;
}

pinkk_usb_insertion_interfaces__msg__UsbPortObservation *
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation * msg = (pinkk_usb_insertion_interfaces__msg__UsbPortObservation *)allocator.allocate(sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation));
  bool success = pinkk_usb_insertion_interfaces__msg__UsbPortObservation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__destroy(pinkk_usb_insertion_interfaces__msg__UsbPortObservation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__init(pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation)) {
      return false;
    }
    data = (pinkk_usb_insertion_interfaces__msg__UsbPortObservation *)allocator.zero_allocate(size, sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pinkk_usb_insertion_interfaces__msg__UsbPortObservation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(&data[i - 1]);
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
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__fini(pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence * array)
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
      pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(&array->data[i]);
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

pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence *
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence * array = (pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence *)allocator.allocate(sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__destroy(pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__are_equal(const pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence * lhs, const pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__UsbPortObservation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence__copy(
  const pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence * input,
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation * data =
      (pinkk_usb_insertion_interfaces__msg__UsbPortObservation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pinkk_usb_insertion_interfaces__msg__UsbPortObservation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__UsbPortObservation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

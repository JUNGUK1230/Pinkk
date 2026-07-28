// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
// generated code does not contain a copyright notice
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `detection_id`
// Member `class_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `bbox`
#include "vision_msgs/msg/detail/bounding_box2_d__functions.h"
// Member `keypoints`
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__functions.h"

bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__init(pinkk_usb_insertion_interfaces__msg__UsbPortDetection * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(msg);
    return false;
  }
  // detection_id
  if (!rosidl_runtime_c__String__init(&msg->detection_id)) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(msg);
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__init(&msg->class_name)) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(msg);
    return false;
  }
  // object_confidence
  // bbox
  if (!vision_msgs__msg__BoundingBox2D__init(&msg->bbox)) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(msg);
    return false;
  }
  // source_image_width
  // source_image_height
  // keypoints
  for (size_t i = 0; i < 4; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__Keypoint2D__init(&msg->keypoints[i])) {
      pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(msg);
      return false;
    }
  }
  return true;
}

void
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(pinkk_usb_insertion_interfaces__msg__UsbPortDetection * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detection_id
  rosidl_runtime_c__String__fini(&msg->detection_id);
  // class_name
  rosidl_runtime_c__String__fini(&msg->class_name);
  // object_confidence
  // bbox
  vision_msgs__msg__BoundingBox2D__fini(&msg->bbox);
  // source_image_width
  // source_image_height
  // keypoints
  for (size_t i = 0; i < 4; ++i) {
    pinkk_usb_insertion_interfaces__msg__Keypoint2D__fini(&msg->keypoints[i]);
  }
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__are_equal(const pinkk_usb_insertion_interfaces__msg__UsbPortDetection * lhs, const pinkk_usb_insertion_interfaces__msg__UsbPortDetection * rhs)
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
  // class_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->class_name), &(rhs->class_name)))
  {
    return false;
  }
  // object_confidence
  if (lhs->object_confidence != rhs->object_confidence) {
    return false;
  }
  // bbox
  if (!vision_msgs__msg__BoundingBox2D__are_equal(
      &(lhs->bbox), &(rhs->bbox)))
  {
    return false;
  }
  // source_image_width
  if (lhs->source_image_width != rhs->source_image_width) {
    return false;
  }
  // source_image_height
  if (lhs->source_image_height != rhs->source_image_height) {
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
  return true;
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__copy(
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection * input,
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * output)
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
  // class_name
  if (!rosidl_runtime_c__String__copy(
      &(input->class_name), &(output->class_name)))
  {
    return false;
  }
  // object_confidence
  output->object_confidence = input->object_confidence;
  // bbox
  if (!vision_msgs__msg__BoundingBox2D__copy(
      &(input->bbox), &(output->bbox)))
  {
    return false;
  }
  // source_image_width
  output->source_image_width = input->source_image_width;
  // source_image_height
  output->source_image_height = input->source_image_height;
  // keypoints
  for (size_t i = 0; i < 4; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__Keypoint2D__copy(
        &(input->keypoints[i]), &(output->keypoints[i])))
    {
      return false;
    }
  }
  return true;
}

pinkk_usb_insertion_interfaces__msg__UsbPortDetection *
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * msg = (pinkk_usb_insertion_interfaces__msg__UsbPortDetection *)allocator.allocate(sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection));
  bool success = pinkk_usb_insertion_interfaces__msg__UsbPortDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__destroy(pinkk_usb_insertion_interfaces__msg__UsbPortDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__init(pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection)) {
      return false;
    }
    data = (pinkk_usb_insertion_interfaces__msg__UsbPortDetection *)allocator.zero_allocate(size, sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pinkk_usb_insertion_interfaces__msg__UsbPortDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(&data[i - 1]);
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
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__fini(pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * array)
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
      pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(&array->data[i]);
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

pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence *
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * array = (pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence *)allocator.allocate(sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__destroy(pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__are_equal(const pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * lhs, const pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__copy(
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * input,
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pinkk_usb_insertion_interfaces__msg__UsbPortDetection * data =
      (pinkk_usb_insertion_interfaces__msg__UsbPortDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pinkk_usb_insertion_interfaces__msg__UsbPortDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

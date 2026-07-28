// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetectionArray.idl
// generated code does not contain a copyright notice
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `detections`
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__functions.h"

bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini(msg);
    return false;
  }
  // detections
  if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__init(&msg->detections, 0)) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini(msg);
    return false;
  }
  return true;
}

void
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detections
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__fini(&msg->detections);
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__are_equal(const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * lhs, const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * rhs)
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
  // detections
  if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__are_equal(
      &(lhs->detections), &(rhs->detections)))
  {
    return false;
  }
  return true;
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__copy(
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * input,
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * output)
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
  // detections
  if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence__copy(
      &(input->detections), &(output->detections)))
  {
    return false;
  }
  return true;
}

pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * msg = (pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray *)allocator.allocate(sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray));
  bool success = pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__destroy(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__init(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray)) {
      return false;
    }
    data = (pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray *)allocator.zero_allocate(size, sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini(&data[i - 1]);
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
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__fini(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * array)
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
      pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini(&array->data[i]);
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

pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence *
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * array = (pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence *)allocator.allocate(sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__destroy(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__are_equal(const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * lhs, const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence__copy(
  const pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * input,
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * data =
      (pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

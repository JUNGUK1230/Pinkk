// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pinkk_usb_insertion_interfaces:msg/Keypoint2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/keypoint2_d.h"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__STRUCT_H_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Keypoint2D in the package pinkk_usb_insertion_interfaces.
/**
  * YOLO가 검출한 하나의 2D keypoint.
 */
typedef struct pinkk_usb_insertion_interfaces__msg__Keypoint2D
{
  uint8_t index;
  double x;
  double y;
  float confidence;
  bool visible;
} pinkk_usb_insertion_interfaces__msg__Keypoint2D;

// Struct for a sequence of pinkk_usb_insertion_interfaces__msg__Keypoint2D.
typedef struct pinkk_usb_insertion_interfaces__msg__Keypoint2D__Sequence
{
  pinkk_usb_insertion_interfaces__msg__Keypoint2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__msg__Keypoint2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__KEYPOINT2_D__STRUCT_H_

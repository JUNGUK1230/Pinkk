// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_detection_array.h"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__STRUCT_H_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'detections'
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_detection__struct.h"

/// Struct defined in msg/UsbPortDetectionArray in the package pinkk_usb_insertion_interfaces.
/**
  * 같은 영상 프레임에서 검출된 USB-A 포트 후보 목록.
 */
typedef struct pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray
{
  std_msgs__msg__Header header;
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence detections;
} pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray;

// Struct for a sequence of pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray.
typedef struct pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence
{
  pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__msg__UsbPortDetectionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION_ARRAY__STRUCT_H_

// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_detection.h"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__STRUCT_H_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__STRUCT_H_

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
// Member 'detection_id'
// Member 'class_name'
#include "rosidl_runtime_c/string.h"
// Member 'bbox'
#include "vision_msgs/msg/detail/bounding_box2_d__struct.h"
// Member 'keypoints'
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__struct.h"

/// Struct defined in msg/UsbPortDetection in the package pinkk_usb_insertion_interfaces.
/**
  * 한 USB-A 포트 후보에 대한 YOLO keypoint 결과.
 */
typedef struct pinkk_usb_insertion_interfaces__msg__UsbPortDetection
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String detection_id;
  rosidl_runtime_c__String class_name;
  float object_confidence;
  vision_msgs__msg__BoundingBox2D bbox;
  uint32_t source_image_width;
  uint32_t source_image_height;
  pinkk_usb_insertion_interfaces__msg__Keypoint2D keypoints[4];
} pinkk_usb_insertion_interfaces__msg__UsbPortDetection;

// Struct for a sequence of pinkk_usb_insertion_interfaces__msg__UsbPortDetection.
typedef struct pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence
{
  pinkk_usb_insertion_interfaces__msg__UsbPortDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__msg__UsbPortDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_DETECTION__STRUCT_H_

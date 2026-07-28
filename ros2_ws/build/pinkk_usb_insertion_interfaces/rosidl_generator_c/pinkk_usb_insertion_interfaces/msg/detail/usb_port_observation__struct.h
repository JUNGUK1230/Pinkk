// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortObservation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/msg/usb_port_observation.h"


#ifndef PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__STRUCT_H_
#define PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__STRUCT_H_

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
// Member 'rejection_reason'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'keypoints'
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__struct.h"

/// Struct defined in msg/UsbPortObservation in the package pinkk_usb_insertion_interfaces.
/**
  * 검출 keypoint와 solvePnP 결과를 원자적으로 전달한다.
 */
typedef struct pinkk_usb_insertion_interfaces__msg__UsbPortObservation
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String detection_id;
  geometry_msgs__msg__Pose pose;
  pinkk_usb_insertion_interfaces__msg__Keypoint2D keypoints[4];
  float object_confidence;
  float reprojection_error_px;
  float depth_m;
  bool valid;
  rosidl_runtime_c__String rejection_reason;
} pinkk_usb_insertion_interfaces__msg__UsbPortObservation;

// Struct for a sequence of pinkk_usb_insertion_interfaces__msg__UsbPortObservation.
typedef struct pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence
{
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__msg__UsbPortObservation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PINKK_USB_INSERTION_INTERFACES__MSG__DETAIL__USB_PORT_OBSERVATION__STRUCT_H_

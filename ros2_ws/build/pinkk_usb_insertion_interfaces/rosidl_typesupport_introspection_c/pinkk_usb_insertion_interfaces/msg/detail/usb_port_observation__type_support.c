// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pinkk_usb_insertion_interfaces:msg/UsbPortObservation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_observation__rosidl_typesupport_introspection_c.h"
#include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_observation__functions.h"
#include "pinkk_usb_insertion_interfaces/msg/detail/usb_port_observation__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `detection_id`
// Member `rejection_reason`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `keypoints`
#include "pinkk_usb_insertion_interfaces/msg/keypoint2_d.h"
// Member `keypoints`
#include "pinkk_usb_insertion_interfaces/msg/detail/keypoint2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation__init(message_memory);
}

void pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation__fini(message_memory);
}

size_t pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__size_function__UsbPortObservation__keypoints(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__get_const_function__UsbPortObservation__keypoints(
  const void * untyped_member, size_t index)
{
  const pinkk_usb_insertion_interfaces__msg__Keypoint2D * member =
    (const pinkk_usb_insertion_interfaces__msg__Keypoint2D *)(untyped_member);
  return &member[index];
}

void * pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__get_function__UsbPortObservation__keypoints(
  void * untyped_member, size_t index)
{
  pinkk_usb_insertion_interfaces__msg__Keypoint2D * member =
    (pinkk_usb_insertion_interfaces__msg__Keypoint2D *)(untyped_member);
  return &member[index];
}

void pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__fetch_function__UsbPortObservation__keypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pinkk_usb_insertion_interfaces__msg__Keypoint2D * item =
    ((const pinkk_usb_insertion_interfaces__msg__Keypoint2D *)
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__get_const_function__UsbPortObservation__keypoints(untyped_member, index));
  pinkk_usb_insertion_interfaces__msg__Keypoint2D * value =
    (pinkk_usb_insertion_interfaces__msg__Keypoint2D *)(untyped_value);
  *value = *item;
}

void pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__assign_function__UsbPortObservation__keypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pinkk_usb_insertion_interfaces__msg__Keypoint2D * item =
    ((pinkk_usb_insertion_interfaces__msg__Keypoint2D *)
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__get_function__UsbPortObservation__keypoints(untyped_member, index));
  const pinkk_usb_insertion_interfaces__msg__Keypoint2D * value =
    (const pinkk_usb_insertion_interfaces__msg__Keypoint2D *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detection_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation, detection_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "keypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation, keypoints),  // bytes offset in struct
    NULL,  // default value
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__size_function__UsbPortObservation__keypoints,  // size() function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__get_const_function__UsbPortObservation__keypoints,  // get_const(index) function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__get_function__UsbPortObservation__keypoints,  // get(index) function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__fetch_function__UsbPortObservation__keypoints,  // fetch(index, &value) function pointer
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__assign_function__UsbPortObservation__keypoints,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation, object_confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reprojection_error_px",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation, reprojection_error_px),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "depth_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation, depth_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation, valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rejection_reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation, rejection_reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_members = {
  "pinkk_usb_insertion_interfaces__msg",  // message namespace
  "UsbPortObservation",  // message name
  9,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__msg__UsbPortObservation),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__msg__UsbPortObservation__get_type_hash,
  &pinkk_usb_insertion_interfaces__msg__UsbPortObservation__get_type_description,
  &pinkk_usb_insertion_interfaces__msg__UsbPortObservation__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, msg, UsbPortObservation)() {
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, msg, Keypoint2D)();
  if (!pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__msg__UsbPortObservation__rosidl_typesupport_introspection_c__UsbPortObservation_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pinkk_usb_insertion_interfaces:action/CartesianMove.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
#include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
#include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"


// Include directives for member types
// Member `target`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `target`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__init(message_memory);
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_message_member_array[5] = {
  {
    "target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Goal, target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Goal, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Goal, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lock_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Goal, lock_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lock_roll_pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Goal, lock_roll_pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_message_members = {
  "pinkk_usb_insertion_interfaces__action",  // message namespace
  "CartesianMove_Goal",  // message name
  5,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__action__CartesianMove_Goal),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_Goal)() {
  pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__rosidl_typesupport_introspection_c__CartesianMove_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"
// Member `actual`
// already included above
// #include "geometry_msgs/msg/pose_stamped.h"
// Member `actual`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__action__CartesianMove_Result__init(message_memory);
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_message_member_array[3] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Result, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Result, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actual",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Result, actual),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_message_members = {
  "pinkk_usb_insertion_interfaces__action",  // message namespace
  "CartesianMove_Result",  // message name
  3,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__action__CartesianMove_Result),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Result__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Result__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Result__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_Result)() {
  pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__action__CartesianMove_Result__rosidl_typesupport_introspection_c__CartesianMove_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"


// Include directives for member types
// Member `actual`
// already included above
// #include "geometry_msgs/msg/pose_stamped.h"
// Member `actual`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__init(message_memory);
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_message_member_array[3] = {
  {
    "actual",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback, actual),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_error_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback, position_error_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "orientation_error_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback, orientation_error_deg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_message_members = {
  "pinkk_usb_insertion_interfaces__action",  // message namespace
  "CartesianMove_Feedback",  // message name
  3,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_Feedback)() {
  pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__rosidl_typesupport_introspection_c__CartesianMove_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "pinkk_usb_insertion_interfaces/action/cartesian_move.h"
// Member `goal`
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__init(message_memory);
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_members = {
  "pinkk_usb_insertion_interfaces__action",  // message namespace
  "CartesianMove_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Request)() {
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_Goal)();
  if (!pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__init(message_memory);
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_members = {
  "pinkk_usb_insertion_interfaces__action",  // message namespace
  "CartesianMove_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Response)() {
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
// already included above
// #include "pinkk_usb_insertion_interfaces/action/cartesian_move.h"
// Member `request`
// Member `response`
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__init(message_memory);
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__fini(message_memory);
}

size_t pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__CartesianMove_SendGoal_Event__request(
  const void * untyped_member)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence * member =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_SendGoal_Event__request(
  const void * untyped_member, size_t index)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence * member =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_SendGoal_Event__request(
  void * untyped_member, size_t index)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence * member =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__CartesianMove_SendGoal_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request * item =
    ((const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request *)
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_SendGoal_Event__request(untyped_member, index));
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request * value =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request *)(untyped_value);
  *value = *item;
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__CartesianMove_SendGoal_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request * item =
    ((pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request *)
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_SendGoal_Event__request(untyped_member, index));
  const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request * value =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request *)(untyped_value);
  *item = *value;
}

bool pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__CartesianMove_SendGoal_Event__request(
  void * untyped_member, size_t size)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence * member =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence *)(untyped_member);
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence__fini(member);
  return pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence__init(member, size);
}

size_t pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__CartesianMove_SendGoal_Event__response(
  const void * untyped_member)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence * member =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_SendGoal_Event__response(
  const void * untyped_member, size_t index)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence * member =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_SendGoal_Event__response(
  void * untyped_member, size_t index)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence * member =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__CartesianMove_SendGoal_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response * item =
    ((const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response *)
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_SendGoal_Event__response(untyped_member, index));
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response * value =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response *)(untyped_value);
  *value = *item;
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__CartesianMove_SendGoal_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response * item =
    ((pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response *)
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_SendGoal_Event__response(untyped_member, index));
  const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response * value =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response *)(untyped_value);
  *item = *value;
}

bool pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__CartesianMove_SendGoal_Event__response(
  void * untyped_member, size_t size)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence * member =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence *)(untyped_member);
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence__fini(member);
  return pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event, request),  // bytes offset in struct
    NULL,  // default value
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__CartesianMove_SendGoal_Event__request,  // size() function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_SendGoal_Event__request,  // get_const(index) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_SendGoal_Event__request,  // get(index) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__CartesianMove_SendGoal_Event__request,  // fetch(index, &value) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__CartesianMove_SendGoal_Event__request,  // assign(index, value) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__CartesianMove_SendGoal_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event, response),  // bytes offset in struct
    NULL,  // default value
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__CartesianMove_SendGoal_Event__response,  // size() function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_SendGoal_Event__response,  // get_const(index) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_SendGoal_Event__response,  // get(index) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__CartesianMove_SendGoal_Event__response,  // fetch(index, &value) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__CartesianMove_SendGoal_Event__response,  // assign(index, value) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__CartesianMove_SendGoal_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_members = {
  "pinkk_usb_insertion_interfaces__action",  // message namespace
  "CartesianMove_SendGoal_Event",  // message name
  3,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Event)() {
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Request)();
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Response)();
  if (!pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_service_members = {
  "pinkk_usb_insertion_interfaces__action",  // service namespace
  "CartesianMove_SendGoal",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_type_support_handle,
  NULL,  // response message
  // pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_type_support_handle
  NULL  // event_message
  // pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_type_support_handle
};


static rosidl_service_type_support_t pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_service_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_service_members,
  get_service_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Request_message_type_support_handle,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Response_message_type_support_handle,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    pinkk_usb_insertion_interfaces,
    action,
    CartesianMove_SendGoal
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    pinkk_usb_insertion_interfaces,
    action,
    CartesianMove_SendGoal
  ),
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal)(void) {
  if (!pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_service_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Event)()->data;
  }

  return &pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__init(message_memory);
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_members = {
  "pinkk_usb_insertion_interfaces__action",  // message namespace
  "CartesianMove_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Request)() {
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "pinkk_usb_insertion_interfaces/action/cartesian_move.h"
// Member `result`
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__init(message_memory);
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_members = {
  "pinkk_usb_insertion_interfaces__action",  // message namespace
  "CartesianMove_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Response)() {
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_Result)();
  if (!pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/service_event_info.h"
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
// already included above
// #include "pinkk_usb_insertion_interfaces/action/cartesian_move.h"
// Member `request`
// Member `response`
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__init(message_memory);
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__fini(message_memory);
}

size_t pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__size_function__CartesianMove_GetResult_Event__request(
  const void * untyped_member)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence * member =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_GetResult_Event__request(
  const void * untyped_member, size_t index)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence * member =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_GetResult_Event__request(
  void * untyped_member, size_t index)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence * member =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__CartesianMove_GetResult_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request * item =
    ((const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request *)
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_GetResult_Event__request(untyped_member, index));
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request * value =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request *)(untyped_value);
  *value = *item;
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__CartesianMove_GetResult_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request * item =
    ((pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request *)
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_GetResult_Event__request(untyped_member, index));
  const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request * value =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request *)(untyped_value);
  *item = *value;
}

bool pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__CartesianMove_GetResult_Event__request(
  void * untyped_member, size_t size)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence * member =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence *)(untyped_member);
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence__fini(member);
  return pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence__init(member, size);
}

size_t pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__size_function__CartesianMove_GetResult_Event__response(
  const void * untyped_member)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence * member =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_GetResult_Event__response(
  const void * untyped_member, size_t index)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence * member =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_GetResult_Event__response(
  void * untyped_member, size_t index)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence * member =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__CartesianMove_GetResult_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response * item =
    ((const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response *)
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_GetResult_Event__response(untyped_member, index));
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response * value =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response *)(untyped_value);
  *value = *item;
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__CartesianMove_GetResult_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response * item =
    ((pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response *)
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_GetResult_Event__response(untyped_member, index));
  const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response * value =
    (const pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response *)(untyped_value);
  *item = *value;
}

bool pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__CartesianMove_GetResult_Event__response(
  void * untyped_member, size_t size)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence * member =
    (pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence *)(untyped_member);
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence__fini(member);
  return pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event, request),  // bytes offset in struct
    NULL,  // default value
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__size_function__CartesianMove_GetResult_Event__request,  // size() function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_GetResult_Event__request,  // get_const(index) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_GetResult_Event__request,  // get(index) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__CartesianMove_GetResult_Event__request,  // fetch(index, &value) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__CartesianMove_GetResult_Event__request,  // assign(index, value) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__CartesianMove_GetResult_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event, response),  // bytes offset in struct
    NULL,  // default value
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__size_function__CartesianMove_GetResult_Event__response,  // size() function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__CartesianMove_GetResult_Event__response,  // get_const(index) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__get_function__CartesianMove_GetResult_Event__response,  // get(index) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__CartesianMove_GetResult_Event__response,  // fetch(index, &value) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__CartesianMove_GetResult_Event__response,  // assign(index, value) function pointer
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__CartesianMove_GetResult_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_members = {
  "pinkk_usb_insertion_interfaces__action",  // message namespace
  "CartesianMove_GetResult_Event",  // message name
  3,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Event)() {
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Request)();
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Response)();
  if (!pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_GetResult_service_members = {
  "pinkk_usb_insertion_interfaces__action",  // service namespace
  "CartesianMove_GetResult",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_type_support_handle,
  NULL,  // response message
  // pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_type_support_handle
  NULL  // event_message
  // pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_type_support_handle
};


static rosidl_service_type_support_t pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_GetResult_service_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_GetResult_service_members,
  get_service_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Request_message_type_support_handle,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Response_message_type_support_handle,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__rosidl_typesupport_introspection_c__CartesianMove_GetResult_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    pinkk_usb_insertion_interfaces,
    action,
    CartesianMove_GetResult
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    pinkk_usb_insertion_interfaces,
    action,
    CartesianMove_GetResult
  ),
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult)(void) {
  if (!pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_GetResult_service_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Event)()->data;
  }

  return &pinkk_usb_insertion_interfaces__action__detail__cartesian_move__rosidl_typesupport_introspection_c__CartesianMove_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "pinkk_usb_insertion_interfaces/action/cartesian_move.h"
// Member `feedback`
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__init(message_memory);
}

void pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_fini_function(void * message_memory)
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_message_members = {
  "pinkk_usb_insertion_interfaces__action",  // message namespace
  "CartesianMove_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage),
  false,  // has_any_key_member_
  pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_message_member_array,  // message members
  pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_message_type_support_handle = {
  0,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinkk_usb_insertion_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_FeedbackMessage)() {
  pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinkk_usb_insertion_interfaces, action, CartesianMove_Feedback)();
  if (!pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__rosidl_typesupport_introspection_c__CartesianMove_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

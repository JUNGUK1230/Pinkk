// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pinkk_usb_insertion_interfaces:action/CartesianMove.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/action/cartesian_move.h"


#ifndef PINKK_USB_INSERTION_INTERFACES__ACTION__DETAIL__CARTESIAN_MOVE__STRUCT_H_
#define PINKK_USB_INSERTION_INTERFACES__ACTION__DETAIL__CARTESIAN_MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/CartesianMove in the package pinkk_usb_insertion_interfaces.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_Goal
{
  geometry_msgs__msg__PoseStamped target;
  int32_t speed;
  int32_t mode;
  bool lock_z;
  bool lock_roll_pitch;
} pinkk_usb_insertion_interfaces__action__CartesianMove_Goal;

// Struct for a sequence of pinkk_usb_insertion_interfaces__action__CartesianMove_Goal.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__Sequence
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"
// Member 'actual'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/CartesianMove in the package pinkk_usb_insertion_interfaces.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_Result
{
  bool success;
  rosidl_runtime_c__String message;
  geometry_msgs__msg__PoseStamped actual;
} pinkk_usb_insertion_interfaces__action__CartesianMove_Result;

// Struct for a sequence of pinkk_usb_insertion_interfaces__action__CartesianMove_Result.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_Result__Sequence
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__action__CartesianMove_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'actual'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/CartesianMove in the package pinkk_usb_insertion_interfaces.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback
{
  geometry_msgs__msg__PoseStamped actual;
  double position_error_m;
  double orientation_error_deg;
} pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback;

// Struct for a sequence of pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__Sequence
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"

/// Struct defined in action/CartesianMove in the package pinkk_usb_insertion_interfaces.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  pinkk_usb_insertion_interfaces__action__CartesianMove_Goal goal;
} pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request;

// Struct for a sequence of pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/CartesianMove in the package pinkk_usb_insertion_interfaces.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response;

// Struct for a sequence of pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/CartesianMove in the package pinkk_usb_insertion_interfaces.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__Sequence request;
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__Sequence response;
} pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event;

// Struct for a sequence of pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__Sequence
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/CartesianMove in the package pinkk_usb_insertion_interfaces.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request;

// Struct for a sequence of pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"

/// Struct defined in action/CartesianMove in the package pinkk_usb_insertion_interfaces.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response
{
  int8_t status;
  pinkk_usb_insertion_interfaces__action__CartesianMove_Result result;
} pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response;

// Struct for a sequence of pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/CartesianMove in the package pinkk_usb_insertion_interfaces.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__Sequence request;
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__Sequence response;
} pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event;

// Struct for a sequence of pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__Sequence
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.h"

/// Struct defined in action/CartesianMove in the package pinkk_usb_insertion_interfaces.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback feedback;
} pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage;

// Struct for a sequence of pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage.
typedef struct pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__Sequence
{
  pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PINKK_USB_INSERTION_INTERFACES__ACTION__DETAIL__CARTESIAN_MOVE__STRUCT_H_

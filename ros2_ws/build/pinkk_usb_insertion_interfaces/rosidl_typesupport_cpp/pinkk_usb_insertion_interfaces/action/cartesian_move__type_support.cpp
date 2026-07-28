// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from pinkk_usb_insertion_interfaces:action/CartesianMove.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
#include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_Goal_type_support_ids_t;

static const _CartesianMove_Goal_type_support_ids_t _CartesianMove_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_Goal_type_support_symbol_names_t _CartesianMove_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_Goal)),
  }
};

typedef struct _CartesianMove_Goal_type_support_data_t
{
  void * data[2];
} _CartesianMove_Goal_type_support_data_t;

static _CartesianMove_Goal_type_support_data_t _CartesianMove_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_Goal_message_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CartesianMove_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Goal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_Goal>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_Goal)() {
  return get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_Result_type_support_ids_t;

static const _CartesianMove_Result_type_support_ids_t _CartesianMove_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_Result_type_support_symbol_names_t _CartesianMove_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_Result)),
  }
};

typedef struct _CartesianMove_Result_type_support_data_t
{
  void * data[2];
} _CartesianMove_Result_type_support_data_t;

static _CartesianMove_Result_type_support_data_t _CartesianMove_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_Result_message_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_Result_message_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_Result_message_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CartesianMove_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Result__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Result__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Result__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_Result>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_Result)() {
  return get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_Feedback_type_support_ids_t;

static const _CartesianMove_Feedback_type_support_ids_t _CartesianMove_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_Feedback_type_support_symbol_names_t _CartesianMove_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_Feedback)),
  }
};

typedef struct _CartesianMove_Feedback_type_support_data_t
{
  void * data[2];
} _CartesianMove_Feedback_type_support_data_t;

static _CartesianMove_Feedback_type_support_data_t _CartesianMove_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_Feedback_message_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CartesianMove_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_Feedback)() {
  return get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_SendGoal_Request_type_support_ids_t;

static const _CartesianMove_SendGoal_Request_type_support_ids_t _CartesianMove_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_SendGoal_Request_type_support_symbol_names_t _CartesianMove_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Request)),
  }
};

typedef struct _CartesianMove_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _CartesianMove_SendGoal_Request_type_support_data_t;

static _CartesianMove_SendGoal_Request_type_support_data_t _CartesianMove_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_SendGoal_Request_message_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CartesianMove_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Request)() {
  return get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_SendGoal_Response_type_support_ids_t;

static const _CartesianMove_SendGoal_Response_type_support_ids_t _CartesianMove_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_SendGoal_Response_type_support_symbol_names_t _CartesianMove_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Response)),
  }
};

typedef struct _CartesianMove_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _CartesianMove_SendGoal_Response_type_support_data_t;

static _CartesianMove_SendGoal_Response_type_support_data_t _CartesianMove_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_SendGoal_Response_message_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CartesianMove_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Response)() {
  return get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_SendGoal_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_SendGoal_Event_type_support_ids_t;

static const _CartesianMove_SendGoal_Event_type_support_ids_t _CartesianMove_SendGoal_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_SendGoal_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_SendGoal_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_SendGoal_Event_type_support_symbol_names_t _CartesianMove_SendGoal_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Event)),
  }
};

typedef struct _CartesianMove_SendGoal_Event_type_support_data_t
{
  void * data[2];
} _CartesianMove_SendGoal_Event_type_support_data_t;

static _CartesianMove_SendGoal_Event_type_support_data_t _CartesianMove_SendGoal_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_SendGoal_Event_message_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_SendGoal_Event_message_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_SendGoal_Event_message_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_SendGoal_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CartesianMove_SendGoal_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_SendGoal_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_SendGoal_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal_Event)() {
  return get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_SendGoal_type_support_ids_t;

static const _CartesianMove_SendGoal_type_support_ids_t _CartesianMove_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_SendGoal_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_SendGoal_type_support_symbol_names_t _CartesianMove_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal)),
  }
};

typedef struct _CartesianMove_SendGoal_type_support_data_t
{
  void * data[2];
} _CartesianMove_SendGoal_type_support_data_t;

static _CartesianMove_SendGoal_type_support_data_t _CartesianMove_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_SendGoal_service_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CartesianMove_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal>,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_GetResult_Request_type_support_ids_t;

static const _CartesianMove_GetResult_Request_type_support_ids_t _CartesianMove_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_GetResult_Request_type_support_symbol_names_t _CartesianMove_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Request)),
  }
};

typedef struct _CartesianMove_GetResult_Request_type_support_data_t
{
  void * data[2];
} _CartesianMove_GetResult_Request_type_support_data_t;

static _CartesianMove_GetResult_Request_type_support_data_t _CartesianMove_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_GetResult_Request_message_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CartesianMove_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Request)() {
  return get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_GetResult_Response_type_support_ids_t;

static const _CartesianMove_GetResult_Response_type_support_ids_t _CartesianMove_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_GetResult_Response_type_support_symbol_names_t _CartesianMove_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Response)),
  }
};

typedef struct _CartesianMove_GetResult_Response_type_support_data_t
{
  void * data[2];
} _CartesianMove_GetResult_Response_type_support_data_t;

static _CartesianMove_GetResult_Response_type_support_data_t _CartesianMove_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_GetResult_Response_message_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CartesianMove_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Response)() {
  return get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_GetResult_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_GetResult_Event_type_support_ids_t;

static const _CartesianMove_GetResult_Event_type_support_ids_t _CartesianMove_GetResult_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_GetResult_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_GetResult_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_GetResult_Event_type_support_symbol_names_t _CartesianMove_GetResult_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Event)),
  }
};

typedef struct _CartesianMove_GetResult_Event_type_support_data_t
{
  void * data[2];
} _CartesianMove_GetResult_Event_type_support_data_t;

static _CartesianMove_GetResult_Event_type_support_data_t _CartesianMove_GetResult_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_GetResult_Event_message_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_GetResult_Event_message_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_GetResult_Event_message_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_GetResult_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CartesianMove_GetResult_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_GetResult_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_GetResult_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult_Event)() {
  return get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_GetResult_type_support_ids_t;

static const _CartesianMove_GetResult_type_support_ids_t _CartesianMove_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_GetResult_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_GetResult_type_support_symbol_names_t _CartesianMove_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult)),
  }
};

typedef struct _CartesianMove_GetResult_type_support_data_t
{
  void * data[2];
} _CartesianMove_GetResult_type_support_data_t;

static _CartesianMove_GetResult_type_support_data_t _CartesianMove_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_GetResult_service_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CartesianMove_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult>,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__functions.h"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _CartesianMove_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CartesianMove_FeedbackMessage_type_support_ids_t;

static const _CartesianMove_FeedbackMessage_type_support_ids_t _CartesianMove_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CartesianMove_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CartesianMove_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CartesianMove_FeedbackMessage_type_support_symbol_names_t _CartesianMove_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_FeedbackMessage)),
  }
};

typedef struct _CartesianMove_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _CartesianMove_FeedbackMessage_type_support_data_t;

static _CartesianMove_FeedbackMessage_type_support_data_t _CartesianMove_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CartesianMove_FeedbackMessage_message_typesupport_map = {
  2,
  "pinkk_usb_insertion_interfaces",
  &_CartesianMove_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_CartesianMove_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_CartesianMove_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CartesianMove_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CartesianMove_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage>()
{
  return &::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove_FeedbackMessage)() {
  return get_message_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t CartesianMove_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL,
  &pinkk_usb_insertion_interfaces__action__CartesianMove__get_type_hash,
  &pinkk_usb_insertion_interfaces__action__CartesianMove__get_type_description,
  &pinkk_usb_insertion_interfaces__action__CartesianMove__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove>()
{
  using ::pinkk_usb_insertion_interfaces::action::rosidl_typesupport_cpp::CartesianMove_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  CartesianMove_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::pinkk_usb_insertion_interfaces::action::CartesianMove::Impl::SendGoalService>();
  CartesianMove_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::pinkk_usb_insertion_interfaces::action::CartesianMove::Impl::GetResultService>();
  CartesianMove_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::pinkk_usb_insertion_interfaces::action::CartesianMove::Impl::CancelGoalService>();
  CartesianMove_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::pinkk_usb_insertion_interfaces::action::CartesianMove::Impl::FeedbackMessage>();
  CartesianMove_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::pinkk_usb_insertion_interfaces::action::CartesianMove::Impl::GoalStatusMessage>();
  return &CartesianMove_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, pinkk_usb_insertion_interfaces, action, CartesianMove)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<pinkk_usb_insertion_interfaces::action::CartesianMove>();
}

#ifdef __cplusplus
}
#endif

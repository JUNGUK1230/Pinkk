// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pinkk_usb_insertion_interfaces:action/CartesianMove.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/action/cartesian_move.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__ACTION__DETAIL__CARTESIAN_MOVE__BUILDER_HPP_
#define PINKK_USB_INSERTION_INTERFACES__ACTION__DETAIL__CARTESIAN_MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace builder
{

class Init_CartesianMove_Goal_lock_roll_pitch
{
public:
  explicit Init_CartesianMove_Goal_lock_roll_pitch(::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal lock_roll_pitch(::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal::_lock_roll_pitch_type arg)
  {
    msg_.lock_roll_pitch = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal msg_;
};

class Init_CartesianMove_Goal_lock_z
{
public:
  explicit Init_CartesianMove_Goal_lock_z(::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal & msg)
  : msg_(msg)
  {}
  Init_CartesianMove_Goal_lock_roll_pitch lock_z(::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal::_lock_z_type arg)
  {
    msg_.lock_z = std::move(arg);
    return Init_CartesianMove_Goal_lock_roll_pitch(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal msg_;
};

class Init_CartesianMove_Goal_mode
{
public:
  explicit Init_CartesianMove_Goal_mode(::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal & msg)
  : msg_(msg)
  {}
  Init_CartesianMove_Goal_lock_z mode(::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_CartesianMove_Goal_lock_z(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal msg_;
};

class Init_CartesianMove_Goal_speed
{
public:
  explicit Init_CartesianMove_Goal_speed(::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal & msg)
  : msg_(msg)
  {}
  Init_CartesianMove_Goal_mode speed(::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_CartesianMove_Goal_mode(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal msg_;
};

class Init_CartesianMove_Goal_target
{
public:
  Init_CartesianMove_Goal_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianMove_Goal_speed target(::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_CartesianMove_Goal_speed(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::action::CartesianMove_Goal>()
{
  return pinkk_usb_insertion_interfaces::action::builder::Init_CartesianMove_Goal_target();
}

}  // namespace pinkk_usb_insertion_interfaces


namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace builder
{

class Init_CartesianMove_Result_actual
{
public:
  explicit Init_CartesianMove_Result_actual(::pinkk_usb_insertion_interfaces::action::CartesianMove_Result & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Result actual(::pinkk_usb_insertion_interfaces::action::CartesianMove_Result::_actual_type arg)
  {
    msg_.actual = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Result msg_;
};

class Init_CartesianMove_Result_message
{
public:
  explicit Init_CartesianMove_Result_message(::pinkk_usb_insertion_interfaces::action::CartesianMove_Result & msg)
  : msg_(msg)
  {}
  Init_CartesianMove_Result_actual message(::pinkk_usb_insertion_interfaces::action::CartesianMove_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_CartesianMove_Result_actual(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Result msg_;
};

class Init_CartesianMove_Result_success
{
public:
  Init_CartesianMove_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianMove_Result_message success(::pinkk_usb_insertion_interfaces::action::CartesianMove_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CartesianMove_Result_message(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::action::CartesianMove_Result>()
{
  return pinkk_usb_insertion_interfaces::action::builder::Init_CartesianMove_Result_success();
}

}  // namespace pinkk_usb_insertion_interfaces


namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace builder
{

class Init_CartesianMove_Feedback_orientation_error_deg
{
public:
  explicit Init_CartesianMove_Feedback_orientation_error_deg(::pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback orientation_error_deg(::pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback::_orientation_error_deg_type arg)
  {
    msg_.orientation_error_deg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback msg_;
};

class Init_CartesianMove_Feedback_position_error_m
{
public:
  explicit Init_CartesianMove_Feedback_position_error_m(::pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback & msg)
  : msg_(msg)
  {}
  Init_CartesianMove_Feedback_orientation_error_deg position_error_m(::pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback::_position_error_m_type arg)
  {
    msg_.position_error_m = std::move(arg);
    return Init_CartesianMove_Feedback_orientation_error_deg(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback msg_;
};

class Init_CartesianMove_Feedback_actual
{
public:
  Init_CartesianMove_Feedback_actual()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianMove_Feedback_position_error_m actual(::pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback::_actual_type arg)
  {
    msg_.actual = std::move(arg);
    return Init_CartesianMove_Feedback_position_error_m(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback>()
{
  return pinkk_usb_insertion_interfaces::action::builder::Init_CartesianMove_Feedback_actual();
}

}  // namespace pinkk_usb_insertion_interfaces


namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace builder
{

class Init_CartesianMove_SendGoal_Request_goal
{
public:
  explicit Init_CartesianMove_SendGoal_Request_goal(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request goal(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request msg_;
};

class Init_CartesianMove_SendGoal_Request_goal_id
{
public:
  Init_CartesianMove_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianMove_SendGoal_Request_goal goal_id(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_CartesianMove_SendGoal_Request_goal(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request>()
{
  return pinkk_usb_insertion_interfaces::action::builder::Init_CartesianMove_SendGoal_Request_goal_id();
}

}  // namespace pinkk_usb_insertion_interfaces


namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace builder
{

class Init_CartesianMove_SendGoal_Response_stamp
{
public:
  explicit Init_CartesianMove_SendGoal_Response_stamp(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response stamp(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response msg_;
};

class Init_CartesianMove_SendGoal_Response_accepted
{
public:
  Init_CartesianMove_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianMove_SendGoal_Response_stamp accepted(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_CartesianMove_SendGoal_Response_stamp(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response>()
{
  return pinkk_usb_insertion_interfaces::action::builder::Init_CartesianMove_SendGoal_Response_accepted();
}

}  // namespace pinkk_usb_insertion_interfaces


namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace builder
{

class Init_CartesianMove_SendGoal_Event_response
{
public:
  explicit Init_CartesianMove_SendGoal_Event_response(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event response(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event msg_;
};

class Init_CartesianMove_SendGoal_Event_request
{
public:
  explicit Init_CartesianMove_SendGoal_Event_request(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_CartesianMove_SendGoal_Event_response request(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CartesianMove_SendGoal_Event_response(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event msg_;
};

class Init_CartesianMove_SendGoal_Event_info
{
public:
  Init_CartesianMove_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianMove_SendGoal_Event_request info(::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CartesianMove_SendGoal_Event_request(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event>()
{
  return pinkk_usb_insertion_interfaces::action::builder::Init_CartesianMove_SendGoal_Event_info();
}

}  // namespace pinkk_usb_insertion_interfaces


namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace builder
{

class Init_CartesianMove_GetResult_Request_goal_id
{
public:
  Init_CartesianMove_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request goal_id(::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request>()
{
  return pinkk_usb_insertion_interfaces::action::builder::Init_CartesianMove_GetResult_Request_goal_id();
}

}  // namespace pinkk_usb_insertion_interfaces


namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace builder
{

class Init_CartesianMove_GetResult_Response_result
{
public:
  explicit Init_CartesianMove_GetResult_Response_result(::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response result(::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response msg_;
};

class Init_CartesianMove_GetResult_Response_status
{
public:
  Init_CartesianMove_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianMove_GetResult_Response_result status(::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_CartesianMove_GetResult_Response_result(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response>()
{
  return pinkk_usb_insertion_interfaces::action::builder::Init_CartesianMove_GetResult_Response_status();
}

}  // namespace pinkk_usb_insertion_interfaces


namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace builder
{

class Init_CartesianMove_GetResult_Event_response
{
public:
  explicit Init_CartesianMove_GetResult_Event_response(::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event response(::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event msg_;
};

class Init_CartesianMove_GetResult_Event_request
{
public:
  explicit Init_CartesianMove_GetResult_Event_request(::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_CartesianMove_GetResult_Event_response request(::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CartesianMove_GetResult_Event_response(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event msg_;
};

class Init_CartesianMove_GetResult_Event_info
{
public:
  Init_CartesianMove_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianMove_GetResult_Event_request info(::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CartesianMove_GetResult_Event_request(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event>()
{
  return pinkk_usb_insertion_interfaces::action::builder::Init_CartesianMove_GetResult_Event_info();
}

}  // namespace pinkk_usb_insertion_interfaces


namespace pinkk_usb_insertion_interfaces
{

namespace action
{

namespace builder
{

class Init_CartesianMove_FeedbackMessage_feedback
{
public:
  explicit Init_CartesianMove_FeedbackMessage_feedback(::pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage feedback(::pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage msg_;
};

class Init_CartesianMove_FeedbackMessage_goal_id
{
public:
  Init_CartesianMove_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianMove_FeedbackMessage_feedback goal_id(::pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_CartesianMove_FeedbackMessage_feedback(msg_);
  }

private:
  ::pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage>()
{
  return pinkk_usb_insertion_interfaces::action::builder::Init_CartesianMove_FeedbackMessage_goal_id();
}

}  // namespace pinkk_usb_insertion_interfaces

#endif  // PINKK_USB_INSERTION_INTERFACES__ACTION__DETAIL__CARTESIAN_MOVE__BUILDER_HPP_

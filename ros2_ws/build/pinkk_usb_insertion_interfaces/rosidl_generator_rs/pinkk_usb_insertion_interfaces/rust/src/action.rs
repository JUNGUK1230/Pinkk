
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lock_z: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lock_roll_pitch: bool,

}



impl Default for CartesianMove_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::CartesianMove_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_Goal {
  type RmwMsg = super::action::rmw::CartesianMove_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.target)).into_owned(),
        speed: msg.speed,
        mode: msg.mode,
        lock_z: msg.lock_z,
        lock_roll_pitch: msg.lock_roll_pitch,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.target)).into_owned(),
      speed: msg.speed,
      mode: msg.mode,
      lock_z: msg.lock_z,
      lock_roll_pitch: msg.lock_roll_pitch,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      target: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.target),
      speed: msg.speed,
      mode: msg.mode,
      lock_z: msg.lock_z,
      lock_roll_pitch: msg.lock_roll_pitch,
    }
  }
}


// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub actual: geometry_msgs::msg::PoseStamped,

}



impl Default for CartesianMove_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::CartesianMove_Result::default())
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_Result {
  type RmwMsg = super::action::rmw::CartesianMove_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
        actual: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.actual)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
        actual: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.actual)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
      actual: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.actual),
    }
  }
}


// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub actual: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position_error_m: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub orientation_error_deg: f64,

}



impl Default for CartesianMove_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::CartesianMove_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_Feedback {
  type RmwMsg = super::action::rmw::CartesianMove_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        actual: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.actual)).into_owned(),
        position_error_m: msg.position_error_m,
        orientation_error_deg: msg.orientation_error_deg,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        actual: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.actual)).into_owned(),
      position_error_m: msg.position_error_m,
      orientation_error_deg: msg.orientation_error_deg,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      actual: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.actual),
      position_error_m: msg.position_error_m,
      orientation_error_deg: msg.orientation_error_deg,
    }
  }
}


// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::CartesianMove_Feedback,

}



impl Default for CartesianMove_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::CartesianMove_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_FeedbackMessage {
  type RmwMsg = super::action::rmw::CartesianMove_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::CartesianMove_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::CartesianMove_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::CartesianMove_Feedback::from_rmw_message(msg.feedback),
    }
  }
}






// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::CartesianMove_Goal,

}



impl Default for CartesianMove_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::CartesianMove_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_SendGoal_Request {
  type RmwMsg = super::action::rmw::CartesianMove_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::CartesianMove_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::CartesianMove_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::CartesianMove_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for CartesianMove_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::CartesianMove_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_SendGoal_Response {
  type RmwMsg = super::action::rmw::CartesianMove_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for CartesianMove_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::CartesianMove_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_GetResult_Request {
  type RmwMsg = super::action::rmw::CartesianMove_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CartesianMove_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::CartesianMove_Result,

}



impl Default for CartesianMove_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::CartesianMove_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for CartesianMove_GetResult_Response {
  type RmwMsg = super::action::rmw::CartesianMove_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::CartesianMove_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::CartesianMove_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::CartesianMove_Result::from_rmw_message(msg.result),
    }
  }
}






#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct CartesianMove_SendGoal;

impl rosidl_runtime_rs::Service for CartesianMove_SendGoal {
    type Request = CartesianMove_SendGoal_Request;
    type Response = CartesianMove_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal() }
    }
}




#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct CartesianMove_GetResult;

impl rosidl_runtime_rs::Service for CartesianMove_GetResult {
    type Request = CartesianMove_GetResult_Request;
    type Response = CartesianMove_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult() }
    }
}






#[link(name = "pinkk_usb_insertion_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove() -> *const std::ffi::c_void;
}

// Corresponds to pinkk_usb_insertion_interfaces__action__CartesianMove
#[allow(missing_docs, non_camel_case_types)]
pub struct CartesianMove;

impl rosidl_runtime_rs::Action for CartesianMove {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = CartesianMove_Goal;

  /// The result message defined in the action definition.
  type Result = CartesianMove_Result;

  /// The feedback message defined in the action definition.
  type Feedback = CartesianMove_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::CartesianMove_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::CartesianMove_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::CartesianMove_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__pinkk_usb_insertion_interfaces__action__CartesianMove() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::CartesianMove_Goal,
  ) -> super::action::rmw::CartesianMove_SendGoal_Request {
   super::action::rmw::CartesianMove_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::CartesianMove_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::CartesianMove_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::CartesianMove_SendGoal_Response {
   super::action::rmw::CartesianMove_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::CartesianMove_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::CartesianMove_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::CartesianMove_Feedback,
  ) -> super::action::rmw::CartesianMove_FeedbackMessage {
    let mut message = super::action::rmw::CartesianMove_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::CartesianMove_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::CartesianMove_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::CartesianMove_GetResult_Request {
   super::action::rmw::CartesianMove_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::CartesianMove_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::CartesianMove_Result,
  ) -> super::action::rmw::CartesianMove_GetResult_Response {
   super::action::rmw::CartesianMove_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::CartesianMove_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::CartesianMove_Result,
  ) {
    (response.status, response.result)
  }
}



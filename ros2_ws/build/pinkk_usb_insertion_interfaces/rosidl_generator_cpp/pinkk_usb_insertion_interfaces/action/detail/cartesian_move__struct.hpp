// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pinkk_usb_insertion_interfaces:action/CartesianMove.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinkk_usb_insertion_interfaces/action/cartesian_move.hpp"


#ifndef PINKK_USB_INSERTION_INTERFACES__ACTION__DETAIL__CARTESIAN_MOVE__STRUCT_HPP_
#define PINKK_USB_INSERTION_INTERFACES__ACTION__DETAIL__CARTESIAN_MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'target'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Goal __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Goal __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CartesianMove_Goal_
{
  using Type = CartesianMove_Goal_<ContainerAllocator>;

  explicit CartesianMove_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0l;
      this->mode = 0l;
      this->lock_z = false;
      this->lock_roll_pitch = false;
    }
  }

  explicit CartesianMove_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0l;
      this->mode = 0l;
      this->lock_z = false;
      this->lock_roll_pitch = false;
    }
  }

  // field types and members
  using _target_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _target_type target;
  using _speed_type =
    int32_t;
  _speed_type speed;
  using _mode_type =
    int32_t;
  _mode_type mode;
  using _lock_z_type =
    bool;
  _lock_z_type lock_z;
  using _lock_roll_pitch_type =
    bool;
  _lock_roll_pitch_type lock_roll_pitch;

  // setters for named parameter idiom
  Type & set__target(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->target = _arg;
    return *this;
  }
  Type & set__speed(
    const int32_t & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__mode(
    const int32_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__lock_z(
    const bool & _arg)
  {
    this->lock_z = _arg;
    return *this;
  }
  Type & set__lock_roll_pitch(
    const bool & _arg)
  {
    this->lock_roll_pitch = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Goal
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Goal
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianMove_Goal_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->lock_z != other.lock_z) {
      return false;
    }
    if (this->lock_roll_pitch != other.lock_roll_pitch) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianMove_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianMove_Goal_

// alias to use template instance with default allocator
using CartesianMove_Goal =
  pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces


// Include directives for member types
// Member 'actual'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Result __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Result __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CartesianMove_Result_
{
  using Type = CartesianMove_Result_<ContainerAllocator>;

  explicit CartesianMove_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : actual(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit CartesianMove_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    actual(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _actual_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _actual_type actual;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__actual(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->actual = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Result
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Result
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianMove_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->actual != other.actual) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianMove_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianMove_Result_

// alias to use template instance with default allocator
using CartesianMove_Result =
  pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces


// Include directives for member types
// Member 'actual'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CartesianMove_Feedback_
{
  using Type = CartesianMove_Feedback_<ContainerAllocator>;

  explicit CartesianMove_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : actual(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position_error_m = 0.0;
      this->orientation_error_deg = 0.0;
    }
  }

  explicit CartesianMove_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : actual(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position_error_m = 0.0;
      this->orientation_error_deg = 0.0;
    }
  }

  // field types and members
  using _actual_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _actual_type actual;
  using _position_error_m_type =
    double;
  _position_error_m_type position_error_m;
  using _orientation_error_deg_type =
    double;
  _orientation_error_deg_type orientation_error_deg;

  // setters for named parameter idiom
  Type & set__actual(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->actual = _arg;
    return *this;
  }
  Type & set__position_error_m(
    const double & _arg)
  {
    this->position_error_m = _arg;
    return *this;
  }
  Type & set__orientation_error_deg(
    const double & _arg)
  {
    this->orientation_error_deg = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_Feedback
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianMove_Feedback_ & other) const
  {
    if (this->actual != other.actual) {
      return false;
    }
    if (this->position_error_m != other.position_error_m) {
      return false;
    }
    if (this->orientation_error_deg != other.orientation_error_deg) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianMove_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianMove_Feedback_

// alias to use template instance with default allocator
using CartesianMove_Feedback =
  pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CartesianMove_SendGoal_Request_
{
  using Type = CartesianMove_SendGoal_Request_<ContainerAllocator>;

  explicit CartesianMove_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit CartesianMove_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const pinkk_usb_insertion_interfaces::action::CartesianMove_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Request
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianMove_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianMove_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianMove_SendGoal_Request_

// alias to use template instance with default allocator
using CartesianMove_SendGoal_Request =
  pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CartesianMove_SendGoal_Response_
{
  using Type = CartesianMove_SendGoal_Response_<ContainerAllocator>;

  explicit CartesianMove_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit CartesianMove_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Response
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianMove_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianMove_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianMove_SendGoal_Response_

// alias to use template instance with default allocator
using CartesianMove_SendGoal_Response =
  pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CartesianMove_SendGoal_Event_
{
  using Type = CartesianMove_SendGoal_Event_<ContainerAllocator>;

  explicit CartesianMove_SendGoal_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit CartesianMove_SendGoal_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_SendGoal_Event
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianMove_SendGoal_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianMove_SendGoal_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianMove_SendGoal_Event_

// alias to use template instance with default allocator
using CartesianMove_SendGoal_Event =
  pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

struct CartesianMove_SendGoal
{
  using Request = pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Request;
  using Response = pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Response;
  using Event = pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal_Event;
};

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CartesianMove_GetResult_Request_
{
  using Type = CartesianMove_GetResult_Request_<ContainerAllocator>;

  explicit CartesianMove_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit CartesianMove_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Request
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianMove_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianMove_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianMove_GetResult_Request_

// alias to use template instance with default allocator
using CartesianMove_GetResult_Request =
  pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CartesianMove_GetResult_Response_
{
  using Type = CartesianMove_GetResult_Response_<ContainerAllocator>;

  explicit CartesianMove_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit CartesianMove_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const pinkk_usb_insertion_interfaces::action::CartesianMove_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Response
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianMove_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianMove_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianMove_GetResult_Response_

// alias to use template instance with default allocator
using CartesianMove_GetResult_Response =
  pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces


// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CartesianMove_GetResult_Event_
{
  using Type = CartesianMove_GetResult_Event_<ContainerAllocator>;

  explicit CartesianMove_GetResult_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit CartesianMove_GetResult_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_GetResult_Event
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianMove_GetResult_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianMove_GetResult_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianMove_GetResult_Event_

// alias to use template instance with default allocator
using CartesianMove_GetResult_Event =
  pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

struct CartesianMove_GetResult
{
  using Request = pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Request;
  using Response = pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Response;
  using Event = pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult_Event;
};

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "pinkk_usb_insertion_interfaces/action/detail/cartesian_move__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage __declspec(deprecated)
#endif

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CartesianMove_FeedbackMessage_
{
  using Type = CartesianMove_FeedbackMessage_<ContainerAllocator>;

  explicit CartesianMove_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit CartesianMove_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinkk_usb_insertion_interfaces__action__CartesianMove_FeedbackMessage
    std::shared_ptr<pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianMove_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianMove_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianMove_FeedbackMessage_

// alias to use template instance with default allocator
using CartesianMove_FeedbackMessage =
  pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace pinkk_usb_insertion_interfaces
{

namespace action
{

struct CartesianMove
{
  /// The goal message defined in the action definition.
  using Goal = pinkk_usb_insertion_interfaces::action::CartesianMove_Goal;
  /// The result message defined in the action definition.
  using Result = pinkk_usb_insertion_interfaces::action::CartesianMove_Result;
  /// The feedback message defined in the action definition.
  using Feedback = pinkk_usb_insertion_interfaces::action::CartesianMove_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = pinkk_usb_insertion_interfaces::action::CartesianMove_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = pinkk_usb_insertion_interfaces::action::CartesianMove_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = pinkk_usb_insertion_interfaces::action::CartesianMove_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct CartesianMove CartesianMove;

}  // namespace action

}  // namespace pinkk_usb_insertion_interfaces

#endif  // PINKK_USB_INSERTION_INTERFACES__ACTION__DETAIL__CARTESIAN_MOVE__STRUCT_HPP_

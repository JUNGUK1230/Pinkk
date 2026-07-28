# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target pinkk_usb_insertion_interfaces::pinkk_usb_insertion_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${pinkk_usb_insertion_interfaces_TARGETS}.
if(pinkk_usb_insertion_interfaces_TARGETS AND NOT TARGET pinkk_usb_insertion_interfaces::pinkk_usb_insertion_interfaces)
  add_library(pinkk_usb_insertion_interfaces::pinkk_usb_insertion_interfaces INTERFACE IMPORTED)
  set_target_properties(pinkk_usb_insertion_interfaces::pinkk_usb_insertion_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${pinkk_usb_insertion_interfaces_TARGETS}")
endif()

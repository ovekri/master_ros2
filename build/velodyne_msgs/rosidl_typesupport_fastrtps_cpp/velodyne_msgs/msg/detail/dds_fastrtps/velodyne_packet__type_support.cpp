// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from velodyne_msgs:msg/VelodynePacket.idl
// generated code does not contain a copyright notice
#include "velodyne_msgs/msg/detail/velodyne_packet__rosidl_typesupport_fastrtps_cpp.hpp"
#include "velodyne_msgs/msg/detail/velodyne_packet__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace builtin_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const builtin_interfaces::msg::Time &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  builtin_interfaces::msg::Time &);
size_t get_serialized_size(
  const builtin_interfaces::msg::Time &,
  size_t current_alignment);
size_t
max_serialized_size_Time(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace builtin_interfaces


namespace velodyne_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_velodyne_msgs
cdr_serialize(
  const velodyne_msgs::msg::VelodynePacket & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: stamp
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.stamp,
    cdr);
  // Member: data
  {
    cdr << ros_message.data;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_velodyne_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  velodyne_msgs::msg::VelodynePacket & ros_message)
{
  // Member: stamp
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.stamp);

  // Member: data
  {
    cdr >> ros_message.data;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_velodyne_msgs
get_serialized_size(
  const velodyne_msgs::msg::VelodynePacket & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: stamp

  current_alignment +=
    builtin_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.stamp, current_alignment);
  // Member: data
  {
    size_t array_size = 1206;
    size_t item_size = sizeof(ros_message.data[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_velodyne_msgs
max_serialized_size_VelodynePacket(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: stamp
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        builtin_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Time(
        full_bounded, current_alignment);
    }
  }

  // Member: data
  {
    size_t array_size = 1206;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _VelodynePacket__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const velodyne_msgs::msg::VelodynePacket *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _VelodynePacket__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<velodyne_msgs::msg::VelodynePacket *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _VelodynePacket__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const velodyne_msgs::msg::VelodynePacket *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _VelodynePacket__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_VelodynePacket(full_bounded, 0);
}

static message_type_support_callbacks_t _VelodynePacket__callbacks = {
  "velodyne_msgs::msg",
  "VelodynePacket",
  _VelodynePacket__cdr_serialize,
  _VelodynePacket__cdr_deserialize,
  _VelodynePacket__get_serialized_size,
  _VelodynePacket__max_serialized_size
};

static rosidl_message_type_support_t _VelodynePacket__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_VelodynePacket__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace velodyne_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_velodyne_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<velodyne_msgs::msg::VelodynePacket>()
{
  return &velodyne_msgs::msg::typesupport_fastrtps_cpp::_VelodynePacket__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, velodyne_msgs, msg, VelodynePacket)() {
  return &velodyne_msgs::msg::typesupport_fastrtps_cpp::_VelodynePacket__handle;
}

#ifdef __cplusplus
}
#endif

// Generated by gencpp from file mav_planning_msgs/ChangeNameServiceResponse.msg
// DO NOT EDIT!


#ifndef MAV_PLANNING_MSGS_MESSAGE_CHANGENAMESERVICERESPONSE_H
#define MAV_PLANNING_MSGS_MESSAGE_CHANGENAMESERVICERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mav_planning_msgs
{
template <class ContainerAllocator>
struct ChangeNameServiceResponse_
{
  typedef ChangeNameServiceResponse_<ContainerAllocator> Type;

  ChangeNameServiceResponse_()
    : success(false)
    , message()  {
    }
  ChangeNameServiceResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , message(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;





  typedef boost::shared_ptr< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ChangeNameServiceResponse_

typedef ::mav_planning_msgs::ChangeNameServiceResponse_<std::allocator<void> > ChangeNameServiceResponse;

typedef boost::shared_ptr< ::mav_planning_msgs::ChangeNameServiceResponse > ChangeNameServiceResponsePtr;
typedef boost::shared_ptr< ::mav_planning_msgs::ChangeNameServiceResponse const> ChangeNameServiceResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mav_planning_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'mav_msgs': ['/home/vinayaka/project_ws/src/mav_comm/mav_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'mav_planning_msgs': ['/home/vinayaka/project_ws/src/mav_comm/mav_planning_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "937c9679a518e3a18d831e57125ea522";
  }

  static const char* value(const ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x937c9679a518e3a1ULL;
  static const uint64_t static_value2 = 0x8d831e57125ea522ULL;
};

template<class ContainerAllocator>
struct DataType< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mav_planning_msgs/ChangeNameServiceResponse";
  }

  static const char* value(const ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
bool success\n\
string message\n\
\n\
";
  }

  static const char* value(const ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ChangeNameServiceResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mav_planning_msgs::ChangeNameServiceResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAV_PLANNING_MSGS_MESSAGE_CHANGENAMESERVICERESPONSE_H

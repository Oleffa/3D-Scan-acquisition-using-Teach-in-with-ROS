// Generated by gencpp from file riegl/RieglTime.msg
// DO NOT EDIT!


#ifndef RIEGL_MESSAGE_RIEGLTIME_H
#define RIEGL_MESSAGE_RIEGLTIME_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace riegl
{
template <class ContainerAllocator>
struct RieglTime_
{
  typedef RieglTime_<ContainerAllocator> Type;

  RieglTime_()
    : header()
    , time(0.0)  {
    }
  RieglTime_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _time_type;
  _time_type time;




  typedef boost::shared_ptr< ::riegl::RieglTime_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::riegl::RieglTime_<ContainerAllocator> const> ConstPtr;

}; // struct RieglTime_

typedef ::riegl::RieglTime_<std::allocator<void> > RieglTime;

typedef boost::shared_ptr< ::riegl::RieglTime > RieglTimePtr;
typedef boost::shared_ptr< ::riegl::RieglTime const> RieglTimeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::riegl::RieglTime_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::riegl::RieglTime_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace riegl

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/jade/share/std_msgs/cmake/../msg'], 'riegl': ['/home/oleffa/catkin_ws/src/riegl/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::riegl::RieglTime_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::riegl::RieglTime_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::riegl::RieglTime_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::riegl::RieglTime_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::riegl::RieglTime_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::riegl::RieglTime_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::riegl::RieglTime_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dce2d1e2213c6c29642ee1125033b12b";
  }

  static const char* value(const ::riegl::RieglTime_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdce2d1e2213c6c29ULL;
  static const uint64_t static_value2 = 0x642ee1125033b12bULL;
};

template<class ContainerAllocator>
struct DataType< ::riegl::RieglTime_<ContainerAllocator> >
{
  static const char* value()
  {
    return "riegl/RieglTime";
  }

  static const char* value(const ::riegl::RieglTime_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::riegl::RieglTime_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
# 0 scanning\n\
# 1 frame start\n\
# 2 frame stop\n\
float64 time\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::riegl::RieglTime_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::riegl::RieglTime_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RieglTime_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::riegl::RieglTime_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::riegl::RieglTime_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RIEGL_MESSAGE_RIEGLTIME_H

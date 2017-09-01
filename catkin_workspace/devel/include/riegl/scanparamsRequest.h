// Generated by gencpp from file riegl/scanparamsRequest.msg
// DO NOT EDIT!


#ifndef RIEGL_MESSAGE_SCANPARAMSREQUEST_H
#define RIEGL_MESSAGE_SCANPARAMSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace riegl
{
template <class ContainerAllocator>
struct scanparamsRequest_
{
  typedef scanparamsRequest_<ContainerAllocator> Type;

  scanparamsRequest_()
    : lineangle(0.0)
    , frameangle(0.0)
    , maxlineangle(0.0)
    , minlineangle(0.0)
    , maxframeangle(0.0)
    , minframeangle(0.0)  {
    }
  scanparamsRequest_(const ContainerAllocator& _alloc)
    : lineangle(0.0)
    , frameangle(0.0)
    , maxlineangle(0.0)
    , minlineangle(0.0)
    , maxframeangle(0.0)
    , minframeangle(0.0)  {
  (void)_alloc;
    }



   typedef double _lineangle_type;
  _lineangle_type lineangle;

   typedef double _frameangle_type;
  _frameangle_type frameangle;

   typedef double _maxlineangle_type;
  _maxlineangle_type maxlineangle;

   typedef double _minlineangle_type;
  _minlineangle_type minlineangle;

   typedef double _maxframeangle_type;
  _maxframeangle_type maxframeangle;

   typedef double _minframeangle_type;
  _minframeangle_type minframeangle;




  typedef boost::shared_ptr< ::riegl::scanparamsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::riegl::scanparamsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct scanparamsRequest_

typedef ::riegl::scanparamsRequest_<std::allocator<void> > scanparamsRequest;

typedef boost::shared_ptr< ::riegl::scanparamsRequest > scanparamsRequestPtr;
typedef boost::shared_ptr< ::riegl::scanparamsRequest const> scanparamsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::riegl::scanparamsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::riegl::scanparamsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace riegl

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/jade/share/std_msgs/cmake/../msg'], 'riegl': ['/home/oleffa/catkin_ws/src/riegl/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::riegl::scanparamsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::riegl::scanparamsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::riegl::scanparamsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::riegl::scanparamsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::riegl::scanparamsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::riegl::scanparamsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::riegl::scanparamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0852ff892d6a7a08b5e3435414667c44";
  }

  static const char* value(const ::riegl::scanparamsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0852ff892d6a7a08ULL;
  static const uint64_t static_value2 = 0xb5e3435414667c44ULL;
};

template<class ContainerAllocator>
struct DataType< ::riegl::scanparamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "riegl/scanparamsRequest";
  }

  static const char* value(const ::riegl::scanparamsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::riegl::scanparamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 lineangle\n\
float64 frameangle\n\
float64 maxlineangle\n\
float64 minlineangle\n\
float64 maxframeangle\n\
float64 minframeangle\n\
";
  }

  static const char* value(const ::riegl::scanparamsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::riegl::scanparamsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.lineangle);
      stream.next(m.frameangle);
      stream.next(m.maxlineangle);
      stream.next(m.minlineangle);
      stream.next(m.maxframeangle);
      stream.next(m.minframeangle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct scanparamsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::riegl::scanparamsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::riegl::scanparamsRequest_<ContainerAllocator>& v)
  {
    s << indent << "lineangle: ";
    Printer<double>::stream(s, indent + "  ", v.lineangle);
    s << indent << "frameangle: ";
    Printer<double>::stream(s, indent + "  ", v.frameangle);
    s << indent << "maxlineangle: ";
    Printer<double>::stream(s, indent + "  ", v.maxlineangle);
    s << indent << "minlineangle: ";
    Printer<double>::stream(s, indent + "  ", v.minlineangle);
    s << indent << "maxframeangle: ";
    Printer<double>::stream(s, indent + "  ", v.maxframeangle);
    s << indent << "minframeangle: ";
    Printer<double>::stream(s, indent + "  ", v.minframeangle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RIEGL_MESSAGE_SCANPARAMSREQUEST_H
// Generated by gencpp from file riegl/scanparams.msg
// DO NOT EDIT!


#ifndef RIEGL_MESSAGE_SCANPARAMS_H
#define RIEGL_MESSAGE_SCANPARAMS_H

#include <ros/service_traits.h>


#include <riegl/scanparamsRequest.h>
#include <riegl/scanparamsResponse.h>


namespace riegl
{

struct scanparams
{

typedef scanparamsRequest Request;
typedef scanparamsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct scanparams
} // namespace riegl


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::riegl::scanparams > {
  static const char* value()
  {
    return "0852ff892d6a7a08b5e3435414667c44";
  }

  static const char* value(const ::riegl::scanparams&) { return value(); }
};

template<>
struct DataType< ::riegl::scanparams > {
  static const char* value()
  {
    return "riegl/scanparams";
  }

  static const char* value(const ::riegl::scanparams&) { return value(); }
};


// service_traits::MD5Sum< ::riegl::scanparamsRequest> should match 
// service_traits::MD5Sum< ::riegl::scanparams > 
template<>
struct MD5Sum< ::riegl::scanparamsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::riegl::scanparams >::value();
  }
  static const char* value(const ::riegl::scanparamsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::riegl::scanparamsRequest> should match 
// service_traits::DataType< ::riegl::scanparams > 
template<>
struct DataType< ::riegl::scanparamsRequest>
{
  static const char* value()
  {
    return DataType< ::riegl::scanparams >::value();
  }
  static const char* value(const ::riegl::scanparamsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::riegl::scanparamsResponse> should match 
// service_traits::MD5Sum< ::riegl::scanparams > 
template<>
struct MD5Sum< ::riegl::scanparamsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::riegl::scanparams >::value();
  }
  static const char* value(const ::riegl::scanparamsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::riegl::scanparamsResponse> should match 
// service_traits::DataType< ::riegl::scanparams > 
template<>
struct DataType< ::riegl::scanparamsResponse>
{
  static const char* value()
  {
    return DataType< ::riegl::scanparams >::value();
  }
  static const char* value(const ::riegl::scanparamsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RIEGL_MESSAGE_SCANPARAMS_H

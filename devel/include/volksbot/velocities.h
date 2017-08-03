// Generated by gencpp from file volksbot/velocities.msg
// DO NOT EDIT!


#ifndef VOLKSBOT_MESSAGE_VELOCITIES_H
#define VOLKSBOT_MESSAGE_VELOCITIES_H

#include <ros/service_traits.h>


#include <volksbot/velocitiesRequest.h>
#include <volksbot/velocitiesResponse.h>


namespace volksbot
{

struct velocities
{

typedef velocitiesRequest Request;
typedef velocitiesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct velocities
} // namespace volksbot


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::volksbot::velocities > {
  static const char* value()
  {
    return "50c2436c38cded221d061b57126c4e40";
  }

  static const char* value(const ::volksbot::velocities&) { return value(); }
};

template<>
struct DataType< ::volksbot::velocities > {
  static const char* value()
  {
    return "volksbot/velocities";
  }

  static const char* value(const ::volksbot::velocities&) { return value(); }
};


// service_traits::MD5Sum< ::volksbot::velocitiesRequest> should match 
// service_traits::MD5Sum< ::volksbot::velocities > 
template<>
struct MD5Sum< ::volksbot::velocitiesRequest>
{
  static const char* value()
  {
    return MD5Sum< ::volksbot::velocities >::value();
  }
  static const char* value(const ::volksbot::velocitiesRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::volksbot::velocitiesRequest> should match 
// service_traits::DataType< ::volksbot::velocities > 
template<>
struct DataType< ::volksbot::velocitiesRequest>
{
  static const char* value()
  {
    return DataType< ::volksbot::velocities >::value();
  }
  static const char* value(const ::volksbot::velocitiesRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::volksbot::velocitiesResponse> should match 
// service_traits::MD5Sum< ::volksbot::velocities > 
template<>
struct MD5Sum< ::volksbot::velocitiesResponse>
{
  static const char* value()
  {
    return MD5Sum< ::volksbot::velocities >::value();
  }
  static const char* value(const ::volksbot::velocitiesResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::volksbot::velocitiesResponse> should match 
// service_traits::DataType< ::volksbot::velocities > 
template<>
struct DataType< ::volksbot::velocitiesResponse>
{
  static const char* value()
  {
    return DataType< ::volksbot::velocities >::value();
  }
  static const char* value(const ::volksbot::velocitiesResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // VOLKSBOT_MESSAGE_VELOCITIES_H

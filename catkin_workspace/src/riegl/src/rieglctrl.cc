#include <ros/ros.h>
#include "stdio.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include "sys/time.h"

#include <unistd.h>
#include <fcntl.h>
#include <stdexcept>

#include "riegl/rieglctrl.h"

#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include "riegl/angle.h"
#include "riegl/scanparams.h"
#include "riegl/Command.h"
#include "riegl/selection.h"




namespace riegl {

RieglCtrl::~RieglCtrl() {
  ROS_INFO("Cleaning up!");
  try {
    session.execute_command("MEAS_ABORT", "", &result, &status, &comment);
    logStatus();
    // now wait for the scanner to abort a scan in progress:
    session.execute_command("MEAS_BUSY", "1", &result, &status, &comment);
    logStatus();
      
    
    // store scan internally:
    session.set_property("STOR_MEDIA", "1");
    session.get_property("STOR_MEM", result);

    ROS_INFO("Internal memory is to %s %% full.", result.c_str());
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute cleanup commands :%s", e.what() );
  }
  session.close();
}

RieglCtrl::RieglCtrl(string address, 
                     double minla, double la, double maxla, 
                     double minfa, double fa, double maxfa, 
                     int mode ) {
  lineangle = la;
  frameangle = fa;
  maxlineangle = maxla;
  minlineangle = minla;
  maxframeangle = maxfa;
  minframeangle = minfa;
  bool connected = true;
  advertise(); 
//  do {
    try {
      session.open(address);
      // abort a scan that might be in progress:
      session.execute_command("MEAS_ABORT", "", &result, &status, &comment);
      logStatus();

      // now wait for the scanner to abort a scan in progress:
      session.execute_command("MEAS_BUSY", "1", &result, &status, &comment);
      logStatus();

      // don't store scan internally:
      session.set_property("STOR_MEDIA", "0");
      if(false) { 
        session.set_property("STOR_MEDIA", "2");
        session.set_property("REFLS_MINPOINTS", "2");
        session.set_property("REFLS_NBMINDIST", "0.2");
        session.set_property("REFLS_DIAMETER", "0.06");
        session.set_property("REFLS_MODE", "1"); // 1=MONITOR
        session.set_property("REFLS_RMAX", "15");
        session.set_property("REFLS_RMIN", "0.5");
        session.set_property("REFLS_THRESH", "15");

      }
      // set mode 0=LongRange, 1=HiSpeed, 2=HiSpeedReflectors
      ostringstream temp;
      temp << mode; 
 
      session.execute_command("MEAS_SET_PROG", temp.str(), &result, &status, &comment );
      logStatus();
    } catch(exception& e)
    {
      ROS_ERROR("Error occured when attempting to connect to VZ-400: %s",e.what() );
      connected = false;
    }
//  } while(!connected);
//

    // start scanner in 0 position
    session.execute_command("SCN_ALIGN_PHI", "0,1", &result, &status, &comment );

    if (!connected) {
      ROS_ERROR("No Connection, exitting!" );
      exit(1);
    }
}

const char* RieglCtrl::ScanParamString(string append) {
  ostringstream temp;
  temp << minlineangle << "," << maxlineangle << "," << lineangle << ",";
  temp << minframeangle << "," << maxframeangle << "," << frameangle << append;
  //res = res + (string)minlineangle + "," + maxlineangle + "," + lineangle + ",";
  //res = res + minframeangle + "," + maxframangle + "," + frameangle;
  return temp.str().c_str();
}

void RieglCtrl::logStatus() {
  for(size_t i=0; i<comment.size(); ++i)
    ROS_DEBUG("%s",comment[i].c_str());
  ROS_DEBUG("Status is %s", status.c_str() );
}


void RieglCtrl::advertise() {
  servstartMeasuring = n.advertiseService("startMeasuring", &RieglCtrl::startMeasuring, this);
  servstopMeasuring = n.advertiseService("stopMeasuring", &RieglCtrl::stopMeasuring, this);
  servsetMeasuring = n.advertiseService("setMeasuring", &RieglCtrl::setMeasuring, this);
  servsetContinuous = n.advertiseService("setContinuous", &RieglCtrl::setContinuous, this);
  servsetSingle = n.advertiseService("setSingle", &RieglCtrl::setSingle, this);
  servsetParams = n.advertiseService("setParams", &RieglCtrl::setParams, this);
  servsetLineAngle = n.advertiseService("setLineAngle", &RieglCtrl::setLineAngle, this);
  servsetFrameAngle = n.advertiseService("setFrameAngle", &RieglCtrl::setFrameAngle, this);
  servsetMaxLineAngle = n.advertiseService("setMaxLineAngle", &RieglCtrl::setMaxLineAngle, this);
  servsetMinLineAngle = n.advertiseService("setMinLineAngle", &RieglCtrl::setMinLineAngle, this);
  servsetMaxFrameAngle = n.advertiseService("setMaxFrameAngle", &RieglCtrl::setMaxFrameAngle, this);
  servsetMinFrameAngle = n.advertiseService("setMinFrameAngle", &RieglCtrl::setMinFrameAngle, this);
  servsetMeasurementType = n.advertiseService("setMeasurementType", &RieglCtrl::setMeasurementType, this);
  servgetProgress = n.advertiseService("getProgress", &RieglCtrl::getProgress, this);
  servgetInclination = n.advertiseService("getInclination", &RieglCtrl::getInclination, this);
  servalignScanner = n.advertiseService("alignScanner", &RieglCtrl::alignScanner, this);
  servReflSaS = n.advertiseService("reflSaS", &RieglCtrl::startReflSaS, this);
  
  servShutdown = n.advertiseService("Shutdown", &RieglCtrl::Shutdown, this);


}
      
bool RieglCtrl::Shutdown(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response) {
  try {
    session.execute_command("SHUTDOWN", "", &result, &status, &comment );
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command SHUTDOWN :%s", e.what() );
    return false;
  }
  return true;

}

bool RieglCtrl::alignScanner(riegl::angle::Request& request, riegl::angle::Request& response)
{
  try {
    char buffer[64];
    sprintf(buffer, "%lf,1", request.angle);
    session.execute_command("SCN_ALIGN_PHI", buffer, &result, &status, &comment );
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command SCN_ALIGN_PHI :%s", e.what() );
    return false;
  }
  return true;
}

bool RieglCtrl::startMeasuring(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response)
{
  try {
    session.execute_command("MEAS_START", "", &result, &status, &comment );
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command MEAS_START :%s", e.what() );
    return false;
  }
  return true;
}
bool RieglCtrl::stopMeasuring(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response)
{
  try {
    session.execute_command("MEAS_ABORT", "", &result, &status, &comment);
    logStatus();

    // now wait for the scanner to abort a scan in progress:
    session.execute_command("MEAS_BUSY", "1", &result, &status, &comment);
    logStatus();
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command MEAS_ABORT :%s", e.what() );
    return false;
  }
  return true;
}
bool RieglCtrl::startReflSaS(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response)
{
  try {
    session.execute_command("REFLSAS_START", "1", &result, &status, &comment );
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command MEAS_START :%s", e.what() );
    return false;
  }
  return true;
}
bool RieglCtrl::setMeasuring(riegl::Command::Request& request, riegl::Command::Request& response)
{
  try {
    session.execute_command(request.command.c_str(), request.params, &result, &status, &comment );
    logStatus();
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command setMeasuring :%s", e.what() );
    return false;
  }
  return true;
}
bool RieglCtrl::setContinuous(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response)
{
  try {
   session.execute_command("SCN_SET_RECT_FOV_SEQ", ScanParamString(",0"), &result, &status, &comment );
   logStatus();
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command SCN_SET_RECT_FOV_SEQ :%s", e.what() );
    return false;
  }
  return true;
}
bool RieglCtrl::setSingle(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response)
{
  try {
    //ROS_ERROR(ScanParamString());
    session.execute_command("SCN_SET_RECT_FOV", ScanParamString() , &result, &status, &comment );
    logStatus();
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command SCN_SET_RECT_FOV :%s", e.what() );
    return false;
  }
  return true;
}
bool RieglCtrl::setParams(riegl::scanparams::Request& request, riegl::scanparams::Request& response)
{
  lineangle = request.lineangle;
  frameangle = request.frameangle;
  maxlineangle = request.maxlineangle;
  minlineangle = request.minlineangle;
  maxframeangle = request.maxframeangle;
  minframeangle = request.minframeangle;
  return true;
}
bool RieglCtrl::setLineAngle(riegl::angle::Request& request, riegl::angle::Request& response)
{
  lineangle = request.angle;
  return true;
}
bool RieglCtrl::setFrameAngle(riegl::angle::Request& request, riegl::angle::Request& response)
{
  frameangle = request.angle;
  return true;
}
bool RieglCtrl::setMaxLineAngle(riegl::angle::Request& request, riegl::angle::Request& response)
{
  maxlineangle = request.angle;
  return true;
}
bool RieglCtrl::setMinLineAngle(riegl::angle::Request& request, riegl::angle::Request& response)
{
  minlineangle = request.angle;
  return true;
}
bool RieglCtrl::setMaxFrameAngle(riegl::angle::Request& request, riegl::angle::Request& response)
{
  maxframeangle = request.angle;
  return true;
}
bool RieglCtrl::setMinFrameAngle(riegl::angle::Request& request, riegl::angle::Request& response)
{
  minframeangle = request.angle;
  return true;
}

bool RieglCtrl::setMeasurementType(riegl::selection::Request& request, riegl::selection::Request& response)
{
  ostringstream temp;
  temp << request.type; 
  try {
    session.execute_command("MEAS_SET_PROG", temp.str(), &result, &status, &comment );
    logStatus();
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command MEAS_SET_PROG :%s", e.what() );
    return false;
  }
  return true;
}

bool RieglCtrl::getProgress(riegl::progress::Request& request, riegl::progress::Response& response)
{
  try {
    session.execute_command("MSCN_PROGRESS", "", &result, &status, &comment );
    logStatus();
    stringstream temp(result);
    temp >> response.progress;
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command MSCN_PROGRESS :%s", e.what() );
    return false;
  }
  return true;
}
      

bool RieglCtrl::getInclination(riegl::inclination::Request& request, riegl::inclination::Response& response) {
  try {
    session.execute_command("INCL_ACQUIRE", "1", &result, &status, &comment );
    logStatus();
    stringstream temp(result);
    temp >> response.roll;
    char t; temp >> t;
    temp >> response.pitch;
  } catch(exception& e)
  {
    ROS_ERROR("Could not execute command MSCN_PROGRESS :%s", e.what() );
    return false;
  }
  return true;
}


}



int main(int argc, char* argv[])
{
  ros::init(argc, argv, "RieglCtrl");
  ros::NodeHandle n;
  std::string device;
  n.param<std::string>("/riegl/ip", device, "192.168.0.125");
  double la, fa, minla, minfa, maxla, maxfa;
  n.param("/riegl/fov/min_line", minla, 30.0);
  n.param("/riegl/fov/max_line", maxla, 130.0);
  n.param("/riegl/fov/min_frame", minfa, 0.0);
  n.param("/riegl/fov/max_frame", maxfa, 360.0);

  n.param("/riegl/resolution/frame", fa, 1.0);
  n.param("/riegl/resolution/line", la, 1.0);

  int mode;
  n.param("/riegl/mode", mode, 1);

  riegl::RieglCtrl ctrl(device.c_str(), minla, la, maxla, minfa, fa, maxfa, mode);
  ros::spin();

  return 0;

}


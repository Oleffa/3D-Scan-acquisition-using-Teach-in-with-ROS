#ifndef RIEGLCTRL_H
#define RIEGLCTRL_H

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

#include <riegl/ctrllib.hpp>
#include <riegl/scanlib.hpp>

#include "std_srvs/Empty.h"
#include "riegl/angle.h"
#include "riegl/scanparams.h"
#include "riegl/Command.h"
#include "riegl/selection.h"
#include "riegl/progress.h"
#include "riegl/inclination.h"

using namespace ctrllib;
using namespace scanlib;
using namespace std;
using namespace std::tr1;


namespace riegl {

  class RieglCtrl {

    private:
      ctrl_client_session session;
      ros::NodeHandle n;
      ros::ServiceServer servstartMeasuring;
      ros::ServiceServer servstopMeasuring;
      ros::ServiceServer servsetMeasuring;
      ros::ServiceServer servsetContinuous;
      ros::ServiceServer servsetSingle;
      ros::ServiceServer servsetLineScan;
      ros::ServiceServer servsetParams;
      ros::ServiceServer servsetLineAngle;
      ros::ServiceServer servsetFrameAngle;
      ros::ServiceServer servsetMaxLineAngle;
      ros::ServiceServer servsetMinLineAngle;
      ros::ServiceServer servsetMaxFrameAngle;
      ros::ServiceServer servsetMinFrameAngle;
      
      
      ros::ServiceServer servShutdown;
      ros::ServiceServer servsetMeasurementType;
      ros::ServiceServer servgetProgress;
      ros::ServiceServer servgetInclination;
      ros::ServiceServer servalignScanner;
      ros::ServiceServer servReflSaS;
        
      std::string result;               // will receive result of the function
      std::vector<std::string> comment; // optional messages from the instrument
      std::string status;               // will receive the instrument's status


      double lineangle;
      double frameangle;
      double maxlineangle;
      double minlineangle;
      double maxframeangle; 
      double minframeangle;
      int mode;
      bool connected;

      const char* ScanParamString(string append="");
      const char* ScanParamStringLineScan(string append="");
      void logStatus();
      void advertise(); 

    public:
      ~RieglCtrl();

      //RieglCtrl(string address = string("192.168.0.234"));
      RieglCtrl(string address = string("10.0.0.1"), 
          double minlineangle=30.0, double lineangle=1.0, 
          double maxlineangle=130.0, double minframeangle=0.0, 
          double frameangle=1.0, double maxframeangle=360.0,
          int mode=1);

      bool startMeasuring(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response);
      bool stopMeasuring(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response);
      bool setMeasuring(riegl::Command::Request& request, riegl::Command::Request& response);
      bool setContinuous(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response);
      bool setSingle(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response);
      bool setLineScan(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response);
      bool setParams(riegl::scanparams::Request& request, riegl::scanparams::Request& response);
      bool setLineAngle(riegl::angle::Request& request, riegl::angle::Request& response);
      bool setFrameAngle(riegl::angle::Request& request, riegl::angle::Request& response);
      bool setMaxLineAngle(riegl::angle::Request& request, riegl::angle::Request& response);
      bool setMinLineAngle(riegl::angle::Request& request, riegl::angle::Request& response);
      bool setMaxFrameAngle(riegl::angle::Request& request, riegl::angle::Request& response);
      bool setMinFrameAngle(riegl::angle::Request& request, riegl::angle::Request& response);
      bool startReflSaS(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response);
      
      bool alignScanner(riegl::angle::Request& request, riegl::angle::Request& response);
      bool Shutdown(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response);
      bool setMeasurementType(riegl::selection::Request& request, riegl::selection::Request& response);
      bool getProgress(riegl::progress::Request& request, riegl::progress::Response& response);
      bool getInclination(riegl::inclination::Request& request, riegl::inclination::Response& response);
  };
}
#endif

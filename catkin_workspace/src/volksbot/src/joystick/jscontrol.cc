#include "firejet.h"
#include "predator.h"
#include "logitechf710.h"
#include <string.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "joystick_control");
  ros::NodeHandle n;
  std::string device;
  n.param<std::string>("/devices/joystick", device, "/dev/input/js0");
	
	std::cout<<"main"<<std::endl;
  
  Joystick *js;
  int type;
  if ( n.getParam("/control/joystick/logitechf710", type) && type) {
    js = new LogitechF(device.c_str());
  } else if ( n.getParam("/control/joystick/predator", type) && type) {
    js = new Predator(device.c_str());
  } else if ( n.getParam("/control/joystick/firejet", type) && type) {
    js = new Firejet(device.c_str());
  } else {
    ROS_ERROR("No joystick type specified!!!\n Aborting joystick-control.\n");
    js = new Predator(device.c_str());
    //return 0;
  }

   // js->waitforevents();
    
  while (ros::ok())
  {
    js->waitforevent();
  }

  delete js;
	return 0;
}

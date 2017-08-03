#include "logitechf710.h"

// service
#include "volksbot/velocities.h"
#include "volksbot/vels.h"

#include "std_srvs/Empty.h"
#include <math.h>

#include "ros/ros.h"

//Custom
#include <iostream>
#include "bachelorarbeit/AddTwoInts.h"

void LogitechF::handleButton(uint8_t number, bool pressed, uint32_t time) {
  std_srvs::Empty e;
  bachelorarbeit::AddTwoInts srv;
  ros::ServiceClient client = n.serviceClient<bachelorarbeit::AddTwoInts>("WPhandler_setWP");
  switch(number) {
    case BUTTON_A:
	std::cout<<"Button A pressed! - setSingle"<<std::endl;
      //ros::service::call("startMeasuring",e);
	ros::service::call("setSingle",e);
      break;
    case BUTTON_B:
	std::cout<<"Button B pressed!"<<std::endl;
      ros::service::call("stopMeasuring",e);
      break;
    case BUTTON_X:
	std::cout<<"Button X pressed!"<<std::endl; 
      //ros::service::call("setSingle",e);
      //ros::service::call("Shutdown",e);
	srv.request.a = 1;
	client.call(srv);
	ros::service::call("startMeasuring",e);
      break;
    case BUTTON_Y:
	std::cout<<"Button Y pressed!"<<std::endl;
      ros::service::call("setContinuous",e);
      break;
    case BUTTON_LEFT:
      if (pressed) {
	std::cout<<"Top Left pressed!"<<std::endl;
      speed = fmax(0.0, speed-10.0);
	std::cout<<speed<<std::endl;
      sendSpeed();
      }
      break;
    case BUTTON_RIGHT:
	std::cout<<"Top Right pressed!"<<std::endl;
      if (pressed) {
      speed = fmin(100.0, speed+10.0);
	std::cout << speed << std::endl ;
      sendSpeed();
      }
      break;
    case START:
      if(pressed) {
        //ROS_ERROR("START THERMO SCAN");
	std::cout<<"START pressed!"<<std::endl;
        ros::service::call("startImageScan",e);
      }
      break;
    case LOGITECH:
	std::cout<<"LOGITECH pressed!"<<std::endl;
      break;
    case BACK:
	if(pressed){
	std::cout<<"Back pressed!\nTODO: Wegpunkt setzen"<<std::endl;
	}
      break;
    default:
      break;
  }
}

void LogitechF::handleAxis(uint8_t number, int16_t value, uint32_t time) {
	//	std::cout<<"handleAxis"<<std::endl;
  switch(number) {
    case LSTICK_LEFTRIGHT:
	std::cout<<"LSTICK_LEFTRIGHT"<<std::endl;
      break;
    case LSTICK_UPDOWN:
	std::cout<<"LSTICK_UPDOWN"<<std::endl;
      if (fabs(value) > STICK_MIN_ACTIVITY) {
        rightvel = ((double)value)/(double)JS_MAX_VALUE;
      } else {
        rightvel = 0;
      }
      sendSpeed();
      break;
    case RSTICK_LEFTRIGHT:
	std::cout<<"RSTICK_LEFTRIGHT"<<std::endl;
      break;
    case RSTICK_UPDOWN:
	std::cout<<"RSTICK_UPDOWN"<<std::endl;
      if (fabs(value) > STICK_MIN_ACTIVITY) {
        leftvel = ((double)value)/(double)JS_MAX_VALUE;
      } else {
        leftvel = 0;
      }
      sendSpeed();
      break;
    case THROTTLE_LEFT:
	std::cout<<"Throttle_left"<<std::endl;
      break;
    case THROTTLE_RIGHT:
	std::cout<<"Throttle_right"<<std::endl;
      break;
    case HUD_UPDOWN: 
	std::cout<<"HUD_UPDOWN"<<std::endl;
      /*
      if (value > 0) { // down
        leftvel = speed; 
        rightvel = speed; 
      } else if (value < 0) {  //up
        leftvel = -speed; 
        rightvel = -speed; 
      } else {
        leftvel = 0; 
        rightvel = 0; 
      }
      sendSpeed(false);*/
      break;
    case HUD_LEFTRIGHT:
	std::cout<<"HUD_LEFTRIGHT"<<std::endl;
      /*
      if (value > 0) { // right
        leftvel = speed; 
        rightvel = -speed; 
      } else if (value < 0) {  //left
        leftvel = -speed; 
        rightvel = speed; 
      } else {
        leftvel = 0; 
        rightvel = 0; 
      }
      sendSpeed(false);
      */
      break;
    default:
      break;
  }
}


void LogitechF::sendSpeed() {
  volksbot::vels velocity;
  velocity.left = leftvel * speed;
  velocity.right = rightvel * speed;
  std::printf("%f %f SPEED %f \n", leftvel, rightvel, speed);
  publisher.publish(velocity);
}

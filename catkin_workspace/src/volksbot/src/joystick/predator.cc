#include "predator.h"

// service
#include "volksbot/velocities.h"
#include "volksbot/vels.h"

#include "std_srvs/Empty.h"
#include <math.h>

void Predator::handleButton(uint8_t number, bool pressed, uint32_t time) {
  std_srvs::Empty e;
  switch(number) {
    case TRIANGLE:
      ros::service::call("stopMeasuring",e);
      break;
    case CIRCLE:
      ros::service::call("setContinuous",e);
      break;
    case CROSS: 
      ros::service::call("Shutdown",e);
      break;
    case SQUARE:
      ros::service::call("setSingle",e);
      break;
    case LEFT2:
      if (pressed) {
      speed = fmax(0.0, speed-10.0);
      sendSpeed();
      }
      break;
    case RIGHT2:
      break;
    case LEFT1:
      if (pressed) {
      speed = fmin(100.0, speed+10.0);
      sendSpeed();
      }
      break;
    case RIGHT1:
      break;
    case SELECT:
      if(pressed) {
        ros::service::call("startThermoScan",e);
      }
      break;
    case START:
      ros::service::call("startMeasuring",e);
      break;
    default:
      break;
  }
}

void Predator::handleAxis(uint8_t number, int16_t value, uint32_t time) {
  switch(number) {
    case LEFTUPDOWN:{
      rightvel = ((double)value)/(double)JS_MAX_VALUE;
      sendSpeed();
      break;}
    case LEFTLEFTRIGHT:
      break;
    case RIGHTUPDOWN:{
      leftvel = ((double)value)/(double)JS_MAX_VALUE;
      sendSpeed();
      break;}
    case RIGHTLEFTRIGHT:
      break;
    case HUDUPDOWN: 
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
      sendSpeed();
      break;
    case HUDLEFTRIGHT:
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
      sendSpeed();
      break;
    default:
      break;
  }
}


void Predator::sendSpeed() {
  volksbot::vels velocity;
  velocity.left = leftvel * speed;
  velocity.right = rightvel * speed;
  ROS_INFO("%f %f SPEED %f \n", leftvel, rightvel, speed);
  publisher.publish(velocity);
}

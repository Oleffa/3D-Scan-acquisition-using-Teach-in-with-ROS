#include "firejet.h"

// service
#include "volksbot/velocities.h"
#include "volksbot/vels.h"

#include "std_srvs/Empty.h"

#define EPS 0.0000001

void Firejet::handleButton(uint8_t number, bool pressed, uint32_t time) {
  std_srvs::Empty e;
  switch(number) {
    case BUTTON1:
      ros::service::call("startMeasuring",e);
      break;
    case BUTTON2:
      break;
    case BUTTON3: 
      break;
    case BUTTON4:
      break;
    case BUTTON5:
      ros::service::call("Shutdown",e);
      break;
    case BUTTON6:
      ros::service::call("stopMeasuring",e);
      break;
    case BUTTON7:
      break;
    case BUTTON8:
      break;
    case BUTTON9:
      ros::service::call("setSingle",e);
      break;
    case BUTTON10:
      break;
    case BUTTON11:
      break;
    case BUTTON12:
      ros::service::call("setContinuous",e);
      break;
    default:
      break;
  }
}

void Firejet::handleAxis(uint8_t number, int16_t value, uint32_t time) {
  switch(number) {
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
    case SPEEDO:{
      speed = 50.0*((double)value)/JS_MIN_VALUE + 50.0;
      setSpeed();
      break;} 
    case JSUPDOWN:{
      sticky = ((double)value)/JS_MIN_VALUE;
      setSpeed();
      break;}
    case JSLEFTRIGHT:{
      stickx = ((double)value)/JS_MAX_VALUE;
      setSpeed();
      break;}
    case JSROLL:
      break;
    default:
      break;
  }
}

/**
 * Computes velocites of the wheels from current position of the stick and
 * throttle. Stick position is given as if it moves within a square, so we 
 * need to map the given values to a circle using polar coordinates.
 *
 */
void Firejet::setSpeed() {
  double angle = atan2(sticky, stickx);
  if (fabs(stickx) < EPS && fabs(sticky) < EPS) {
    leftvel = 0;
    rightvel = 0;
    sendSpeed();
    return;
  }
  double max;

  if (fabs(stickx) > fabs(sticky)) {
    max = sticky/fabs(stickx);
  } else {
    max = stickx/fabs(sticky);
  }

  double distToCenter = sqrt( stickx*stickx + sticky*sticky) / sqrt(max*max + 1);
  // front left 
  if (angle > M_PI/2.0) {
    rightvel = (angle - (M_PI * 3.0/4.0)) * -4.0/M_PI;
    leftvel = 1.0;
  // front right
  } else if (angle > 0) {
    rightvel = 1.0;
    leftvel = (angle - (M_PI/4.0)) * 4.0/M_PI;
  // rear left
  } else if (angle < -M_PI/2.0) {
    rightvel = -1.0;
    leftvel = (angle + (M_PI * 3.0/4.0)) * -4.0/M_PI;
  } else {
    rightvel = (angle + (M_PI/4.0)) * 4.0/M_PI;
    leftvel = -1.0;
  }

  rightvel = -rightvel * distToCenter * speed;
  leftvel = -leftvel * distToCenter * speed;
  sendSpeed();
}

/**
 * Sends commands to the subcribers (robot)
 */
void Firejet::sendSpeed() {
  volksbot::vels velocity;
  velocity.left = leftvel;
  velocity.right = rightvel;
  publisher.publish(velocity);
}

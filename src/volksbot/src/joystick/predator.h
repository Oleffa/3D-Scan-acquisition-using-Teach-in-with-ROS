#ifndef PREDATOR_HH
#define PREDATOR_HH

#include "joystick.h"
#include <ros/ros.h>
#include "volksbot/vels.h"
#include <std_msgs/String.h>

#define TRIANGLE 0x00
#define CIRCLE   0x01
#define CROSS    0x02
#define SQUARE   0x03
#define LEFT2    0x04
#define RIGHT2   0x05
#define LEFT1    0x06
#define RIGHT1   0x07
#define SELECT   0x08
#define START    0x09

#define LEFTLEFTRIGHT  0x00
#define LEFTUPDOWN     0x01
#define RIGHTUPDOWN    0x03
#define RIGHTLEFTRIGHT 0x04
#define HUDUPDOWN      0x06
#define HUDLEFTRIGHT   0x05

class Predator : public Joystick {
  public:

    Predator() : Joystick() { init(); };
    Predator(const char* fn) : Joystick(fn) { init(); };

    virtual void handleButton(uint8_t number, bool pressed, uint32_t time);
    virtual void handleAxis(uint8_t number, int16_t value, uint32_t time);

    inline double getLeftVel() {return leftvel;}
    inline double getRightVel() {return rightvel;}

  private:
    ros::NodeHandle n;
    ros::Publisher publisher;


    void sendSpeed();

    inline void init() {
      speed = 20.0;
      rightvel = leftvel = 0.0;
      publisher = n.advertise<volksbot::vels>("Vel", 100);
    }

    double leftvel, rightvel;
    double speed;

};

#endif

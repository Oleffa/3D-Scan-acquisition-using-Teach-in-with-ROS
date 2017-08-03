#ifndef FIREJET_HH
#define FIREJET_HH

#include "joystick.h"
#include <ros/ros.h>
#include "volksbot/vels.h"
#include <std_msgs/String.h>

#define BUTTON1   0x00
#define BUTTON2   0x01
#define BUTTON3   0x02
#define BUTTON4   0x03
#define BUTTON5   0x04
#define BUTTON6   0x05
#define BUTTON7   0x06
#define BUTTON8   0x07
#define BUTTON9   0x08
#define BUTTON10  0x09
#define BUTTON11  0x0A
#define BUTTON12  0x0B

#define HUDUPDOWN     0x06
#define HUDLEFTRIGHT  0x05
#define SPEEDO        0x02
#define JSUPDOWN      0x01
#define JSLEFTRIGHT   0x00
#define JSROLL        0x03

class Firejet : public Joystick {
  public:

    Firejet() : Joystick() { init(); };
    Firejet(const char* fn) : Joystick(fn) { init(); };

    virtual void handleButton(uint8_t number, bool pressed, uint32_t time);
    virtual void handleAxis(uint8_t number, int16_t value, uint32_t time);

    inline double getLeftVel() {return leftvel;}
    inline double getRightVel() {return rightvel;}

  private:
    ros::NodeHandle n;
    ros::Publisher publisher;


    void setSpeed();
    void sendSpeed();

    inline void init() {
      stickx = sticky = speed = 0.0;
      publisher = n.advertise<volksbot::vels>("Vel", 100);
      /*
      gnuplot = popen("gnuplot -persist", "w");
      fprintf(gnuplot, "set xrange[-1:1]\n");
      fprintf(gnuplot, "set yrange[-1:1]\n");
      fflush(gnuplot);*/
    }/*
    FILE *gnuplot;*/

    double leftvel, rightvel;
    double speed;
    double stickx, sticky;

};

#endif

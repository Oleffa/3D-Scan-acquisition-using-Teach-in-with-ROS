#include <ros/ros.h>
#include "stdio.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include "sys/time.h"

#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdexcept>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_A 0x61
#define KEYCODE_Y 0x79

// service
#include "volksbot/velocities.h"

#include "kbcontrol.h"

namespace volksbot {

kbcontrol::kbcontrol() {
  velocity.request.left = 0;
  velocity.request.right = 0;
  speed = 0.0;
  kfd = 0;
}

void kbcontrol::setVelocity(char c) {
    switch(c)
    {
      case KEYCODE_L:
        velocity.request.left = -speed;
        velocity.request.right = speed;
        break;
      case KEYCODE_R:
        velocity.request.left = speed;
        velocity.request.right = -speed;
        break;
      case KEYCODE_U:
        velocity.request.left = -speed;
        velocity.request.right = -speed;
        break;
      case KEYCODE_D:
        velocity.request.left = speed;
        velocity.request.right = speed;
        break;
      case KEYCODE_Q:
        speed = 0;
        velocity.request.left = speed;
        velocity.request.right = speed;
        break;
    }
}

void kbcontrol::run() {
  char c,previous;
  previous = KEYCODE_Q;
  struct termios cooked;
  tcgetattr(kfd, &cooked);
  cooked.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  cooked.c_cc[VEOL] = 1;
  cooked.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &cooked);
  
  ros::Rate loop_rate(5);

  while (ros::ok() ) {
    c = KEYCODE_Q;
    read(kfd, &c, 1);
    
    switch(c)
    {
      case KEYCODE_A:
        speed += 10;
        if (speed > 100.0) speed = 100.0;
        setVelocity(previous);
        break;
      case KEYCODE_Y:
        speed *= 0.9;
        setVelocity(previous);
        break;
      default:
        previous = c;
        break;
    }
    setVelocity(c);
    printf("Call servic: %f %f\n", velocity.request.left, velocity.request.right);
    ros::service::call("Controls", velocity);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "keyboard_control");
  volksbot::kbcontrol controller;
  controller.run();
	return 0;
}

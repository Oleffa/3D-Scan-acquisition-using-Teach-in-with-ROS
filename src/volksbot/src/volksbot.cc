#include <ros/ros.h>
#include "stdio.h"
#include "CVmc.h"

#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include "sys/time.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_A 0x61
#define KEYCODE_Y 0x79

#include <unistd.h>
#include <fcntl.h>
#include <stdexcept>


#include <sys/mman.h>

VMC::CVmc *vmc;

void quit(int sig)
{
  exit(0);
}

int main(int argc, char* argv[])
{
	mlockall(MCL_CURRENT | MCL_FUTURE);
  printf("Ros init...\n");
  ros::init(argc, argv, "VMC_Module");
  ros::NodeHandle n;
  std::string device;
//  n.param<std::string>("/devices/volksbot", device, "rtser1");
  n.param<std::string>("/devices/volksbot", device, "/dev/ttyS5");

	vmc = new VMC::CVmc(device.c_str());
	
  if(!vmc->isConnected()) {
    printf("Could not connect\n");
    delete vmc;
    return 1;
  }
  signal(SIGINT,quit);
  ros::spin();

  delete vmc;
	return 0;
}

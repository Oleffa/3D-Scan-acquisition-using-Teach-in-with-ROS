#include <ros/ros.h>

// service
#include "volksbot/velocities.h"

namespace volksbot {

  class kbcontrol {
    private:

      ros::NodeHandle n;
      ros::ServiceClient client;
      volksbot::velocities velocity;
      double speed;
      int kfd;

      void setVelocity(char c);

    public:
      kbcontrol();
      void run();
  };
}

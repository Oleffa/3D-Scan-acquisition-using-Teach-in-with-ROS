#include "volksbot/odometry.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Odometry");

  bool publish_tf = false;

  if(argc > 1) {
    publish_tf = atoi(argv[1]);
  }

  volksbot::Odometry odo(publish_tf);
  ros::spin();
  //odo.update(70);

	return 0;
}

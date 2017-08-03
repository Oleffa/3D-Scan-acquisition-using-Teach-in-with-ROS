#include <rosgraph_msgs/Clock.h>
#include <ros/ros.h>

struct timespec tp;

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
  tp.tv_sec = msg->clock.sec;
  tp.tv_nsec = msg->clock.nsec;

  clock_settime(CLOCK_REALTIME, &tp);
}



int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ClockPublisher");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("IrmaClock", 10, clockCallback);

  ros::spin();

  // Node was killed -> reset system time
  if (!system("ntpdate -u  ptbtime1.ptb.de\n") ) {
    ROS_ERROR("Could not reset system clock!!!");
  }
  return 0;
}       



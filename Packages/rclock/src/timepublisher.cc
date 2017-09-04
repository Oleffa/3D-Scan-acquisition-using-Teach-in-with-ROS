#include <rosgraph_msgs/Clock.h>
#include <ros/ros.h>

#include "rclock/logDir.h"

#include <string>
using std::string;

#include <sys/stat.h>

string logdir;

string getTime() {
  time_t rawtime;
  struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );

  char date[25];
  sprintf (date, "%04d_%02d_%02d_%02d_%02d_%02d", (1900+timeinfo->tm_year), (1+timeinfo->tm_mon), timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec );
  return string(date);
}

bool callback(rclock::logDir::Request& empty, rclock::logDir::Response& directory) {
  directory.directory = logdir;
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ClockSetter");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<rosgraph_msgs::Clock>("IrmaClock", 10);
  
  logdir = getTime();
  string basedir;
  n.param<std::string>("/log/basedir", basedir, "/tmp/dat/");
  basedir = basedir + logdir + "/"; // /tmp/dat/YY_MM_DD_HH_MM_SS/
  mkdir(basedir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
  logdir = basedir;
  ros::ServiceServer service = n.advertiseService("logDirectory", callback);

  rosgraph_msgs::Clock clock;
  
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    clock.clock = ros::Time::now();

    pub.publish(clock);

    ros::spinOnce();
    loop_rate.sleep();
  }      
          
  return 0;
}       


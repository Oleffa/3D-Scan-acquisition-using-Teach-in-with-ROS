#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <stdlib.h>

#include <stdio.h>
FILE * gnup ;

void chatterCallback(const sensor_msgs::LaserScanConstPtr& scan )
{
    ROS_INFO("Received scan\n");
    FILE *dat = fopen("/tmp/dat.txt","w");
    for (unsigned int i = 0; i < scan->ranges.size(); i++) {
      fprintf(dat, "%f\n", scan->ranges[i]  );
    }
    
    fprintf(gnup, "plot '/tmp/dat.txt' u 0:1\n");
    fflush(gnup);
}
int main(int argc, char** argv)
{
  gnup = popen("gnuplot","w");
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber chatter_sub = n.subscribe("LMS100", 1, chatterCallback);
  ros::spin();

  pclose(gnup);
}


#include <ros/ros.h>
#include <string.h>
#include "sensor_msgs/LaserScan.h"
#include "lms100_cola.h"
#include "stdio.h"

template <class T>
inline T rad(const T deg)
{
    return ( (2 * M_PI * deg) / 360 );
}



int main(int argc, char* argv[])
{
  ROS_INFO("Starting sick LMS node!\n" );
  ros::init(argc, argv, "SICK_sensor");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<sensor_msgs::LaserScan>("LMS",100);

  // Get parameters, or set default
  std::string hostname;
  n.param<std::string>("sick/ip", hostname, "192.168.0.2");
  
  int port;
  n.param("sick/port", port, 2111);
 
  double min, max, res, freq;
  n.param("sick/fov/min", min, -45.0);
  n.param("sick/fov/max", max, 225.0);
  n.param("sick/fov/resolution", res, 0.5);
  n.param("sick/rate", freq, 50.0);

  // connect and setup
  lms100_cola *lms = new lms100_cola(hostname.c_str(), port, 0);

  do {
    ROS_INFO("Attempting to connect to device!\n" );
    if ( lms->Connect() ) {  
      ROS_INFO("Error occured when attempting to connect to LMS100!\n");
      ROS_INFO("waiting... \n");
      usleep(500000);
    } else {
      break;
    }
  } while (true);
  ROS_INFO("Connected...\n" );

  lms->wait();

  int err = lms->SetResolutionAndFrequency(freq, res, min, max);

  if (err) {
    ROS_INFO("Error %d occured while setting FOV!\n", err);
  }

  ROS_INFO("MEASURING while setting FOV!\n" );

  // Start measuring, setup the laser message
  int nr_ranges = lms->getNRMeas(); 
  double *ranges;
  sensor_msgs::LaserScan laserMsg; 
  laserMsg.angle_min = rad(min-90.0);
  laserMsg.angle_max = rad(max-90.0);
  laserMsg.angle_increment = rad(res);
  laserMsg.time_increment  = 1/(freq*nr_ranges);
  laserMsg.scan_time       = 1.0/freq;
  laserMsg.range_min = 0.05;
  laserMsg.range_max = 20.0;
  laserMsg.ranges.resize(nr_ranges );
  laserMsg.intensities.resize(0);
  ros::Rate loop_rate(freq);

  lms->MainSetup(); 

  int counter = 0;
    
  // measure...
  lms->ReadResult();
  while(n.ok())
  {
    if(lms->ReadResult() == 0){
      lms->ParseResult();
      ranges = lms->getMeas();
      laserMsg.header.frame_id = "/front_laser"; // no timestamp data at the moment TODO
      laserMsg.header.stamp = ros::Time::now();

     
      // check whether any measurements are above or below threshold. shouldn't happen but check anyway
      //  TODO remove if unnecessary
      for (int i = 0; i < nr_ranges; i++) {
    /*    if (ranges[i] > 20.0) {
          laserMsg.ranges[i] = 20.0;
          ROS_ERROR("too large measurement in SICK !!!");
        } else if (ranges[i] < 0.05) {
          laserMsg.ranges[i] = 0.05;
//          ROS_ERROR("too small measurement in SICK !!!");
        } else {*/
          laserMsg.ranges[i] = ranges[i];
        //}
      }      
      counter++;
      if (counter == 5) {
      //ROS_ERROR("%d ", nr_ranges);
        publisher.publish(laserMsg);
        counter = 0;
      }
      
    }
    //      ROS_ERROR("too small measurement in SICK !!!");

    ros::spinOnce();
    loop_rate.sleep();
  }

  delete lms;
	return 0;
}


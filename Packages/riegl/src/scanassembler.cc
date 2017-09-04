#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Empty.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

#include "riegl/RieglStatus.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <queue>
using std::queue;
#include <vector>
using std::vector;
#include <sstream>
using std::stringstream;
using std::string;

#include <pthread.h>

#include <rclock/logDir.h>


tf::TransformListener *listener;

class  scanstruct {
public:
  scanstruct() {
//    clouds.reserve(721);
    clouds.reserve(5000);
  }

  ~scanstruct() {
    for (unsigned int i = 0; i < clouds.size(); i++)
      delete clouds[i];
    clouds.clear();
  }

  vector<sensor_msgs::PointCloud *> clouds;
  ros::Time framestart;  // to get the startpose

  string fileName;
  unsigned int index;
};

scanstruct *current;
queue< scanstruct* > scanq;
vector<pthread_t*> threadlist;

template <class T>
inline std::string to_string(const T& t, int width)
{
  stringstream ss;
  ss << std::setfill('0') << std::setw(width) << t;
  return ss.str();
}

void statusCallback(const riegl::RieglStatusConstPtr& rs ) {
  if (rs->status == 1) {
    current->framestart = rs->header.stamp; 
    ROS_INFO("ASSEMBLER: received frame_start");
  } else if (rs->status == 2) { // frame stop
    ROS_INFO("ASSEMBLER: received frame_stop. Scan has %lu lines.", current->clouds.size());
    scanstruct *old = current;
    current = new scanstruct();
    scanq.push(old);
  }
}

void chatterCallback(const sensor_msgs::PointCloudConstPtr& scanline )
{
  sensor_msgs::PointCloud *transformedline = new sensor_msgs::PointCloud();

  // now transform
  try {    //listener->transformPoint(point.header.frame_id, ros::Time(0), point, "/odom", transformedpoint);

    listener->waitForTransform(scanline->header.frame_id, "/odom_combined", scanline->header.stamp, ros::Duration(0.1));
    listener->transformPointCloud("/odom_combined", *scanline, *transformedline);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  current->clouds.push_back(transformedline);
  
}


void *writeScan(void *_scan) {
  if (!_scan) return 0;
  
  std::ofstream o;     // file stream for scan
  scanstruct *scan = (scanstruct *)_scan;
  string scanFileName = scan->fileName + "scan"+ to_string(scan->index,3) + ".3d";
  string poseFileName = scan->fileName + "scan"+ to_string(scan->index,3) + ".pose";
  string framesFileName = scan->fileName + "scan"+ to_string(scan->index,3) + ".frames";
  
  int BSIZE = 5100;
  char buffer[BSIZE];
  char *pos = buffer;

  
  // open file
  ROS_INFO("opening scan %d \n", scan->index);
  o.open(scanFileName.c_str()); 
  o << std::setprecision(10);

  // write points using the buffer
  ROS_INFO("writing scan %d \n", scan->index);
  //      for (unsigned int i = 0; i < scan->points.size(); i++) {
  for (unsigned int i = 0; i < scan->clouds.size(); i++) {
    sensor_msgs::PointCloud *pc = scan->clouds[i];
    for (unsigned int j = 0; j < pc->points.size(); j++) {
      geometry_msgs::Point32 p = pc->points[j];
      sprintf(pos, "%16.8f %16.8f %16.8f\n", -100.0 * p.y, 100.0 * p.z, 100.0 * p.x); 
      pos += 51; // 16 + 1 + 16 + 1 + 16 + 1
      if (pos + 51 > buffer + BSIZE) // next write would overflow
      {
        o.write(buffer, (pos - buffer));
        pos = buffer;
      }
    }
  }
  o.close();

  // pose
  o.open(poseFileName.c_str()); 
  o << "0 0 0\n0 0 0" << std::endl;
  o.flush();
  o.close();
  
  // frames
  o.open(framesFileName.c_str()); 
  o << "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 2" << std::endl;
  o << "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 2" << std::endl;
  o << "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 2" << std::endl;
  o.flush();
  o.close();

  delete scan; 

  return 0;
}

int main(int argc, char** argv)
{
  // wait for tf to be ready
  sleep(5);

  ros::init(argc, argv, "assembler");
  ros::NodeHandle n;

  //tf::TransformListener l;
  tf::TransformListener l(ros::Duration(10000));
  listener = &l;
 
  // get configuration
  string filename;
  n.param<std::string>("/log/scandir", filename, "scan3d/");
 
  // wait for global logging directory to becom available
  ros::service::waitForService("logDirectory");
  rclock::logDir::Request empty;
  rclock::logDir::Response dir;
  // get loggin dir
  ros::service::call("logDirectory", empty, dir);

  filename = dir.directory + filename; // /tmp/dat/YY_MM_DD_HH_MM_SS/scan3d/

  // create directory
  mkdir(filename.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);

  current = new scanstruct();
  int index = 0;  //start writing with scan 0
  ros::Rate loop_rate(10);



  // start node
  ros::Subscriber chatter_sub = n.subscribe("riegl", 100, chatterCallback);
  ros::Subscriber status_sub = n.subscribe("rieglstatus", 100, statusCallback);

  while(ros::ok()) {
    if (!scanq.empty()) {
      scanstruct *scan = scanq.front();
      scanq.pop();
      
      if (!scan->clouds.empty()) {

        scan->fileName = filename; 
        scan->index = index;
        index++;
        ROS_INFO("Started THREAD for index %d", index);

        pthread_t *thread = new pthread_t();
        pthread_create( thread, NULL, writeScan, (void*)scan);
        threadlist.push_back(thread);
      }
    } 

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Terminated, waiting for scans to finish...");

  // ROS is done, clean up this node (wait for threads)
  for (unsigned int i = 0; i < threadlist.size(); i++) {
    pthread_join( *threadlist[i], NULL );
    delete threadlist[i];
    ROS_INFO("Scan %d is finished.", i);
  }
}


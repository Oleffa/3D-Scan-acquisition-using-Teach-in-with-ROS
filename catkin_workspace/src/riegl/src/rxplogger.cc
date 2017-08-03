#include <ros/ros.h>

#include <riegl/scanlib.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

#include <string.h>
#include <iostream>
#include <exception>
#include <cmath>
#include <limits>
// You might need to adjust the below include path, depending on your
// compiler setup and which tr1 implementation you are using.
#if defined(_MSC_VER)
#   include <memory>
#else
#   include <tr1/memory>
#endif

using namespace scanlib;
using namespace std;
using namespace std::tr1;


#include <rclock/logDir.h>

int main(int argc, char* argv[])
{
  int BUFLENGTH = 1024;
  char buf[BUFLENGTH];
  ros::init(argc, argv, "RXPLogger");
  ros::NodeHandle n;
  string ip, filename;
  n.param<std::string>("/riegl/ip", ip, "192.168.0.125");
  n.param<std::string>("/log/rxp", filename, "raw.rxp");

  // wait for global logging directory to become available
  ros::service::waitForService("logDirectory");
  // request path to logging dir
  rclock::logDir::Request empty;
  rclock::logDir::Response dir;
  ros::service::call("logDirectory", empty, dir);

  // add rxp to dir
  filename = dir.directory + filename; // /tmp/dat/YY_MM_DD_HH_MM_SS/raw.rxp

  try {
    // The basic_rconnection class contains the communication
    // protocol between the scanner or file and the program.
    shared_ptr<basic_rconnection> rc;

    rc = basic_rconnection::create("rdtp://" + ip +"/current");
    rc->open();

    while(true) {
      int f = open(filename.c_str(), O_WRONLY|O_APPEND|O_CREAT, S_IRWXU|S_IRWXG|S_IRWXO);
      int read = rc->readsome(buf, BUFLENGTH);
      if (read < 0 ) { 
        ROS_FATAL("RXPLOGGER: Error while reading from VZ-400. Code: %d", -read);
        close(f);
        exit(0);
      }
      int written = write(f, buf, read);
      
      if (written < 0 ) { 
        ROS_FATAL("RXPLOGGER: Error while writing to file %s. Code: %d", filename.c_str(), -written);
        close(f);
        exit(0);
      }

      close(f);
    }

    rc->close();
    return 0;
  }
  catch(exception& e) {
    cerr << e.what() << endl;
    return 1;
  }
  catch(...) {
    cerr << "unknown exception" << endl;
    return 1;
  }

  return 0;
}

#include <ros/ros.h>

#include <riegl/scanlib.hpp>

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

#include "riegl/RieglTime.h"
// The import class is derived from pointcloud class, which assembles the
// scanner data into distinct targets and computes x,y,z pointcloud data.
// The pointcloud class and its base class have a huge number of overridables
// that give access to all of the scanners data, e.g. "gps" or "housekeeping"
// data. The pointcloud class also has the necessary logic to align the
// data to gps information (if embedded in the rxp stream) and will return
// timestamps in the domain of the gps.

class importer
    : public pointcloud
{

public:
    importer(ros::Duration interval) : pointcloud(false) // set this to true if you need gps aligned timing
    {
      logtime = ros::Time(0);     // last time we published a time
      loginterval = interval;

      // announce topic
      ros::NodeHandle n;
      time_pub = n.advertise<riegl::RieglTime>("riegltime",100);

      // init riegltime
      rt.header.stamp = ros::Time::now();
      rt.header.frame_id = "/riegl";
    }


protected:
    ros::Publisher time_pub;
    riegl::RieglTime rt;


    // overridden from pointcloud class
    void on_echo_transformed(echo_type echo)
    {
      ros::Time rostime = ros::Time::now();
      if (rostime - logtime > loginterval ) {
        target& t(targets[target_count - 1]);
        rt.time = t.time;
        rt.header.stamp = rostime;
        time_pub.publish(rt);

        logtime = rostime;
      }
    }

private:
    ros::Time logtime;
    ros::Duration loginterval;
    FILE *file;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "RIEGL_sensor");
  ros::NodeHandle n;
  string ip;
  n.param<std::string>("/riegl/ip", ip, "192.168.0.125");
  double interval;
  n.param<double>("/log/interval", interval, 0.001);
  
  try {
    // The basic_rconnection class contains the communication
    // protocol between the scanner or file and the program.
    shared_ptr<basic_rconnection> rc;

    // The static create function analyses the argument and
    // gives back a suitable class that is derived from
    // basic_rconnection and can handle the requested protocol.
    // The argument string is modelled using the common 'uri'
    // syntax, i.e. a protocol specifier such as 'file:' or
    // 'rdtp:" is followed by the 'resource location'.
    // E.g. 'file:C:/scans/test.rxp' would specify a file that
    // is stored on drive C in the scans subdirectory.
    rc = basic_rconnection::create("rdtp://" + ip +"/current");
//    rc = basic_rconnection::create("rdtp://" + ip +"/current?type=hk");
    rc->open();

    // The decoder class scans off distinct packets from the
    // continuous data stream i.e. the rxp format and manages
    // the packets in a buffer.
    decoder_rxpmarker dec(rc);

    // The importer ( based on pointcloud and basic_packets class)
    // recognizes the packet types and calls into a distinct
    // function for each type. The functions are overidable
    // virtual functions, so a derived class can get a callback
    // per packet type.
//    importer     imp(ros::Duration(interval));
    importer     imp((ros::Duration)interval);

    // The buffer, despite its name is a structure that holds
    // pointers into the decoder buffer thereby avoiding
    // unnecessary copies of the data.
    buffer       buf;

    // This is the main loop, alternately fetching data from
    // the buffer and handing it over to the packets recognizer.
    // Please note, that there is no copy overhead for packets
    // which you do not need, since they will never be touched
    // if you do not access them.
    for ( dec.get(buf); !dec.eoi() && ros::ok(); dec.get(buf) ) {
      imp.dispatch(buf.begin(), buf.end());
    }

    cout <<  "done with dispatching!!!" << endl;

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

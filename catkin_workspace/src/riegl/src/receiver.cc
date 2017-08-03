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

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "riegl/RieglStatus.h"
// The import class is derived from pointcloud class, which assembles the
// scanner data into distinct targets and computes x,y,z pointcloud data.
// The pointcloud class and its base class have a huge number of overridables
// that give access to all of the scanners data, e.g. "gps" or "housekeeping"
// data. The pointcloud class also has the necessary logic to align the
// data to gps information (if embedded in the rxp stream) and will return
// timestamps in the domain of the gps.
int lc = 0;

static inline unsigned long GetCurrentRTime()
{
  static unsigned long milliseconds;
  struct timespec tp;
  //gettimeofday(&tv, NULL);
  clock_gettime(CLOCK_REALTIME, &tp);
  milliseconds = tp.tv_sec * 1000000000 + tp.tv_nsec;
  return milliseconds;
}


class importer
    : public pointcloud
{

public:
    importer() : pointcloud(false) // set this to true if you need gps aligned timing
    {
      publisher = n.advertise<sensor_msgs::PointCloud>("riegl",100);
      status = n.advertise<riegl::RieglStatus>("rieglstatus",100);
    }

protected:
    ros::NodeHandle n;
    ros::Publisher publisher;
    ros::Publisher status;

    vector<geometry_msgs::Point32> points;

    void sendPoints() {
      if (points.empty()) return;

      sensor_msgs::PointCloud rl;
      rl.header.stamp = ros::Time::now();
      rl.header.frame_id = "/riegl";
      rl.points.resize(points.size());
      rl.channels.resize(0); // TODO: also send reflectivity and other stuff
      for (unsigned int i = 0; i < points.size(); i++) {
        rl.points[i] = points[i];
      }
      publisher.publish(rl);
      points.clear();

    }

    void on_line_start_up(const line_start_up<iterator_type>& arg) {

      pointcloud::on_line_start_up(arg);
      lc++;
      sendPoints();
    }


    // overridden from pointcloud class
    void on_echo_transformed(echo_type echo)
    {
      target& t(targets[target_count - 1]);
      geometry_msgs::Point32 p;
      p.x = t.vertex[0];
      p.y = t.vertex[1];
      p.z = t.vertex[2];
      points.push_back(p);
    }
  
    void on_frame_start_up(const frame_start_up<iterator_type>& arg) {
      basic_packets::on_frame_start_up(arg);
      riegl::RieglStatus rs;
      rs.header.stamp = ros::Time::now();
      rs.header.frame_id = "/riegl";
      rs.status = 1; 
      status.publish(rs);
    }
    
    void on_frame_start_dn(const frame_start_dn<iterator_type>& arg) {
      basic_packets::on_frame_start_dn(arg);
      riegl::RieglStatus rs;
      rs.header.stamp = ros::Time::now();
      rs.header.frame_id = "/riegl";
      rs.status = 1; 
      status.publish(rs);
    }

    void on_frame_stop(const frame_stop<iterator_type>& arg) {
      
      basic_packets::on_frame_stop(arg);
      riegl::RieglStatus rs;
      rs.header.stamp = ros::Time::now();
      rs.header.frame_id = "/riegl";
      rs.status = 2; 
      status.publish(rs);
    }

    // overridden from basic_packets
    // this function gets called when a the scanner emits a notification
    // about an exceptional state.
    void on_unsolicited_message(const unsolicited_message<iterator_type>& arg) {
        pointcloud::on_unsolicited_message(arg);
        // in this example we just print a warning to stderr
        cerr << "WARNING: " << arg.message << endl;
        // the following line would put out the entire content of the packet
        // converted to ASCII format:
        // cerr << arg << endl;
    }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "RIEGL_sensor");
  ros::NodeHandle n;
  string ip;
  n.param<std::string>("/riegl/ip", ip, "192.168.0.125");

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
            importer     imp;

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

#include "rosbag2/recorder.h"
#include "rosbag/exceptions.h"

#include <rclock/logDir.h>
#include <string>
using std::string;

int main(int argc, char** argv) {
  ros::init(argc, argv, "record", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  
  string filename;
  n.param<std::string>("/log/bagfile", filename, "log.bag");
  
  // wait for global logging directory to become available
  ros::service::waitForService("logDirectory");
  rclock::logDir::Request empty;
  rclock::logDir::Response dir;
  // get loggin dir
  ros::service::call("logDirectory", empty, dir);
  filename = dir.directory + filename; // /tmp/dat/YY_MM_DD_HH_MM_SS/scan3d/

  rosbag2::RecorderOptions opts;
  opts.topics.push_back("/IrmaClock");
  opts.topics.push_back("/LMS");
  opts.topics.push_back("/VMC");
  opts.topics.push_back("/Vel");
  opts.topics.push_back("/clock");
  opts.topics.push_back("/imu_data");
  opts.topics.push_back("/odom");
  opts.topics.push_back("/robot_pose_ekf/odom_combined");
  opts.topics.push_back("/map");
  opts.topics.push_back("/riegl");
  opts.topics.push_back("/rieglstatus");
  opts.topics.push_back("/tf");
  opts.topics.push_back("/riegltime");

  opts.prefix = filename; 
  opts.append_date = false;
            

  // Run the recorder
  rosbag2::Recorder recorder(opts);
  int result = recorder.run();

  return result;
}

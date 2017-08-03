// =====================================================================================
// 
//       Filename:  calibration.cc
// 
//    Description:  ROS node to publish coordinate transformations between the various robot's coordinate frames
// 
//        Version:  1.0
//        Created:  05/11/2010 07:41:48 PM
//       Revision:  none
//       Compiler:  g++
// 
//         Author:  Jan Elseberg (), jelseber@uos.de
//        Company:  Universitaet Osnabrueck
// 
// =====================================================================================
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "volksbot_calibration");
  ros::NodeHandle n;

  ros::Rate r(50);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.12, 0.0, 0.24)), // 24 cm to the top and 12 to the front
        ros::Time::now(),"/base_link", "/front_laser"));
    r.sleep();
  }
}


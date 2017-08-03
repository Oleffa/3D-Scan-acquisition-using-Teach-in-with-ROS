#include "ros/ros.h"
#include "bachelorarbeit/AddTwoInts.h"
#include "std_msgs/Int64.h"

ros::Publisher publisher;

bool add(bachelorarbeit::AddTwoInts::Request  &req,
         bachelorarbeit::AddTwoInts::Response &res)
{
  ROS_INFO("Waypoint added");
  res.sum = req.a;
  std_msgs::Int64 t;
  t.data = res.sum;
  publisher.publish(t);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "WPhandler");
  ros::NodeHandle n;
  publisher = n.advertise<std_msgs::Int64>("WP",1000);
  ros::ServiceServer service = n.advertiseService("WPhandler_setWP", add);
  ROS_INFO("Ready to receive Waypoints");
  ros::spin();

  return 0;
}


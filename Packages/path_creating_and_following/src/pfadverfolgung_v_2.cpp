#include "ros/ros.h"
#include "vels.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "gio_path_v_2.h"
#include "math.h"
#include "volksbot/vels.h"
#include <sstream>
#include <string>
#include "std_srvs/Empty.h"

double x, y;
double angle;
char filename[255];
int filenumber = 0;
int asdf = 0;
string fname;

void odomSubscriber(const nav_msgs::Odometry::ConstPtr& msg){
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	double  qx = msg->pose.pose.orientation.x;
	double  qy = msg->pose.pose.orientation.y;
	double  qz = msg->pose.pose.orientation.z;
	double  qw = msg->pose.pose.orientation.w;
	angle = atan2(2 * ( qw * qz + qx * qy),1 - 2 * ( qz * qz + qy * qy));
}

int main(int argc, char **argv)
{
	//init stuff
	fname = argv[1];
	std_srvs::Empty e;
	ros::init(argc, argv, "talker");
	ros::NodeHandle node;
	ros::Publisher chatter_pub = node.advertise<volksbot::vels>("Vel", 1000);
	ros::Rate loop_rate(10);
	tf::TransformListener listener;  
	CGioController cgio;
	sprintf(filename, "%spath%d.dat",fname.c_str(),filenumber);


	//start point for each file read by the controller
	nextFile:
	cgio.getPathFromFile(filename);
	cgio.setCurrentVelocity(13);
	double u = 0.0;
	double w = 0.0;
	double vleft = 0.0;
	double vright = 0.0;

	while (ros::ok()){
	tf::StampedTransform transform;
	try{
		listener.lookupTransform("/map","/base_link",ros::Time(0),transform);
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
	}
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();
		angle = atan2(2*(transform.getRotation().w()*transform.getRotation().z()),1-2*(transform.getRotation().z()*transform.getRotation().z()));
	   // ROS_INFO("X::%f\tY::%f\tANGLE::%f\t",x,y,angle);

	if(cgio.getNextState(u,w,vleft,vright,1) == 0){
		if(asdf < 10){
			asdf = asdf +1;
		}else{
			ROS_INFO("Path Done");
			asdf = 0;
			break;
		}
	}else{
		cgio.setPose(x,y,angle);	
		cgio.getNextState(u,w,vleft,vright,1);
		volksbot::vels msg;
		msg.left = -vleft;
		msg.right = -vright;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	}
	
	filenumber = filenumber + 1;
	sprintf(filename, "%spath%d.dat", fname.c_str(), filenumber);
	if(cgio.getPathFromFile(filename) == 1){
		if(filenumber > 0){
			ROS_INFO("Start measuring");
			ros::service::call("startMeasuring",e);
			ros::Duration(6).sleep();
		}
		goto nextFile;
	}else{
		ROS_INFO("All paths done, END");
	}
  return 0;
}

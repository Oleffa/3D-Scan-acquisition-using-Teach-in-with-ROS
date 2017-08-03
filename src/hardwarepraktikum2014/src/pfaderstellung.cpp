#include <ros/ros.h> 
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <fstream>
#include "std_msgs/Int64.h"
#include <string>


int fileNumber = 0;
bool justPressed;

using namespace std;
// Initialize a callback to get velocity values: // 
void Odometry_Callback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message) {


ROS_INFO("Receiving"); 
ofstream os;
char data[100];

char filename[21];
sprintf(filename, "path%d.dat", fileNumber);

os.open(filename, ios::out|ios::binary|ios::app);

sprintf(data, "%3.3f %3.3f \n", message->pose.pose.position.x, message->pose.pose.position.y);

os << data;

os.close();

}

void SetWP(const std_msgs::Int64 message){
	if(justPressed == true){
		justPressed = false;
	}else{
		fileNumber = fileNumber + 1;
		justPressed = true;
		ROS_INFO("WP added");
	}
}

int main( int argc, char** argv) {

ros::init(argc, argv, "amcl_subscriber");
justPressed = false;
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("amcl_pose", 1000, Odometry_Callback);
ros::Subscriber sub_WP = n.subscribe("WP", 1000, SetWP);

ros::spin();

return 0;

}

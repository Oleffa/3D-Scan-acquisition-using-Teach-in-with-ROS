#include <ros/ros.h> 
#include "std_msgs/String.h"
//#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <fstream>
#include "std_msgs/Int64.h"
#include <string>


int fileNumber = 0;
bool justPressed;
int pointPrecision = 0; //increase to reduce number of points in path
int i = 1;

using namespace std;
string fname;
//constructor
class pfaderstellung
{
  public:
    pfaderstellung(const string& outputFolder)
    {
	//constructor
	}
};
// Initialize a callback to get velocity values:
// Old method body
//void Odometry_Callback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message) {
void Odometry_Callback( const geometry_msgs::PoseStamped::ConstPtr& message) {

ofstream os;
char data[100];
char filename[21];

sprintf(filename, "%spath%d.dat", fname.c_str(), fileNumber);

os.open(filename, ios::out|ios::binary|ios::app);

// old
//sprintf(data, "%3.3f %3.3f \n", message->pose.pose.position.x, message->pose.pose.position.y);
sprintf(data, "%3.3f %3.3f \n", message->pose.position.x, message->pose.position.y);
if(i >= pointPrecision){
	ROS_INFO("Received"); 
	 i = 0;
	os << data;
}else{
	ROS_INFO("Ignored"); 
	i = i + 1;
}
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

//old init code
//ros::init(argc, argv, "amcl_subscriber");
//justPressed = false;
//ros::NodeHandle n;
//ros::Subscriber sub = n.subscribe("amcl_pose", 1000, Odometry_Callback);
//new init code
//get path to folder from launch file
fname = argv[1];
ros::init(argc, argv, "pfaderstellung_subscriber");
cout << fname << endl;
justPressed = false;
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("pose", 1000, Odometry_Callback);

//independent
ros::Subscriber sub_WP = n.subscribe("WP", 1000, SetWP);
ros::spin();

return 0;

}

#include <ros/ros.h>
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

using namespace std;


///////////////////////////////////////////////////////////////////////////	var. declaration
#define PI 3.141592

geometry_msgs::Twist cmd_vel;

vector<float> lidar;							// lidar ranges

float inf=1/0.0;								// infinite number
float o_dist;									// object distance
float o_dir;									// object direction


///////////////////////////////////////////////////////////////////////////	parameters



///////////////////////////////////////////////////////////////////////////	sub, pub class
class sub_pub						
{
private:
	ros::NodeHandle nh;
	ros::Publisher vel_pub;
	ros::Subscriber lidar_sub;

public:
	sub_pub()									// subscriber, publisher declaration
	{
		vel_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		
		lidar_sub=nh.subscribe("/scan", 10, &sub_pub::lidar_callback, this);
		
		
	}
	
	void vel_publish()							// velocity command publish func.
	{
		vel_pub.publish(cmd_vel);
	}
	
	void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)	// call back function
	{
		lidar.clear();
		lidar=msg->ranges;
	}
};


///////////////////////////////////////////////////////////////////////////	object location func.
void object_location()
{
	int cnt=0;
	int dir=0;
	o_dist=0;
	
	for(int i=0; i<lidar.size(); i++)
	{
		if(lidar[i]!=inf)
		{
			o_dist+=lidar[i];
			
			if(i<(lidar.size()/2))
				dir+=i;
			else
				dir+=i-lidar.size();
			
			cnt++;
		}
	}
	o_dir=(float)dir/(float)cnt*360.0/(float)lidar.size();
	o_dist/=cnt;
	
	ROS_INFO("direction: %f", o_dir);
	ROS_INFO("distance: %f", o_dist);
}


///////////////////////////////////////////////////////////////////////////	main function
int main (int argc, char** argv)
{
	ros::init(argc, argv, "object_following");
	sub_pub sp;									// declare sub, pub class

	ros::Rate loop_rate(5);
	
	while(ros::ok())
	{
		ros::spinOnce();
		
		object_location();
		
		sp.vel_publish();
		
		loop_rate.sleep();
	}
	
	return 0;
}

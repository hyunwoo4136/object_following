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
float o_dist;									// object distance [m]
float o_dir;									// object direction [deg]


///////////////////////////////////////////////////////////////////////////	parameters
float t_dist=0.8;								// target distance [m]
float t_dir=0.0;								// target direction [deg]

float p_dist=0.6;								// p gain for distance control
float p_dir=0.9;								// p gain for direction control


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
		
		cmd_vel.linear.y=0;						// initialize velocity command
		cmd_vel.linear.z=0;
		cmd_vel.angular.x=0;
		cmd_vel.angular.y=0;
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
	o_dist/=cnt;
	o_dir=(float)dir/(float)cnt*2.0*PI/(float)lidar.size();
}


///////////////////////////////////////////////////////////////////////////	vel. control func.
void vel_control()
{
	float dist_err=t_dist-o_dist;
	float dir_err=t_dir-o_dir;
	
	cmd_vel.linear.x=-p_dist*dist_err;
	cmd_vel.angular.z=-p_dir*dir_err;
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
		vel_control();
		
		sp.vel_publish();
		
		loop_rate.sleep();
	}
	
	return 0;
}


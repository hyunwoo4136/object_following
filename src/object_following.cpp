#include <ros/ros.h>
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

using namespace std;


///////////////////////////////////////////////////////////////////////////	var. declaration
#define PI 3.141592

geometry_msgs::Twist cmd_vel;

bool sub_flag=false;

vector<float> lidar;							// lidar ranges
vector<float> lidar_f;							// filtered lidar ranges
vector<int> b_idx;								// boundary index

int l_num;										// lidar data element number
float dev;

float inf=1/0.0;								// infinite number
float o_dist;									// object distance [m]
float o_dir=0.0;								// object direction [deg]


///////////////////////////////////////////////////////////////////////////	parameters
float max_rng=8.0;								// lidar max range
float tau=0.5;									// high pass filter coefficient

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
		sub_flag=true;
		
		lidar.clear();
		lidar=msg->ranges;
		
		l_num=lidar.size();
		
		for(int i=0; i<l_num; i++)
		{
			if(lidar[i]==inf)
				lidar[i]=max_rng;
		}
	}
};


///////////////////////////////////////////////////////////////////////////	h-p filter func.
void high_pass_filter()
{
	lidar_f.clear();
	
	lidar_f.push_back(tau*lidar[0]);
	
	for(int i=1; i<l_num; i++)
	{
		lidar_f.push_back(tau*(lidar_f[i-1]+lidar[i]-lidar[i-1]));
	}
	
	lidar_f[0]=tau*(lidar_f[l_num-1]+lidar[0]-lidar[l_num-1]);
}


///////////////////////////////////////////////////////////////////////////	std. dev. func.
void std_deviation()
{
	float mean=0.0;
	
	for(int i=0; i<l_num; i++)
	{
		mean+=lidar_f[i];
	}
	mean/=l_num;
	
	for(int i=0; i<l_num; i++)
	{
		dev+=(lidar_f[i]-mean)*(lidar_f[i]-mean);
	}
	dev=sqrt(dev/l_num);
}


///////////////////////////////////////////////////////////////////////////	boundary func.
void find_boundary()
{
	b_idx.clear();
	
	for(int i=0; i<l_num; i++)
	{
		if(fabs(lidar_f[i])>dev)
			b_idx.push_back(i);
	}
	
	for(int i=b_idx.size()-1; i>=0; i--)
	{
		if((b_idx[i]-b_idx[i-1]==1) && (lidar_f[b_idx[i]]*lidar_f[b_idx[i-1]]>0))
			b_idx.erase(b_idx.begin()+i);
	}
	
	////////////////////////////////////////////////////////////////for debugging
	for(int i=0; i<b_idx.size(); i++)
	{
		ROS_INFO("boundary [%d]: %d", i, b_idx[i]);
	}
}


///////////////////////////////////////////////////////////////////////////	object location func.
void object_location()
{
	int dir=int(o_dir/2.0/PI*(float)l_num);
	int left_b;
	int right_b;
	
	o_dist=0;
	o_dir=0;
	
	if((dir>b_idx[b_idx.size()-1]-l_num) && (dir<b_idx[0]))
	{
		left_b=b_idx[b_idx.size()-1];
		right_b=b_idx[0];
	}
	else
	{
		for(int i=1; i<b_idx.size(); i++)
		{
			if(b_idx[i]<l_num/2)
			{
				if((dir>b_idx[i-1]) && (dir<b_idx[i]))
				{
					left_b=b_idx[i-1];
					right_b=b_idx[i];
					break;
				}
			}
			else
			{
				if((dir>b_idx[i-1]-l_num) && (dir<b_idx[i]-l_num))
				{
					left_b=b_idx[i-1];
					right_b=b_idx[i];
					break;
				}
			}
		}
	}
	
	////////////////////////////////////////////////////////////////for debugging
	ROS_INFO("reference direction: %d", dir);
	ROS_INFO("left boundary: %d", left_b);
	ROS_INFO("right boundary: %d", right_b);
	
	if(left_b<right_b)
	{
		for(int i=left_b; i<right_b; i++)
		{
			o_dist+=lidar[i];
			
			if(i<(l_num/2))
				o_dir+=(float)i;
			else
				o_dir+=float(i-l_num);
		}
		o_dist/=float(right_b-left_b);
		o_dir=o_dir/float(right_b-left_b)*2.0*PI/(float)l_num;
	}
	else
	{
		for(int i=0; i<right_b; i++)
		{
			o_dist+=lidar[i];
			
			o_dir+=(float)i;
		}
		
		for(int i=left_b; i<l_num; i++)
		{
			o_dist+=lidar[i];
			
			o_dir+=float(i-l_num);
		}
		o_dist/=float(right_b-left_b+l_num);
		o_dir=o_dir/float(right_b-left_b+l_num)*2.0*PI/(float)l_num;
	}
	
	////////////////////////////////////////////////////////////////for debugging
	ROS_INFO("distance: %f", o_dist);
	ROS_INFO("direction: %f", o_dir*360.0/2.0/PI);
}


///////////////////////////////////////////////////////////////////////////	vel. control func.
void vel_control()
{
	float dist_err=o_dist-t_dist;
	float dir_err=o_dir-t_dir;
	
	cmd_vel.linear.x=p_dist*dist_err;
	cmd_vel.angular.z=p_dir*dir_err;
}


///////////////////////////////////////////////////////////////////////////	main function
int main (int argc, char** argv)
{
	ros::init(argc, argv, "object_following");
	sub_pub sp;									// declare sub, pub class

	ros::Rate loop_rate(10);
	
	while(ros::ok())
	{
		ros::spinOnce();
		
		if(sub_flag==true)
		{
			high_pass_filter();
			std_deviation();
			find_boundary();
			object_location();
			vel_control();
			sp.vel_publish();
		}
		
		loop_rate.sleep();
	}
	
	return 0;
}


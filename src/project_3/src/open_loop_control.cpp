#include"ros/ros.h"
#include"iostream"
#include"sstream"
#include<stdio.h>
#include<math.h>
#include"geometry_msgs/Twist.h"

#define pi 3.141592

using namespace std;

float ext(int n)
{
	float intangle = ((n-2)*180)/n;
	float temp = 180 - intangle;
	//cout<<temp;
	temp = temp * (pi/180);
	//cout<<temp;
	return temp;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"open_loop_control_node");
	ros::NodeHandle nh;
	ros::Publisher pose_pub  = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",100);
	ros::Rate loop_rate(100);
	geometry_msgs::Twist msg;
	int n = 0; int l = 0;
	cout<<"Enter the number of sides"<<endl;
	cin>>n;
	cout<<"Enter the length of each side in meters"<<endl;
	cin>>l;
	float extangle = ext(n);
	for( int i = 1; i<= n ; i++)
	{
		for(int j = 1; 	j<=2*l; j++)
		{
			ros::Duration(0.5).sleep();
			{
				msg.linear.x =0.5;			
				msg.linear.y =0.0;
				msg.linear.z =0.0;

				msg.angular.x =0.0;
				msg.angular.y =0.0;
				msg.angular.z =0.0;
		
				pose_pub.publish(msg);
			}
		}
		if(i<n)
		{
			for(float k = 0.5; k<=2*extangle; k+=0.5)
			{
				ros::Duration(0.5).sleep();
				{
					msg.linear.x =0.0;			
					msg.linear.y =0.0;
					msg.linear.z =0.0;

					msg.angular.x =0.0;
					msg.angular.y =0.0;
					msg.angular.z =0.5;

					pose_pub.publish(msg);
				}
			}
		}
	}
	ros::spinOnce();
	loop_rate.sleep();
	return 0;
}

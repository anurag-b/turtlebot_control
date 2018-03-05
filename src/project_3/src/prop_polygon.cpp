#include"iostream"
#include"sstream"
#include<stdio.h>
#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include"geometry_msgs/Twist.h"
#include<nav_msgs/Odometry.h>
#include<tf/transform_listener.h>
#include<math.h>

#define pi 3.141592

using namespace std;

ros::Publisher pose_pub;

float poserror(float x1, float y1, float x2, float y2);
float ext(int n);
float  getLin_vel(float x1, float y1, float x2, float y2, float p1); 
float  getAng_vel(float x1, float y1, float x2, float y2, float theta1, float theta2,  float p2, float p3);
void moveBot(float l_vel, float a_vel);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prop_polygon_node");
  ros::NodeHandle nh;
  pose_pub  = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",100);
  geometry_msgs::Twist msg;
  ros::Rate rate(10.0);
  tf::TransformListener listener;
  tf::StampedTransform transform;
  float n,l, pose_check, linear_vel, angular_vel,chk;
  float p1 = 1.0;
  float p2 = 0.01;
  float p3 = 1.2;
  cout<<"Enter the number of sides"<<endl;
  cin>>n;
  cout<<"Enter the length of each side in meters"<<endl;
  cin>>l;
  float start[3], goal[2];
  float previous_ori = 0;
  try
  {
    listener.waitForTransform("/odom", "/base_footprint",
    ros::Time(0), ros::Duration(10.0) );			
    listener.lookupTransform("/odom", "/base_footprint",
    ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) 
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  start[0] = transform.getOrigin().x();
  start[1] = transform.getOrigin().y();
  start[2] = tf::getYaw(transform.getRotation());
  float *current = start;
  goal[0] = current[0];
  goal[1] = current[1];
  float goal_ori = current[2];
  printf("%F\n", current[0]);
  printf("%F\n", current[1]);
  printf("%F\n", goal_ori);
  for(int i = 1; i<=n; i++)
  {
    goal[0] = goal[0] + l * cos(goal_ori);
    goal[1] = goal[1] + l * sin(goal_ori);
    printf("Goal_x %F\n", goal[0]);
    printf("Goal_y %F\n", goal[1]);
    printf("Goal_ori %F\n", goal_ori); 
    while(1)
    {
      linear_vel = getLin_vel(current[0],current[1],goal[0],goal[1],p1);
      angular_vel = getAng_vel(current[0],current[1],goal[0],goal[1],current[2], goal_ori, p2,p3);
      printf("Linear Velocity = %F\n",linear_vel);
      printf("Angular Velocity = %F\n",angular_vel);
      
      moveBot(linear_vel,angular_vel);
      
      ros::Duration(1.0).sleep();
      ros::spinOnce();
      rate.sleep();

      tf::StampedTransform transform;
      try
      {
        listener.waitForTransform("/odom","/base_footprint",
        ros::Time(0), ros::Duration(10.0) );			
        listener.lookupTransform("/odom","/base_footprint",
        ros::Time(0), transform);
      }
      catch (tf::TransformException &ex) 
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
      current[0] = transform.getOrigin().x();
      current[1] = transform.getOrigin().y();
      current[2] = tf::getYaw(transform.getRotation());
      pose_check = poserror(current[0],current[1],goal[0],goal[1]);
      chk = 0.35;
      if(i == n)
        chk = 0.05;
      if (pose_check <= chk)
      {
        moveBot(0,0);
        printf("Pose Achieved\n");
        break;
      }
    }
    previous_ori = goal_ori;
    goal_ori = previous_ori + ext(n);
    if (goal_ori > pi)
      goal_ori = goal_ori - (2*pi);
    if(n==4)
    {
      if(goal_ori < -pi+0.017)
        goal_ori = pi;
    }
    //printf("new Goal_ori %F\n",goal_ori);
  }
  moveBot(0,0);
  ros::spinOnce();
  rate.sleep();
  return 0;
}

float  getLin_vel(float x1, float y1, float x2, float y2, float p1)
{
  printf("x1lin and y1lin = %F %F\n",x1,y1);
  printf("x2lin and y2lin = %F %F\n",x2,y2);
  float v_max = 0.2;
  float rho = poserror(x1,y1,x2,y2);
  printf("rho = %F\n",rho);
  float lin_vel = p1 * rho;
  if(lin_vel > v_max)
  {
    lin_vel = v_max;
  }
  return lin_vel;
}

float  getAng_vel(float x1, float y1, float x2, float y2, float theta1, float theta2,  float p2, float p3)
{
  float u_max = 0.5;
  float temp = atan((y2-y1)/(x2-x1));
  printf("theta1 = %F\n",theta1);
  printf("theta2 = %F\n",theta2);		
  float alpha = theta1 - temp;
  float neta = theta2 - theta1;
  printf("alpha = %F\n",alpha);
  printf("neta = %F\n",neta);
  float ang_vel = (p2 * alpha) + (p3 * neta);
  //float ang_vel = (p3 * neta);
  printf("Ang_vel = %F\n",ang_vel);
  if(ang_vel > u_max)
  {
    ang_vel = u_max;
  }
  if(ang_vel < -1*u_max)
  {
    ang_vel = u_max;
  }
  return ang_vel;
}

void moveBot(float l_vel, float a_vel)
{
  geometry_msgs::Twist msg;
  msg.linear.x =l_vel;			
  msg.linear.y =0.0;
  msg.linear.z =0.0;

  msg.angular.x =0.0;
  msg.angular.y =0.0;
  msg.angular.z =a_vel;
  
  pose_pub.publish(msg);
}

float poserror(float x1, float y1, float x2, float y2)
{ 
  printf("x1pose and y1pose = %F %F\n",x1,y1);
  printf("x2pose and y2pose = %F %F\n",x2,y2);
  float d = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
  return d;
}

float ext(int n)
{
  float intangle = ((n-2)*180)/n;
  float temp = 180 - intangle;
  //cout<<temp;
  temp = temp * (pi/180);
  cout<<temp;
  return temp;
}

#define DEBUG 0
#define DEBUG_ROS_INFO 1 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

float x,y,z;

void odomCallback(const nav_msgs::Odometry& msg)
{
	ROS_INFO("%.2lf %.2lf", msg.pose.pose.position.x, msg.pose.pose.position.y); 
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
	
}

int main(int argc, char **argv)
{
  FILE *fp;
  
  
  fp = fopen("//home//amap//race_data//race_log//race_log.txt","r");
  if(fp == NULL) 
  {
		ROS_INFO("error for race data log creation ~~ \n\n");
		exit(0);
  }
	
  ros::init(argc, argv, "odom_save");
  
  ros::NodeHandle n;  
  std::string odom_sub_topic = "/ackermann_steering_controller/odom";

  ros::param::get("~odom_pub_topic", odom_sub_topic);
  
  ros::Subscriber sub1 = n.subscribe(odom_sub_topic, 10, &odomCallback);
  ros::Rate loop_rate(5);  // 10
  
  while (ros::ok())
  {
	//fprintf(fp,"%f %f\n", x,y);	  
	loop_rate.sleep();
    ros::spinOnce();    
  }
  
  fclose(fp);
  
}


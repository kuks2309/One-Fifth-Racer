#define DEBUG 0
#define DEBUG_ROS_INFO 1 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h" 
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <string.h>  
#include <unistd.h>  
#include <errno.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/ioctl.h>  
#include <fcntl.h>  
#include <unistd.h>  
#include <sstream>
#include <math.h>



#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)
// unit : m


//////////////////////////////// Odometry //////////////////////////////

double roll,pitch,yaw,yaw_old;
double roll_d,pitch_d,yaw_d,yaw_d_old;
double delta_yaw_d, delta_yaw;

double datum_lat;
double datum_lon;
double datum_yaw;
double datum_utm_east;
double datum_utm_north;

double east;
double north;
int zone =0;

geometry_msgs::Pose2D Pose1;
geometry_msgs::Pose2D Pose2;
geometry_msgs::Pose2D initial_Pose1;
geometry_msgs::Pose2D initial_Pose2;
geometry_msgs::Pose2D delta_Pose1;
geometry_msgs::Pose2D initial_utm_pose;

geometry_msgs::PoseStamped Relative_current_pos;

void wgs2utm(double lat, double lon, int zone , double& east, double& north){
    double lat_rad = lat * M_PI/180;
    double lon_rad = lon * M_PI/180;

    double phi = lat_rad;
    double lambda = lon_rad;
    double lambda0 = (zone * 6 -183) * M_PI/180;
    double sm_a = 6378137;
    double sm_b = 6356752.31;

    double ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
    double nu2 = ep2*pow(cos(phi), 2.0);
    double N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
    double l = lambda - lambda0;
    double t = tan(phi);
    double t2 = t * t;

    double l3coef = 1 - t2 + nu2;
    double l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
    double l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
    double l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
    double l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
    double l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    east = N * cos(phi) * l + 
        (N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) + 
        (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) + 
        (N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

    double n = (sm_a - sm_b) / (sm_a + sm_b);
    double alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n,4.0) / 64.0));
    double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);
    double gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n,4.0) / 32.0);
    double delta = (-35.0 * pow(n,3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
    double epsilon = (315.0 * pow(n, 4.0) / 512.0);

    double ArcLengthMeridian = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0  * phi)) + (epsilon * sin(8.0 * phi)));

    north = ArcLengthMeridian + 
            (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0)) + 
            (t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) + 
            (t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) + 
            (t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
}


void gps_datum_Callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  //printf("datum test\n");	
  double easting;
  double northing;
  
  datum_lat = msg->x;               
  datum_lon = msg->y;
  datum_yaw = msg->z;  
	
  if(datum_lon < 0){
        zone = (datum_lon + 180) / 6 + 1;
    }
    else{
        zone = datum_lon / 6 + 31;
    }
    
  wgs2utm(datum_lat, datum_lon, zone, easting, northing);
  
  
  east = easting * 0.9996 + 500000;
  north = northing * 0.9996;

  //printf("Zone %d %lf %lf\n", zone,east,north);
  
   
  initial_utm_pose.x = east;  //gps datum 처리 
  initial_utm_pose.y = north;  //gps datum 처리	
}


void pose1Callback(const sensor_msgs::NavSatFix& msg)
{
	Pose1.y = (double)msg.latitude;
	Pose1.x = (double)msg.longitude;
	Pose1.theta = (double)msg.altitude;
}

void pose2Callback(const geometry_msgs::Pose2D& msg)
{
	Pose2.x = (double)msg.x;
	Pose2.y = (double)msg.y;
	Pose2.theta = (double)msg.theta;
}

void angleCallback(const std_msgs::Float32& msg)
{

	yaw = msg.data;
	printf("yaw1: %lf \n", RAD2DEG(yaw));
	
	//printf("yaw_init: %lf \n", RAD2DEG(init_yaw_angle));
	//yaw = yaw - init_yaw_angle;
	//printf("yaw2: %lf \n", RAD2DEG(yaw));
}

int main(int argc, char **argv)
{	
  
  datum_lat = datum_lon = 0.0;

  
  ros::init(argc, argv, "gps_utm_odom");

  ros::NodeHandle n;  
  std::string odom_pub_topic = "utm_odom";
  /*other*/
  std::vector<double> gps_init_datum ={0,};
  
  
  
  //gps_init_datum[0] = 0.0;
  //gps_init_datum[1] = 0.0;
  

  ros::param::get("/gps_init_datum",  gps_init_datum); //gps datum 수신 향후 처리
  ros::param::get("~odom_pub_topic", odom_pub_topic);
  
  ros::Subscriber subFix = n.subscribe("/ublox_gps/fix",10, &pose1Callback);    
  ros::Subscriber subUtm = n.subscribe("/gps/utm_pos1",10, &pose2Callback);     // utm 좌표 subscribe
  
  
  ros::Subscriber subHeading = n.subscribe("/gps_heading_angle",10, &angleCallback); 
  ros::Subscriber sub_gps_datum      = n.subscribe("/gps/datum",1,&gps_datum_Callback);       // front gps   
  
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_pub_topic, 20);
  
    
  //////////////////  odometry  //////////////////////
  std::string odom_frame_id = "odom";
  std::string odom_child_frame_id =  "gps_footprint"; // "base_link"
  
  ////////////////  TF odometry  //////////////////////
  static tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion odom_quat;
  
   //covariance matrix
  float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
                            0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
                            0,  0,    99999, 0,     0,     0,  // covariance on gps_z
                            0,  0,    0,     99999, 0,     0,  // large covariance on rot x
                            0,  0,    0,     0,     99999, 0,  // large covariance on rot y
                            0,  0,    0,     0,     0,     0.01};  // large covariance on rot z 
  //load covariance matrix
  for(int i = 0; i < 36; i++)
  {
      odom.pose.covariance[i] = covariance[i];;
  }     
  
  ros::Rate loop_rate(10);  // 10
 
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0; 
 
  //datum_lat = gps_init_datum[0];
  //datum_lon = gps_init_datum[1];
  
 
 

  while (ros::ok())
  {
	  
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
    std_msgs::String msg;
   
    std::stringstream ss;    
    std::string data;
    
    Relative_current_pos.pose.position.x =  (Pose2.y - initial_utm_pose.y);
	Relative_current_pos.pose.position.y =  -(Pose2.x - initial_utm_pose.x);   
    
    printf("GPS datum  %.4lf  %.4lf  %.4lf  %.4lf\n",datum_lat ,datum_lon, initial_utm_pose.x,initial_utm_pose.y);
       
   
    odom_quat = tf::createQuaternionMsgFromYaw(yaw);//yaw trans quat
 
     //pub tf(odom->base_footprint)
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odom_frame_id;     
    odom_trans.child_frame_id = odom_child_frame_id;       
    odom_trans.transform.translation.x = Relative_current_pos.pose.position.x;
    odom_trans.transform.translation.y = Relative_current_pos.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
     //pub odom
    odom.header.stamp = ros::Time::now(); 
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = odom_child_frame_id;       
    odom.pose.pose.position.x = Relative_current_pos.pose.position.x;     
    odom.pose.pose.position.y = Relative_current_pos.pose.position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;       
   //odom.twist.twist.linear.x = myOdomCaculateData.velocity_linear;
   //odom.twist.twist.angular.z = myOdomCaculateData.velocity_angular;
    odom_broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);  
     
   
    loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }


  return 0;
}


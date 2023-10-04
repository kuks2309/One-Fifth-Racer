#define DEBUG 1
#define DEBUG_ROS_INFO 1 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


#include "geometry_msgs/Pose2D.h"

#include "laser_geometry/laser_geometry.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>


// Image processing blob analysis
#include<string.h>
#include <iostream>
#include <sstream>
          

// unit : m
#define Sonar_Detect_Range 0.3

#define MAX_Obstacle_dist 20 //[m]

#define Laser_Scan_Data 720

#define LIDAR_Obstacle_avoid_angle 40
#define LIDAR_Side_Detection_anlge 90

#define OFF 0
#define ON  1

int    LIDAR_Obstacle_angle;   
double LIDAR_Obstacle_distance;
bool   LIDAR_Rotation_CCW;   
double Lane_Width = 0.5;
double Robot_Width; 
double Robot_Width_Tolerance;

double car_target_angle         =  0.0;
int    number_of_obstacle_point =   10;

tf::StampedTransform tf_map_to_base_link;
tf::StampedTransform tf_map_to_base_footprint;

sensor_msgs::PointCloud2  pc2_laser, pc2_2;
sensor_msgs::PointCloud2 *pc2_out = new sensor_msgs::PointCloud2;

tf::StampedTransform tf_map_to_laser;
tf::StampedTransform tf_laser_to_map;
                      
laser_geometry::LaserProjection projector_laser_map;
geometry_msgs::Pose2D obstacle_avoid_target_waypoint;

ros::Publisher test_pub;
ros::Publisher avoid_pose2D_pub;

struct Point 
{ 
	float x;
	float y;
	float z;
};

double roll,pitch,yaw;
double roll_d,pitch_d,yaw_d;
int heading_angle = 0;
int initial_heading_angle = 0;
int steer_angle = 0;
int steer_angle_wall = 0;
int status_avoidance_control = OFF;
double new_wp_x_base_link,new_wp_y_base_link;
    

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
  /*
   *   ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    */        
      tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
      tf2::Matrix3x3 m(q);     
 
      m.getRPY(roll, pitch, yaw);
      
     // printf("%6.3lf(rad)  %6.3lf \n",yaw, yaw*180/3.14159);
              
}
     
void car_target_angle_Callback(const std_msgs::Float32& msg) 
{
	
	car_target_angle = msg.data;
	
}



void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	sensor_msgs::PointCloud laser_cloud_map;
    int count = (int)(360. / RAD2DEG(scan->angle_increment));
    double *obstacle;
 
    int sum  =  0;
    int sum_obstacle = 0; 
    int sum_l = 0, sum_r = 0;
    double dist_y = 0.0; 
    int max=-1;
    int max_i = -1;
    int left_wall_sum = 0;
    double left_wall_min_distance  = MAX_Obstacle_dist;
    double right_wall_min_distance = MAX_Obstacle_dist;
    int right_wall_sum = 0;
    double free_space = 0.0;
    double free_space_min_angle = 0.0;
    double free_space_min_angle_degree = 0.0;
    double x,y;
    obstacle = new double[181];
    for(int i =0;i<181;i++) obstacle[i] = MAX_Obstacle_dist; 
    
   
   // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
   // ROS_INFO("%f %f",scan->scan_time , scan->time_increment);
   // ROS_INFO("%d angle_range, %f, %f %f", count,RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max), RAD2DEG(scan->angle_increment));
   // printf("count %d \n", count);
  
   
    tf::TransformListener tf_listener;
    
    bool found_trasnform = tf_listener.waitForTransform("/map", "/laser",ros::Time(0), ros::Duration(0.1) ); 
    if(found_trasnform != 1)  	
	{
		ROS_ERROR(" No transform from map to laser!");
		return;
	}
    
    
    geometry_msgs::TransformStamped tr_laser_map, tr_map_laser;
    tf_listener.lookupTransform("/laser", "/map", ros::Time(0), tf_laser_to_map);
    tf::transformStampedTFToMsg(tf_laser_to_map,tr_laser_map);
    
    tf_listener.lookupTransform("/map", "/laser", ros::Time(0), tf_map_to_laser);
    tf::transformStampedTFToMsg(tf_map_to_laser,tr_map_laser);
    
        
    laser_cloud_map.header.frame_id  = "map"; 
    laser_cloud_map.points.resize(count); 
    laser_cloud_map.channels.resize(1);
    laser_cloud_map.channels[0].name = "distance";
    laser_cloud_map.channels[0].values.resize(count);
    
    for(int i = 0; i < count; i++)
    {
		int degree = (int)RAD2DEG(scan->angle_min + scan->angle_increment * i);
        if(degree < 0) degree +=360;
        
        if(scan->ranges[i] <= LIDAR_Obstacle_distance )
        { 
			y =  scan->ranges[i] * sin( DEG2RAD(degree) );
			x =  scan->ranges[i] * cos( DEG2RAD(degree) );
			
			if( (fabs(y) < Lane_Width  ) )
			{
				
				sum_obstacle ++; 
				if(sum_obstacle >number_of_obstacle_point) ROS_WARN("Obstacle detected!");
				status_avoidance_control = ON;
				
			}
			else
			{
				status_avoidance_control = OFF; 
			}
		   
			if( /*(fabs(y) < Lane_Width  ) && */( (degree>=0) && (degree <= LIDAR_Obstacle_angle)  ) || ( (degree>=360-LIDAR_Obstacle_angle) && (degree<360)  ) )
			{
				geometry_msgs::Point laser_scan_point, laser_scan_map_point , map_scan_laser_scan_point;
				laser_scan_point.x = x;
				laser_scan_point.y = y;
				laser_scan_point.z = 0.0;
			
				//tf2::doTransform(laser_scan_point,laser_scan_map_point, tr_laser_map); 
				tf2::doTransform(laser_scan_point,laser_scan_map_point, tr_map_laser); 
				laser_cloud_map.points[i].x = laser_scan_map_point.x;
				laser_cloud_map.points[i].y = laser_scan_map_point.y;
				laser_cloud_map.points[i].z = laser_scan_map_point.z;
				laser_cloud_map.channels[0].values[i] = 1024;
				
				sum++;
			}		
		   if( (degree>=0) && (degree<=90-45) ) 
		   {     
			    // printf("test1: %3d, %6.3f %3d %6.3f\n",   degree,   scan->ranges[i], 90-degree ,obstacle[90-degree] );			     
			     if(LIDAR_Rotation_CCW == 1)    obstacle[90-degree] = (scan->ranges[i]);
			     else                           obstacle[degree+90] = (scan->ranges[i]);
		
		   }
		   if( (degree>=270+45) && (degree<360) ) 
		   { 
			 
			   //printf("test2:%3d, %6.3f %3d %6.3f\n",   degree,   scan->ranges[i], 450-degree ,obstacle[450-degree] );
			   if(LIDAR_Rotation_CCW == 1)     obstacle[450-degree] = (scan->ranges[i]);
			   else                            obstacle[degree-270] = (scan->ranges[i]);
		   }
		   
	    }
	    
       // ROS_INFO(": [%3d, %5d]", degree, obstacle[degree] );      
    }
    
     test_pub.publish(laser_cloud_map);
    
    // filtering
    
    
    
 
}



int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "lidar_obstacle_detect");
  
  LIDAR_Obstacle_angle      =  20;
  LIDAR_Obstacle_distance  =  5;
  LIDAR_Rotation_CCW        =  1;
  Robot_Width                =  0.3;
  Robot_Width_Tolerance    =  0.10;
  double roll_base_footprint, pitch_base_footprint, yaw_base_footprint;
  
  
  ros::NodeHandle n;  
  std::string lidar_topic = "/scan";
  std::string imu_topic = "/imu/data";
  std::string odom_pub_topic = "/odom";
  std::string status_avoid_contol_topic  ="/obstacle/lidar_avoid_control_status"; 
  std::string obstacle_avoid_angle_topic = "/Car_Control_cmd/Steer_avoidance"; 
  
  char frameid[] ="/sonar_range";
  
  /*other*/
  ros::param::get("~lidar_topic", lidar_topic);
  ros::param::get("~imu_topic", imu_topic); 
  ros::param::get("~odom_pub_topic", odom_pub_topic);
  ros::param::get("~status_void_contol", status_avoid_contol_topic);
  ros::param::get("~LIDAR_Obstacle_angle",     LIDAR_Obstacle_angle); //obstacle avoidance의 검출 앵글 -20~20
  ros::param::get("~LIDAR_Obstacle_distance",  LIDAR_Obstacle_distance); //obstacle avoidance의 검출 반경 1.2m
  ros::param::get("~LIDAR_Rotation_CCW",       LIDAR_Rotation_CCW);
  ros::param::get("~Lane_Width",               Lane_Width); 
  ros::param::get("~Robot_Width",              Robot_Width); 
  ros::param::get("~Robot_Width_Tolerance",    Robot_Width_Tolerance); 
  
  ros::param::get("~number_of_obstacle_point", number_of_obstacle_point);
  

  ros::Subscriber sub_IMU              = n.subscribe(imu_topic, 20, imuCallback);
  ros::Subscriber sub_lidar            = n.subscribe(lidar_topic, 10, &scanCallback);
 
  ros::Publisher obstacle_control_status_pub  = n.advertise<std_msgs::Int8>(status_avoid_contol_topic, 10);
  
   test_pub = n.advertise<sensor_msgs::PointCloud>("/pointcloud/laser_map",50); 
  
  printf("LIDAR_Obstacle_angle : %3d \n" , LIDAR_Obstacle_angle);
  printf("LIDAR_Obstacle_distance : %4.2lf \n" , LIDAR_Obstacle_distance);
  printf("LIDAR_Rotation_CCW : %1d \n" , LIDAR_Rotation_CCW);
  printf("Robot_Width : %4.2lf \n",Robot_Width);
     
  
  ////////////////   imu _sensor //////////////////////
  sensor_msgs::Imu imu_data;
  roll = pitch = yaw = 0.0;
  ////////////////   sonar _sensor //////////////////////
  sensor_msgs::Range sonar_msg;
  sonar_msg.header.frame_id =  frameid;
  sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg.field_of_view = (30.0/180.0) * 3.14;
  sonar_msg.min_range = 0.0;
  sonar_msg.max_range = 1.50;  //[unit :m]
  
  
  //////////////// Waypoint /////////////////////
  
  geometry_msgs::Pose2D  new_waypoint_obstacle;
  
  ////////////////// Visaul Makrer /////////////////////////
  Point p; 
  std::vector<Point> vec_point; 
  for(int i=0; i<1; i++) 
  { 
	  p.x = i; p.y = i; p.z = i; 
	  vec_point.push_back(p); 
  }

  visualization_msgs::MarkerArray node_link_arr;
  visualization_msgs::Marker line_list;

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  
  tf::TransformListener tf_listener;
  
  ros::Rate loop_rate(10);  // 10
  
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
  
    
	std_msgs::Int8 avoidance_control_status;
	avoidance_control_status.data = status_avoidance_control ;	
	
	obstacle_control_status_pub.publish(avoidance_control_status);
	status_avoidance_control = OFF;
	

	loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  
  
  
  return 0;
}




#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"


// maximum angular velocity 
#define MAX_L_STEER -6.0
#define MAX_R_STEER  6.0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoints_NO 6
#define WayPoint_X_Tor 0.1
#define WayPoint_Y_Tor 0.3

double pos_x = 0.0;
double pos_y = 0.0;
float vision_steering_angular_velocity = 0.0;
float car_linear_x = 0.0;
double roll,pitch,yaw;

float rqt_robot_steering_cmd_angular = 0.0;
float rqt_robot_steering_cmd_angular_old = 0.0;

float rqt_robot_steering_cmd_linear = 0.0;
float rqt_robot_steering_cmd_linear_old = 0.0;


struct Point 
{ 
	float x; 
	float y; 
	float z;
};


struct WayPoints
{
	double x;
	double y;
	
} ;

struct Current_Pos
{
	double x = 0.0;
	double y = 0.0;
	double yaw_angle = 0.0 ;
} my_pose;

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
	
	double r,p,y;
	my_pose.x = (double)msg.pose.position.x;
	my_pose.y = (double)msg.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      
   
    m.getRPY(r, p, y);
    my_pose.yaw_angle = y;		
}


void odomCallback(const nav_msgs::Odometry& msg)
{
	//ROS_INFO("%.2lf %.2lf", msg.pose.pose.position.x, msg.pose.pose.position.y); 
	my_pose.x = msg.pose.pose.position.x;
	my_pose.y = msg.pose.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      
 
    m.getRPY(roll, pitch, yaw);
    my_pose.yaw_angle = yaw;
    	
}

void rqt_robot_cmd_velCallback(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  rqt_robot_steering_cmd_angular = msg.angular.z;
  rqt_robot_steering_cmd_linear  = msg.linear.x ;
             
  if( (rqt_robot_steering_cmd_angular!= rqt_robot_steering_cmd_angular_old) ||  (rqt_robot_steering_cmd_linear!= rqt_robot_steering_cmd_linear_old))
  {
	  printf("rqt_steering\n");
	  vision_steering_angular_velocity = rqt_robot_steering_cmd_angular;
	  car_linear_x = rqt_robot_steering_cmd_linear;
	  if(vision_steering_angular_velocity  >= MAX_R_STEER)  vision_steering_angular_velocity  = MAX_R_STEER;
      if(vision_steering_angular_velocity  <= MAX_L_STEER)  vision_steering_angular_velocity  = MAX_L_STEER;  
	  
  }
  
  rqt_robot_steering_cmd_angular_old = rqt_robot_steering_cmd_angular;
  rqt_robot_steering_cmd_linear_old  = rqt_robot_steering_cmd_linear ;
   
}



void VisionSteerControlCallback(const std_msgs::Float32& angle)
{
  vision_steering_angular_velocity  = angle.data ;
  
  if(vision_steering_angular_velocity  >= MAX_R_STEER)  vision_steering_angular_velocity  = MAX_R_STEER;
  if(vision_steering_angular_velocity  <= MAX_L_STEER)  vision_steering_angular_velocity  = MAX_L_STEER;  
}


int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "car_navigation");

  ros::NodeHandle n;
   
  std::string odom_sub_topic = "/ackermann_steering_controller/odom";

  ros::param::get("~odom_pub_topic", odom_sub_topic);
   
  ros::Subscriber sub1 = n.subscribe("/Car_Control_cmd/Vison_SteerAngle_Float16",100, &VisionSteerControlCallback);  
  ros::Subscriber sub2 = n.subscribe("/slam_out_pose",100, &poseCallback);
  ros::Subscriber sub3 = n.subscribe(odom_sub_topic, 10, &odomCallback);
  ros::Subscriber sub4 = n.subscribe("/rqt_robot_steering/cmd_vel",100,&rqt_robot_cmd_velCallback);
   
  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);
  
  ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("marker/node", 1);
  ros::Publisher target_pos_pub = n.advertise<geometry_msgs::Pose2D>("pos_goal", 1);
 

  ros::Rate loop_rate(10);  // 10
  
  geometry_msgs::Twist msg_cmd;
  
  struct WayPoints my_waypoints_list[WayPoints_NO];
  int count = 0;
  int mission_flag[WayPoints_NO] = {0,};
  double pos_error_x = 0.0;
  double pos_error_y = 0.0; 
  
  
  my_waypoints_list[0].x = 0.4;   
  my_waypoints_list[0].y = 0.05;
  
  my_waypoints_list[1].x = 0.9;   
  my_waypoints_list[1].y = 0.1;  
   
  Point p; std::vector<Point> vec_point; 
  for(int i=0; i<2; i++)
  { 
	  p.x = my_waypoints_list[i].x; 
	  p.y = my_waypoints_list[i].y; 
	  p.z = 0.0; 
	  vec_point.push_back(p); 
  }
  
  visualization_msgs::MarkerArray marker_arr; 
  for (size_t i = 0; i < vec_point.size(); i++)
  { 
    Point o_marker = vec_point[i]; 
    visualization_msgs::Marker marker; 
    marker.header.frame_id = "/map"; // map frame ±âÁØ 
    marker.header.stamp = ros::Time::now(); 
    marker.type = visualization_msgs::Marker::SPHERE; 
    marker.id = i; 
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.orientation.w = 1.0; 
    marker.pose.position.x = o_marker.x; // marker x position
    marker.pose.position.y = o_marker.y; // marker y position  
    // Points are green 
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; 
    marker.scale.x = 0.05; 
    marker.scale.y = 0.05; 
    marker_arr.markers.push_back(marker);
  } 
  
  while (ros::ok())
  {
	/*  
	pos_error_x = abs(my_pose.x - my_waypoints_list[0].x);
	pos_error_y = abs(my_pose.y - my_waypoints_list[0].y);  
	ROS_INFO(" X : %6.3lf   Y : %6.3lf  Yaw : %6.3lf E_x : %6.3lf  E_y : %6.3lf", my_pose.x, my_pose.y, RAD2DEG(my_pose.yaw_angle), pos_error_x, pos_error_y);  
	if(( pos_error_x<= WayPoint_X_Tor)&&( pos_error_y <= WayPoint_Y_Tor))
    {  
	   ROS_INFO("WayPoint-1"); 
	   mission_flag[0] = 1; 
	} 
	
	pos_error_x = abs(my_pose.x - my_waypoints_list[1].x);
	pos_error_y = abs(my_pose.y - my_waypoints_list[1].y);  
	ROS_INFO(" X : %6.3lf   Y : %6.3lf  E_x : %6.3lf  E_y : %6.3lf", my_pose.x, my_pose.y, pos_error_x, pos_error_y);  
	if(( pos_error_x<= WayPoint_X_Tor)&&( pos_error_y <= WayPoint_Y_Tor))
    {  
	   ROS_INFO("WayPoint-2"); 
	   mission_flag[1] = 1; 
	} 
	 markers_pub.publish(marker_arr);
	 */
	 
	 msg_cmd.linear.x = car_linear_x;
	 if(msg_cmd.linear.x>=0)      msg_cmd.angular.z = vision_steering_angular_velocity ;
	 else                         msg_cmd.angular.z = -vision_steering_angular_velocity ;
     pub_cmd_vel.publish(msg_cmd);
	 
	 ROS_INFO(" X : %6.3lf  Y : %6.3lf  A : %.5lf ",my_pose.x, my_pose.y,vision_steering_angular_velocity);
	 
	loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  return 0;
}

#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define _USE_MATH_DEFINES

#include <math.h>
#include <iostream>

using namespace std;

#define MAX_angluar_velocity M_PI/10

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

double roll,pitch,yaw;
double roll_d,pitch_d,yaw_d, yaw_d_360;
double yaw_d_old  = 0.;
double target_yaw = 0.;

double Kp   = 0.01;
double Kd   = 0.04;
double Ki   = 0.00;
double error     = 0.0;
double error_d   = 0.0;
double error_sum = 0.0;

double linear_x, angular_z;

bool control_action_flag = 0;

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
      
    //printf("%6.3lf(rad)  %6.3lf \n",yaw, yaw*180/3.14159);
    yaw_d = RAD2DEG(yaw);
    
    yaw_d_360 = (yaw_d_360 < 0) ? yaw_d + 360 : yaw_d;
     
}

void cmd_vel_Callback(const geometry_msgs::Twist & cmd_input)
{
  
   linear_x  = cmd_input.linear.x ;//m/s
   angular_z = cmd_input.angular.z ;//rad/s
}

void gps_heading_angle_Callback(const std_msgs::Float32& msg)
{
	yaw   = msg.data;
	yaw_d = RAD2DEG(yaw);
}

void target_yaw_Callback(const std_msgs::Float32& msg)
{
	int     quotient  = 0  ;
	double  remainder = 0.0;
	target_yaw = msg.data ;
	//if(target_yaw < 0) target_yaw +=360;
	
	quotient  = target_yaw/360;
	target_yaw = target_yaw - 360*quotient;
	
	if(target_yaw  <-180)
	{
		target_yaw += 360;		
	}
	
	
	//target_yaw  =  (target_yaw >=  180) ? (target_yaw -360.0) : target_yaw;
	
}

void control_action_Callback(const std_msgs::Bool& msg)
{
	control_action_flag = msg.data;
}


double control_yaw(void)
{
	double cmd_vel_angluar_z;
	double yaw_d1;
	//int     quotient  = 0  ;
	//double  remainder = 0.0;
	int error_d = 0;
	double error1;
	double CW_flag = 0.0;
	
	CW_flag= sin(DEG2RAD(target_yaw-yaw_d));
	
	
   
    printf("sin(%6.3lf - %6.3lf) = %6.3lf\n ", target_yaw, yaw_d, CW_flag );
    //printf("error %6.3lf  %6.3lf \n", error1, error2);

	error1 = target_yaw - yaw_d;	
           
    if( (target_yaw > 179) || (target_yaw < -179))
    {
		error1 = target_yaw - yaw_d_360;		
	
	} 		
	
    error = error1;
    //if(fabs(error1) > fabs(error2))  error = error2;
    //else error = error2;
    
	cmd_vel_angluar_z = Kp * error + Kd * error_d + Ki * error_sum;
	
	error_d = error;
	yaw_d_old = yaw_d1;
	
	cmd_vel_angluar_z  =  (cmd_vel_angluar_z >=  MAX_angluar_velocity)?  MAX_angluar_velocity : cmd_vel_angluar_z;
	cmd_vel_angluar_z  =  (cmd_vel_angluar_z <= -MAX_angluar_velocity)? -MAX_angluar_velocity : cmd_vel_angluar_z;
	
	ROS_INFO("Yaw Angle : %6.3lf %6.3lf Target Yaw : %6.3lf Error : %6.3lf | cmd_vel_angluar_z %6.3lf",yaw_d,yaw_d_360,target_yaw,error,cmd_vel_angluar_z );
	
	return cmd_vel_angluar_z;
}

int main(int argc, char **argv)
{
 
  double pid_output = 0.0;
  ros::init(argc, argv, "car_yaw_control_node");
  ros::NodeHandle n;
  
  std::string imu_topic = "/imu";
  std::string gps_heading_angle_topic = "/gps_heading_angle";
  std::string cmd_vel_input_topic =  "/cmd_vel_input";
  std::string yaw_traget_topic = "/yaw_target_topic";
  std::string cmd_vel_output_topic = "/ackermann_steering_controller/cmd_vel";
  std::string control_action_topic = "/robot_yaw_control_enable";
  
  ros::param::get("~imu_topic", imu_topic);     
  ros::param::get("~cmd_vel_input_topic",  cmd_vel_input_topic); //cmd_vel_input
  ros::param::get("~cmd_vel_input_topic",  yaw_traget_topic); //cmd_vel_input
  ros::param::get("mobile_base/commands/velocity",  cmd_vel_output_topic);
  ros::param::get("gps_heading_angle",  gps_heading_angle_topic);
  
  
  cout << "imu_topic : " << imu_topic << endl;
  cout << "cmd_vel_input_topic :" << cmd_vel_input_topic <<  endl;  
  cout << "yaw_traget_topic : " << yaw_traget_topic <<  endl;    
  cout << "cmd_vel_output_topic : " << cmd_vel_output_topic << endl;  
  cout << "gps_heading_angle" <<  gps_heading_angle_topic<< endl;
  
  
  roll = pitch = yaw = roll_d = pitch_d = yaw_d = yaw_d_old = 0.0;
  
  geometry_msgs::Vector3 rpy_angle_radian;
  geometry_msgs::Vector3 rpy_angle_degree;
  
  geometry_msgs::Twist cmd_vel_msg;
   
   
  //ros::Subscriber subIMU                =  n.subscribe(imu_topic, 20, imuCallback);
  ros::Subscriber sub_target_yaw        =  n.subscribe(yaw_traget_topic, 10, target_yaw_Callback);
  ros::Subscriber sub_cmd_vel_inut      =  n.subscribe(cmd_vel_input_topic, 1, cmd_vel_Callback);  
  ros::Subscriber sub_control_action    =  n.subscribe(control_action_topic, 1,control_action_Callback);
  ros::Subscriber sub_gps_heading_angle =  n.subscribe(gps_heading_angle_topic,1,gps_heading_angle_Callback);
   
  ros::Publisher  cmd_vel_pub    =  n.advertise<geometry_msgs::Twist>(cmd_vel_output_topic, 20);
  
  ros::Rate loop_rate(10.0); //10.0HZ
  while(ros::ok())
  {
      rpy_angle_radian.x = roll;
      rpy_angle_radian.y = pitch;
      rpy_angle_radian.z = yaw;
      
      rpy_angle_degree.x = roll_d;
      rpy_angle_degree.y = pitch_d;
      rpy_angle_degree.z = yaw_d;
      
      if(control_action_flag == 1)
      {
        pid_output = control_yaw();
       
        cmd_vel_msg.angular.z = pid_output ;
        cmd_vel_msg.linear.x  = linear_x ;
          
	  }
	  else
	  {
	    cmd_vel_msg.linear.x  = linear_x ; 
	    cmd_vel_msg.angular.z = angular_z ; 
	  }
	  
      cmd_vel_pub.publish(cmd_vel_msg);     
       

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

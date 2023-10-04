#define DEBUG 0
#define DEBUG_ROS_INFO 1 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>
#include <iostream>

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)
#define LOOP_RATE            50
// unit m
#define Sonar_Detect_MAX_Range 3.0
#define Sonar_Obstacle_Range   1.0
#define Lidar_Obstacle_Range   1.0

// Steering Control
#define Neutral_Angle_Offset  3
#define Right_MAX -25
#define Left_MAX  25
#define STEER_NEUTRAL_ANGLE 3

int steering_angle = 0;
int steering_angle_cmd = 0;
float motor_speed = 0.0;
float motor_speed_cmd = 0.0;

int Base_Speed = 60;
int steering_angle_old = 0;
float motor_speed_old = 0;
float sonar[3]={0.0,};
long encoder1data = 0;
long encoder2data = 0;

double roll,pitch,yaw,yaw_old;
double roll_d,pitch_d,yaw_d,yaw_d_old;
double delta_yaw_d, delta_yaw;
int  imu_read_flag_start = 0;
bool imu_init_angle_flag = 0;
double init_yaw_angle = 0.0;
double init_yaw_angle_d = 0.0;

double imu_heading_anlge_offset = 0.0 ; //degree

double m_acceleration =  0.05;   //henes speed acceleratoin
double m_deceleration =  0.05;  //henes speed

double s_acceleration =  1;   //henes steer
double s_deceleration =  1;   //henes steer

double odom_init_x = 0.0;
double odom_init_y = 0.0;

struct BaseSensorData
{
    int encoder = 0;   
    int encoder_old = 0; 
}myBaseSensorData;


union
{
    float data ;
    char  bytedata[4];
} m_robot_speed , m_current_robot_speed;

struct OdomCaculateData
{
    //motor params
    float distance_ratio= 338.0 ;//0.000176; //unit: m/encode  338.0
    float encode_sampling_time=0.05; //unit: s-> 20hz
    float cmd_vel_linear_max= 1.2; //unit: m/s
    float cmd_vel_angular_max=0.8; //unit: rad/s
    //odom result
    float position_x=0.0; //unit: m
    float position_y=0.0; //unit: m
    float oriention=0.0; //unit: rad
    float velocity_linear=0.0; //unit: m/s
    float velocity_angular=0.0; //unit: rad/s
    
}myOdomCaculateData;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
	
	 char buf[8];
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
      roll_d  = RAD2DEG(roll);
      pitch_d = RAD2DEG(pitch);
      yaw_d   = RAD2DEG(yaw); 
      //printf("yaw_d: %lf \n", yaw_d);       
            
}


void imu_yaw_offset_Callback(const std_msgs::Float32& msg)
{
	
	imu_heading_anlge_offset = msg.data;
}



void encoder1Callback(const std_msgs::Int32& encoder_data1)
{
	encoder1data = encoder_data1.data;
}
void encoder2Callback(const std_msgs::Int32& encoder_data2)
{
	encoder2data = encoder_data2.data;
}


void CarControlCallback(const geometry_msgs::Twist& msg)
{
   steering_angle_cmd = (int)(msg.angular.z) ;
   
   if(steering_angle_cmd >= Left_MAX )   steering_angle_cmd = Left_MAX ;
   if(steering_angle_cmd <= Right_MAX )  steering_angle_cmd = Right_MAX ;
   
   motor_speed_cmd = (float)(msg.linear.x);

}


void CarSteerControlCallback(const std_msgs::Int16& angle)
{
  steering_angle_cmd = (int)(angle.data) ;
  
  if(steering_angle_cmd >= Left_MAX )   steering_angle_cmd = Left_MAX ;
  if(steering_angle_cmd <= Right_MAX )  steering_angle_cmd = Right_MAX ;   
  
}

void CarSpeedControlCallback(const std_msgs::Float32 speed)
{
  motor_speed_cmd = (float)speed.data;
   if(motor_speed_cmd>=3.0)   motor_speed_cmd = 3.0;
   if(motor_speed_cmd<=-3.0)  motor_speed_cmd = -3.0;   
  
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = (int)( 360. / RAD2DEG(scan->angle_increment));
    int sum=0; 
    
    for(int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        
        if(((degree>=90-15)&&(degree<=90+15)))
        {
          ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
          if(scan->ranges[i] <= Lidar_Obstacle_Range) 
          {
            sum++;
          }
          
        }
      
    }
     if(sum >=10)   
     {
       motor_speed_cmd = 0;
       ROS_INFO("Lidar Obstacle Detected !!");
       sum=0;
     }
     else           motor_speed = Base_Speed;     
}

                   
void sonar1Callback(const std_msgs::Float32& sonar_data)
{
	 sonar[0]= (float)(sonar_data.data);
}
void sonar2Callback(const std_msgs::Float32& sonar_data)
{
	 sonar[1]= (float)(sonar_data.data);
}
void sonar3Callback(const std_msgs::Float32& sonar_data)
{
	 sonar[2]= (float)(sonar_data.data);
}


void odometry_cal(void)
{		
  int delta_encoder = myBaseSensorData.encoder - myBaseSensorData.encoder_old;
  float base_link_delta_x;   float base_link_delta_y;  float base_link_delta_l;
  float odom_delta_x;        float odom_delta_y;  
  float radius;
  float temp = yaw;
  float temp_d = yaw_d;
  
  delta_yaw_d = yaw_d - yaw_d_old;
  
  delta_yaw = yaw - yaw_old;
  
  
  base_link_delta_l =  (float)delta_encoder ;  /// myOdomCaculateData.distance_
  
  if(fabs(delta_yaw)>1.0e-7) 
  {
	  
	 radius = base_link_delta_l / delta_yaw;
	 if(delta_yaw < 0)  base_link_delta_y = -radius*(1 - cos(delta_yaw));
	 else               base_link_delta_y =  radius*(1 - cos(delta_yaw));
	 
	 base_link_delta_x = radius * sin(delta_yaw);	
	      
  }
  else  
  {
	base_link_delta_x = base_link_delta_l;
	base_link_delta_y = 0.0;
  }  
    base_link_delta_x /= myOdomCaculateData.distance_ratio;
    base_link_delta_y /= myOdomCaculateData.distance_ratio;
    
    
    
    odom_delta_x =  base_link_delta_x * cos(yaw_old)  - base_link_delta_y * sin(yaw_old);   // rotation_matrix
	odom_delta_y =  base_link_delta_x * sin(yaw_old)  + base_link_delta_y * cos(yaw_old);

  myOdomCaculateData.position_x += odom_delta_x; //unit: m
  myOdomCaculateData.position_y += odom_delta_y; //unit: m
  myOdomCaculateData.oriention   = yaw ; //unit: rad	

  if(DEBUG == 1)printf(" %6.3lf %6.3lf %6.3lf \n", 	myOdomCaculateData.position_x, myOdomCaculateData.position_y, RAD2DEG(myOdomCaculateData.oriention));
     
  yaw_d_old = temp_d;       
  yaw_old = temp;      
}



void robot_steer_profile_control(void)
{
  if(steering_angle > steering_angle_cmd)   
  {
    steering_angle -=  s_deceleration / LOOP_RATE;
    steering_angle = (steering_angle <= steering_angle_cmd ) ? steering_angle_cmd : steering_angle;
  }
  else if(steering_angle < steering_angle_cmd)
  {  
    steering_angle +=  s_acceleration / LOOP_RATE; 
    steering_angle = (steering_angle >= steering_angle_cmd) ? steering_angle_cmd : steering_angle;
  }
  else 
  {
      steering_angle = steering_angle_cmd;
  }
}

void robot_speed_profile_control(void)
{
  if(motor_speed > motor_speed_cmd)   //현재 속도가 명령 속도 보다 클 때 , 감속 조건  m_robot_speed 가 명령어임
  {
    motor_speed -=  m_deceleration / LOOP_RATE;
    motor_speed = (motor_speed <= motor_speed_cmd ) ? motor_speed_cmd : motor_speed;
  }
  else if(motor_speed < motor_speed_cmd) //현재 속도가 명령속도 보다 클때 , 가속 조건
  { 
    motor_speed +=  m_acceleration / LOOP_RATE; 
    motor_speed = (motor_speed >= motor_speed_cmd) ? motor_speed_cmd : motor_speed;
  }
  else 
  {
      motor_speed = motor_speed_cmd;
  }
}
  

int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "Henes_Car_Control");

  ros::NodeHandle n;
  
  std::string cmd_vel_topic              = "/cmd_vel";
  std::string odom_pub_topic             = "/odom/car";
  std::string imu_topic                  = "/handsfree/imu";
  std::string imu_yaw_offset_angle_topic = "/imu/yaw_offset_degree"; 
   
  /*other*/
  ros::param::get("~m_acceleration", m_acceleration); //acceleraton  parameter of of motor
  ros::param::get("~m_deceleration", m_deceleration); //deceleration parameter of motor
  ros::param::get("~s_acceleration", s_acceleration); //acceleraton  parameter of steering
  ros::param::get("~s_deceleration", s_deceleration); //deceleration parameter of steering
   
  ros::param::get("~cmd_vel_topic", cmd_vel_topic);
  ros::param::get("~odom_pub_topic", odom_pub_topic);
  ros::param::get("~imu_topic", imu_topic); 
  ros::param::get("~imu_yaw_offset_angle_topic",  imu_yaw_offset_angle_topic);
  
  
  std_msgs::String msg;
  std_msgs::Int16 steerangle;
  std_msgs::Int16 carspeed;
  
    
  geometry_msgs::Twist teleop_cmd_vel_data;
  
  
  cout << "m_acceleration : " << m_acceleration << endl;  
  cout << "m_deceleration : " << m_deceleration << endl;
  
  cout << "s_acceleration : " << s_acceleration << endl;  
  cout << "s_deceleration : " << s_deceleration << endl;
  
  ros::Subscriber sub1                     = n.subscribe("/cmd_vel", 10, &CarControlCallback);
  ros::Subscriber sub2                     = n.subscribe("Car_Control_Cmd/SteerAngle_Int16",10, &CarSteerControlCallback);  
  ros::Subscriber sub4                     = n.subscribe("Car_Control_Cmd/Speed_Float32",10, &CarSpeedControlCallback);  
  //ros::Subscriber sub3 = n.subscribe("/scan", 1000, &scanCallback);
  ros::Subscriber subEncoder1              = n.subscribe("/encoder1",10,&encoder1Callback); // front  encoder data susscribe
  ros::Subscriber subEncoder2              = n.subscribe("/encoder2",10,&encoder2Callback); // rear  encoder data susscribe
 
  ros::Subscriber subIMU                   = n.subscribe(imu_topic, 20, &imuCallback);  // imu data susscribe
  ros::Subscriber sub_IMU_yaw_offset_angle =  n.subscribe(imu_yaw_offset_angle_topic, 20, imu_yaw_offset_Callback);
 
  //ros::Subscriber sonar_sub1 = n.subscribe("/sonar1", 10, &sonar1Callback);
  //ros::Subscriber sonar_sub2 = n.subscribe("/sonar2", 10, &sonar2Callback);
  //ros::Subscriber sonar_sub3 = n.subscribe("/sonar3", 10, &sonar3Callback);
  
  
  ros::Publisher teleop_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("teleop_cmd_vel", 10);
  
  //ros::Publisher car_control_pub1 = n.advertise<std_msgs::Int16>("Car_Control/SteerAngle_Int16", 10);
  //ros::Publisher car_control_pub2 = n.advertise<std_msgs::Int16>("Car_Control/Speed_Int16", 10);
  /*
  ros::Publisher car_sonar1_pub = n.advertise<sensor_msgs::Range>("/RangeSonar1",10);
  ros::Publisher car_sonar2_pub = n.advertise<sensor_msgs::Range>("/RangeSonar2",10);
  ros::Publisher car_sonar3_pub = n.advertise<sensor_msgs::Range>("/RangeSonar3",10);
  */
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_pub_topic, 20);
 
  ros::Rate loop_rate(LOOP_RATE);  // 10
  
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  
  //////////////////  odometry  //////////////////////
  std::string odom_frame_id = "odom";
  std::string odom_child_frame_id = "base_link";
  
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
  
   ros::Duration(1).sleep();   // 1 sec stop for safety
  
  while (ros::ok())
  {
	robot_speed_profile_control();
	robot_steer_profile_control();
    ROS_INFO("Cur Speed : %3.3f | Cmd Speed : %3.3f | Steering : %2d | Cmd Steering : %2d", motor_speed, motor_speed_cmd,steering_angle,steering_angle_cmd);    
    
    if(steering_angle != steering_angle_old) 
    {
       teleop_cmd_vel_data.angular.z = steering_angle;
	   teleop_cmd_vel_data.linear.x  = motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
    }
    
    if(motor_speed != motor_speed_old)
    {    
       teleop_cmd_vel_data.angular.z = steering_angle;
	   teleop_cmd_vel_data.linear.x  = motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
    }

    steering_angle_old = steering_angle;
    motor_speed_old = motor_speed; 
    
    //printf("sonar %6.3lf \n",sonar);
    
    //////////////////// sonar obstacle detectioin //////////////////
    if( ( (sonar[0]>0) && ( sonar[0] <= Sonar_Obstacle_Range ) ) || ( (sonar[1]>0) && ( sonar[1] <= Sonar_Obstacle_Range) ) ||  ( (sonar[2]>0) && ( sonar[2] <= Sonar_Obstacle_Range) ) ) 
    {
       //motor_speed_cmd  = 0;
       ROS_INFO("Sonar Obstacle detection : %3.2lf %3.2lf %3.2lf", sonar[0], sonar[1], sonar[2]);       
    }    
    else
    {
		 //motor_speed_cmd = motor_speed;
	} 
	
	   
    steerangle.data = steering_angle;
    carspeed.data = motor_speed;
  
    //car_control_pub1.publish(steerangle);
    //car_control_pub2.publish(carspeed);
    
    //ROS_INFO("Sonar : %5.1lf %5.1lf %5.1lf", sonar[0], sonar[1], sonar[2]);
    
    //myBaseSensorData.encoder = -(encoder1data + encoder2data) / 2;
    myBaseSensorData.encoder = -encoder2data;   //depend on encoder direction
	odometry_cal();
	myBaseSensorData.encoder_old =  myBaseSensorData.encoder;
	
	
    //odom_oriention trans to odom_quat
    odom_quat = tf::createQuaternionMsgFromYaw(myOdomCaculateData.oriention);//yaw trans quat
 
    //pub tf(odom->base_footprint)
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odom_frame_id;     
    odom_trans.child_frame_id = odom_child_frame_id;       
    odom_trans.transform.translation.x = myOdomCaculateData.position_x;
    odom_trans.transform.translation.y = myOdomCaculateData.position_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    //pub odom
    odom.header.stamp = ros::Time::now(); 
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = odom_child_frame_id;       
    odom.pose.pose.position.x = myOdomCaculateData.position_x;     
    odom.pose.pose.position.y = myOdomCaculateData.position_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;       
    //odom.twist.twist.linear.x = myOdomCaculateData.velocity_linear;
    //odom.twist.twist.angular.z = myOdomCaculateData.velocity_angular;
    //odom_broadcaster.sendTransform(odom_trans);
   // odom_pub.publish(odom);
    
    loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }

  motor_speed_cmd = 0;
  steering_angle_cmd = 0;
  
  return 0;
}




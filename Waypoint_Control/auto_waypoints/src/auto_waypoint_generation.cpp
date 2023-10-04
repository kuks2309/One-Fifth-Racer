#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"   
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/NavSatFix.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Imu.h"



#include <math.h>

#define MAX_L_STEER -30
#define MAX_R_STEER 30
#define STEER_NEUTRAL_ANGLE 0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define Line_Follwoing_Control_Mode  1.0  //단위는 m이며 가상 line following mode에 진입  가상 line을 중심으로 1.0m 이내에 들면 ㅣine follooing mode로 steering 각도를 제어 함


#define WayPoints_NO 500
#define WayPoint_X_Tor 0.4//0.4
#define WayPoint_Y_Tor 0.3//0.5

#define V_Region_NO  2
#define V_Speed_Region_NO 3
#define W_Region_NO  1
#define Pass_Region_NO 1
#define Park_Region_NO 1

double pos_x = 0.0;
double pos_y = 0.0;

int    vision_steering_angle   = 0;
int    waypoint_steering_angle = 0;
double steering_angle_base     = 0.0;
double car_steering_angle      = 0.0;
double car_speed               = 0;
int    no_waypoints            = WayPoints_NO;
int    lidar_obs_flag          = 0;
int    sonar_obs_flag          = 0;

bool   topic_gps_datum_rcv = false;
bool   use_utm_absolute_mode = true;
double waypoint_line_angle_utm = 0.0;

double datum_lat;
double datum_lon;
double datum_yaw;

double datum_utm_east;
double datum_utm_north;

double target_utm_x = 0.0;
double target_utm_y = 0.0; 

//init_flag
int init_flag          = 0;
int wp_go_id           = 0;
int wp_finish_id       = 0;
int run_flag           = 0;
bool use_imu_yaw_angle = 1;

int    steering_control_method       = 0;
double steering_correction_factor    = 1.0;
double way_point_ahead_distance      = 0.5;

double cross_track_offset  = 0.0;

double roll,pitch,yaw;

double imu_roll,imu_pitch,imu_yaw;
double imu_heading_angle                  = 0.0;
double gps_heading_angle                  = 0.0;


bool start_save_flag = true;

 
struct Point 
{ 
	double x; 
	double y; 
	double z;
};

struct WayPoints
{
	double x;
	double y;	
	double theta_degree;
};

struct Rect_Region
{
	double top;
	double bottom;
	double left;
	double right;	
};

geometry_msgs::Pose2D my_pose1;
geometry_msgs::Pose2D my_pose2;
geometry_msgs::Pose2D my_pose_odom;
geometry_msgs::Pose2D my_target_pose_goal;
geometry_msgs::Pose2D my_target_pose_goal_prev;
geometry_msgs::Pose2D my_pose_utm_waypoint;    // coordinate,  current waypoint to goal waypoint
geometry_msgs::Pose2D initial_gps_pose;
geometry_msgs::Pose2D obstacle_avoid_target_waypoint;
geometry_msgs::Pose2D utm_datum_pose;


struct WayPoints auto_waypoints_list[WayPoints_NO];


void wp_auto_make_flag_Callback(const std_msgs::Int8& flag)
{
	run_flag = flag.data;
}

//GPS의경우 UTM 좌표를 따라서 XY가 다름

void gps_datum_Callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	
	//printf("GPS Datum RCV!\n");
	topic_gps_datum_rcv = true;
	datum_lat = msg->x;               
	datum_lon = msg->y;
	datum_yaw = msg->z;  
    
}

void GPSHeadingAngleCallback(const std_msgs::Float32& msg)
{
	gps_heading_angle = msg.data;   // radian 으로 받을 것
}

 
void init_waypoint(void)
{
	
	int result = -10;
	
}


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


int fix = 0;
int fix1 = 0;
int fix2 = 0;

void gps1_check_Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)   // rear GPS
{
    fix1 = gps_msg->status.status; 
    
    if (gps_msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
    {
        ROS_WARN("GPS1  No fix.");
        return;
    }
    else if(gps_msg->status.status == 1) 
    {
		
		ROS_WARN("GPS1  float.");
		return;
	}
	else if(gps_msg->status.status == 2) 
	{
		//ROS_INFO("GPS1  FIX.");
	}
	else
	{
		
	}
      
} 

void gps2_check_Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)   // rear GPS
{
    fix2 = gps_msg->status.status; 
    
    if (gps_msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
    {
        ROS_WARN("GPS2  No fix.");
        return;
    }
    else if(gps_msg->status.status == 1) 
    {
		
		ROS_WARN("GPS2  float.");
		return;
	}
	else if(gps_msg->status.status == 2) 
	{
		//ROS_INFO("GPS2  FIX.");
	}
	else
	{
		
	}
      
} 

void gps_heading_angle_Callback(const std_msgs::Float32& msg)
{
	gps_heading_angle = msg.data;	
}

void imu_heading_angle_Callback(const std_msgs::Float32& msg)
{
	imu_heading_angle = msg.data;	
}

void gps1_utm_poseCallback(const geometry_msgs::Pose2D& msg)
{
	my_pose1.x     =   msg.x;      //UTM 좌표의 경우 map 좌표와 X,Y 좌표가 90도 회전되어 있음
	my_pose1.y     =   msg.y;      //UTM 좌표의 경우 map 좌표와 X,Y 좌표가 90도 회전되어 있음
	my_pose1.theta =   msg.theta;
	
//if(msg.theta <=0) my_pose.theta =  msg.theta + 2*M_PI;   
}

void gps2_utm_poseCallback(const geometry_msgs::Pose2D& msg)
{
	my_pose2.x     =   msg.x;      //UTM 좌표의 경우 map 좌표와 X,Y 좌표가 90도 회전되어 있음
	my_pose2.y     =   msg.y;      //UTM 좌표의 경우 map 좌표와 X,Y 좌표가 90도 회전되어 있음
	my_pose2.theta =   msg.theta;
	
//if(msg.theta <=0) my_pose.theta =  msg.theta + 2*M_PI;   
}


double waypoint_distance_limit =  2.0;                //[m]
double waypoint_angle_limit    = 10.0;                //degree

int main(int argc, char **argv)
{
	
	char buf[2];
	ros::init(argc, argv, "waypoint_auto_generation");

	ros::NodeHandle n;

	geometry_msgs::Pose2D gps_init_pose2d_data;

	datum_lat = datum_lon =  datum_yaw = 0.0; 


	std::string waypoint_save_file_path                = "/imu/heading_angle_radian";
	std::string waypoint_save_auto_flag_topic          = "/wp/waypoint_auto_make_flag";
	std::string imu_heading_angle_radian_topic         = "/imu/heading_angle_radian";
	std::string gps_heading_angle_radian_topic         = "/imu/heading_angle_radian";
	
	ros::param::get("~waypoint_distance_limit", waypoint_distance_limit);                 //auto generation을 위한 waypoint 간의 간격  - 이 값을 넘어가면 자동 생성
	ros::param::get("~waypoint_angle_limit"   , waypoint_angle_limit);                    //auto generation을 위한 waypoint 각도      - 이 값을 넘어가면 자동 생성
					
	
	ros::Subscriber sub_gps_datum                     = n.subscribe("/gps/datum",1,&gps_datum_Callback);  // front gps      
	ros::Subscriber sub_gps_heading_angle             = n.subscribe("/gps/heading_angle",1,&GPSHeadingAngleCallback);


	ros::Subscriber sub_imu_yaw_angle                 = n.subscribe(imu_heading_angle_radian_topic,1,&imu_heading_angle_Callback);

	ros::Subscriber sub_fix1                          = n.subscribe("/gps1/fix",1,&gps1_check_Callback);  // front gps 
	ros::Subscriber sub_fix2                          = n.subscribe("/gps2/fix",1,&gps2_check_Callback);  // rear  gps
	ros::Subscriber sub_gps_utm1                      = n.subscribe("/gps/utm_pos1",1, &gps1_utm_poseCallback);
	ros::Subscriber sub_gps_utm2                      = n.subscribe("/gps/utm_pos2",1, &gps2_utm_poseCallback);

	ros::Subscriber sub_auto_save_flag                = n.subscribe(waypoint_save_auto_flag_topic,1,&wp_auto_make_flag_Callback);  // front gps 
		
	ros::Rate loop_rate(30);  

	
	long count = 0;
	int mission_flag[WayPoints_NO] = {0,};
	double pos_error_x = 0.0;
	double pos_error_y = 0.0;
	int auto_wp_no = 0;
    bool waypoint_loop = false;
    
	ros::Time current_time, last_time;

	geometry_msgs::Pose2D pose_goal, pose_start; 

	nav_msgs::Path target_line_path; 

	init_waypoint(); 

	initial_gps_pose.x = datum_lat;  //gps datum 처리 
	initial_gps_pose.y = datum_lon;  //gps datum 처리


	int vision_id = -1;
	int vision_speed_id = -1;
	int waypoint_id = 0;

	double delta_x, delta_y ;
	delta_x = delta_y = 0.0;

	double base_line_a, base_line_b; // waypoint line between start and target point

	if(use_utm_absolute_mode == true)
	{
		ROS_INFO("\n\n\nutm absolute mode\n\n\n");
		ros::Duration(3.0).sleep() ;          
	}
	else
	{
		ROS_INFO("\n\n\n\nutm relative mode\n\n\n\n");  
		
		printf("topic_gps_datum_mode  : %d\n", topic_gps_datum_rcv );
			 
		ros::Duration(2.0).sleep() ;
	}

	
	while (ros::ok())
	{	
		current_time = ros::Time::now();
		
		gps_init_pose2d_data.theta =  0;
		
		double gps_utm_x = (my_pose1.x + my_pose2.x)/2;
		double gps_utm_y = (my_pose1.y + my_pose2.y)/2;
		
		if(start_save_flag == true)
		{
			
			if( (fix1 == 2) &&  (fix2 == 2) )
			{
								
				auto_waypoints_list[0].x             = gps_utm_x;
				auto_waypoints_list[0].y             = gps_utm_y;
				auto_waypoints_list[0].theta_degree  = RAD2DEG(gps_heading_angle);
				
				printf("[  0] waypoint :  %6.3lf, %6.3lf ,%6.3lf\n", auto_waypoints_list[0].x,auto_waypoints_list[0].y,auto_waypoints_list[0].theta_degree);
				start_save_flag = false;				
			}
		}
		
		double waypoint_distance = sqrt( pow( (gps_utm_x - auto_waypoints_list[auto_wp_no].x ), 2) + pow( (gps_utm_y - auto_waypoints_list[auto_wp_no].y ), 2) );
		double waypoint_angle    = fabs( RAD2DEG(gps_heading_angle) - auto_waypoints_list[auto_wp_no].theta_degree ) ;
		
		if( (start_save_flag == false) && ( (waypoint_distance >= waypoint_distance_limit) || (waypoint_angle > waypoint_angle_limit) ) )
		{
			printf("waypoint_distance : %6.3lf \n", waypoint_distance);
			auto_wp_no++;
			auto_waypoints_list[auto_wp_no].x             = gps_utm_x;
			auto_waypoints_list[auto_wp_no].y             = gps_utm_y;
			auto_waypoints_list[auto_wp_no].theta_degree  = RAD2DEG(gps_heading_angle);
			for(int i=0 ; i <= auto_wp_no ; i++)
			{
				printf("[%3d] waypoint :  %6.3lf, %6.3lf ,%6.3lf\n", i, auto_waypoints_list[i].x,auto_waypoints_list[i].y,auto_waypoints_list[i].theta_degree);
			}	
		}
		
		double waypoint_loop_distance = sqrt( pow( (gps_utm_x - auto_waypoints_list[0].x ), 2) + pow( (gps_utm_y - auto_waypoints_list[0].y ), 2) );
		
		
		if( (start_save_flag == false) && (auto_wp_no > 2) && (waypoint_loop_distance <= waypoint_distance_limit) )
		{
			waypoint_loop = true;
			printf("waypoint_loop_distance : %6.3lf \n", waypoint_loop_distance);
			printf("Waypoint Loop finish\n");
		}
		last_time = ros::Time::now();
		loop_rate.sleep();
		ros::spinOnce();
    
	}
	return 0;
}


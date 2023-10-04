#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os,sys
import time
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Float32

import numpy as np
import math
import utm
import tf


gps_datum_data=[0.0,0.0,0.0]
gps_fix_data=[0.0,0.0,0.0]


latitude =  0.0
logitude =  0.0 
altitude =  0.0 

gps_utm    = np.zeros(2)
gps1_utm   = np.zeros(2)
gps2_utm   = np.zeros(2)

gps1_lla   = np.zeros(2)
gps2_lla   = np.zeros(2)

offset_gps = 0.5

base_move  = 0.1



def gps1_fix_pub():
    global gps1_lla
    
    print("gps1_lla :", gps1_lla)
    pub = rospy.Publisher("/gps1/fix", NavSatFix, queue_size=10)
    msg = NavSatFix()
    
    msg.header.stamp = rospy.Time.now()   
    msg.header.frame_id = 'gps'
    msg.latitude  = gps1_lla[0]
    msg.longitude = gps1_lla[1]
    msg.altitude = 0
    msg.status.status = 2
    msg.position_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0)
    msg.position_covariance_type = 0
    
    pub.publish(msg)
    
    return
    
def gps2_fix_pub():
	global gps2_lla
	pub = rospy.Publisher("/gps2/fix", NavSatFix, queue_size=10)
	msg = NavSatFix()
    
	msg.header.stamp = rospy.Time.now()   
	msg.header.frame_id = 'gps'
	msg.latitude  = gps2_lla[0]
	msg.longitude = gps2_lla[1]
	msg.altitude  = 0
	msg.status.status = 2
	msg.position_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0)
	msg.position_covariance_type = 0
    
	pub.publish(msg)
    
def gps_heading_pub():
	global gps_fix_data
	pub = rospy.Publisher("heading", QuaternionStamped, queue_size=10)   
	
	msg = QuaternionStamped()
	msg.header.stamp = rospy.Time.now()   
	msg.header.frame_id = 'gps'
    
	quaternion = tf.transformations.quaternion_from_euler(0, 0, gps_fix_data[2]/180.0*math.pi)
	msg.quaternion.x = quaternion[0]
	msg.quaternion.y = quaternion[1]
	msg.quaternion.z = quaternion[2]
	msg.quaternion.w = quaternion[3]
    
	pub.publish(msg)    

def lla_to_utm():    
	print("base gps : lla to utm")
	global offset_gps
	global gps1_utm
	global gps2_utm
	
	angle1 = gps_fix_data[2] 
	angle2 = angle1 + 180.0
	#print("angle : ",angle1, angle2)  #utm_e 를 기준으로 각도
	coordinate = utm.from_latlon(gps_fix_data[0], gps_fix_data[1])
	gps_utm[0] = coordinate[0]
	gps_utm[1] = coordinate[1]
	

	
	#print( math.cos( math.radians(angle1) ) , math.cos( math.radians(angle2) ) )
	
	utm1_x = gps_utm[0] - offset_gps * math.sin( math.radians(angle1) )
	utm1_y = gps_utm[1] + offset_gps * math.cos( math.radians(angle1) )
	
	utm2_x = gps_utm[0] - offset_gps * math.sin( math.radians(angle2) )
	utm2_y = gps_utm[1] + offset_gps * math.cos( math.radians(angle2) )
	
	gps1_utm[0] = utm1_x
	gps1_utm[1] = utm1_y
	
	gps2_utm[0] = utm2_x
	gps2_utm[1] = utm2_y
	
	print("Front :", gps1_utm[0], gps1_utm[1])
	print("Rear  :", gps2_utm[0], gps2_utm[1])
	
	#print("bass gps utm ", gps_utm[0], gps_utm[1])
	return
	
def utm_to_lla():
	print("utm to lla")
	global offset_gps
	global gps_fix_data
	global gps1_lla
	global gps2_lla
	
	angle1 = gps_fix_data[2]
	angle2 = angle1 +180.0
	
	print("UTM  :",gps_utm[0], gps_utm[1])
	gos_lla_coordinate = utm.to_latlon(gps_utm[0], gps_utm[1],52,'U')
	print("GPS  :", gos_lla_coordinate[0], gos_lla_coordinate[1])
	
	gps_fix_data[0] = gos_lla_coordinate[0]
	gps_fix_data[1] = gos_lla_coordinate[1]
	
	coordinate = utm.from_latlon(gps_fix_data[0], gps_fix_data[1])
	gps_utm[0] = coordinate[0]
	gps_utm[1] = coordinate[1]
	
	#print( math.cos( math.radians(angle1) ) , math.cos( math.radians(angle2) ) )
	
	utm1_x = gps_utm[0] - offset_gps * math.sin( math.radians(angle1) )
	utm1_y = gps_utm[1] + offset_gps * math.cos( math.radians(angle1) )
	
	utm2_x = gps_utm[0] - offset_gps * math.sin( math.radians(angle2) )
	utm2_y = gps_utm[1] + offset_gps * math.cos( math.radians(angle2) )
	
	gps1_utm[0] = utm1_x
	gps1_utm[1] = utm1_y
	
	gps1_fix_data= utm.to_latlon(gps1_utm[0], gps1_utm[1],52,'U')
	gps1_lla = gps1_fix_data
	print("GPS1 :", gps1_fix_data[0],gps1_fix_data[1])
	
	gps2_utm[0] = utm2_x
	gps2_utm[1] = utm2_y
	
	gps2_fix_data= utm.to_latlon(gps2_utm[0], gps2_utm[1],52,'U')
	gps2_lla = gps2_fix_data
	print("GPS2 :", gps2_fix_data[0],gps2_fix_data[1])
	
	return		
    



def gps_callback(msg):
	global  latitude, longitude, altitude 
	global  gps_fix_data
	
	latitude = msg.latitude
	longitude = msg.longitude
	altitude = msg.altitude
	
	gps_fix_data[0] = latitude
	gps_fix_data[1] = longitude
	
	#rospy.loginfo("Latitude: %f, Longitude: %f, Altitude: %f", latitude, longitude, altitude)

	#print("Latitude: " + latitude + "Longitude: " + longitude "Altitude: " +altitude)
	
	lla_to_utm()
	utm_to_lla()
	
def imu_callback(msg):
	global  gps_fix_data
	orientation = msg.orientation
	
	euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    # Yaw 각도 추출 (라디안 값을 도로 변환)
	yaw = euler[2]
    
	yaw_degrees = euler[2] * 180.0 / 3.14159265359
	gps_fix_data[2] = yaw_degrees
	#rospy.loginfo("Yaw Angle (degrees): %.2f", yaw_degrees)
	
	pub_new_imu = rospy.Publisher("/robor/imu/data", Imu, queue_size=10)
	pub_new_imu_yaw_degree = rospy.Publisher("/robor/imu/yaw_degree", Float32, queue_size=10)
	pub_new_imu_yaw_radian = rospy.Publisher("/robor/imu/yaw_radian", Float32, queue_size=10)
	
	pub_new_imu.publish(msg)
	
	pub_new_imu_yaw_degree.publish(yaw_degrees)
	pub_new_imu_yaw_radian.publish(yaw)
	return; 
    

    	
def gps_publisher_node():
	rospy.init_node('gps_imu_publisher_node', anonymous=True)
	rospy.Subscriber('/gps/fix_front', NavSatFix, gps_callback)
	rospy.Subscriber('/imu', Imu, imu_callback)
	
	#gps1_pub = rospy.Publisher('/gps/fix1', NavSatFix, queue_size=1)
	#gps2_pub = rospy.Publisher('/gps/fix2', NavSatFix, queue_size=1)
	
	
	rate = rospy.Rate(10)  # 발행 속도 설정 (1 Hz로 설정)

	while not rospy.is_shutdown():
		# GPS 메시지를 발행합니다.
		gps1_fix_pub()
		gps2_fix_pub()
		
		rate.sleep()

if __name__ == '__main__':
	try:
		gps_publisher_node()
	except rospy.ROSInterruptException:
		pass



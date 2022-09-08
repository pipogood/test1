#!/usr/bin/env python
import rospy
import os
import math
import time
import json
from math import sin,cos
# from open_manipulator_msgs.msg import KinematicsPose
# from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import paho.mqtt.client as mqtt

Q1 = 'OK'
Q2 = 'OK'
Q3 = 'OK'
Q4 = 'OK'
Q5 = 'OK'
Q6 = 'OK'
Q7 = 'OK'
Q8 = 'OK'
Time = 0
dt = 0.1
send_to_unity = {}
flag_pub = 1
flag_time = 1

class DemoNode():
	def __init__(self):
		self.timer = rospy.Timer(rospy.Duration(dt), self.demo_callback)

	def demo_callback(self, timer):
		global flag_pub
		global Time
		global flag_time
		Time += dt
		if Q2 != "STOP" and Q3 != "STOP" and Q4 != "STOP" and Q5 != "STOP" and Q1 != "STOP" and Q6 != "STOP" and Q7 != "STOP" and Q8 != "STOP":
			flag_pub = 1
			#print("pub")
			client.publish("Mobot/lidar","OK")
			#if flag_time == 1:
				#Time = 0
				#flag_time = 0
			#if Time > 1.5:
				#print(Time)
				#flag_pub = 1
				#flag_time = 1

		send_to_unity = {'lidar_q1':Q1, 'lidar_q2':Q2, 'lidar_q3':Q3, 'lidar_q4':Q4, 'lidar_q5':Q5,'lidar_q6':Q6, 'lidar_q7':Q7, 'lidar_q8':Q8}
		client.publish("mobot/unity/lidar",json.dumps(send_to_unity,sort_keys=True))
		

def callback_ridar1(data):
	global Q2
	global Q3
	global Q4
	global Q5
	global flag_pub
	Q2 = "OK"
	Q3 = "OK"
	Q4 = "OK"
	Q5 = "OK"
	send_msg = {"mobi_vx":0,"mobi_vy":0,"mobi_w":0}
	count = int(math.floor(data.scan_time/data.time_increment))
	q2_stop = 0
	q2_beware =0
	q3_stop = 0
	q3_beware =0
	q5_stop = 0
	q5_beware =0
	for i in range (0,count):
		degree = (data.angle_min + data.angle_increment * i)*57.2958
		if(degree >= 90 and degree <= 180):
			degree = degree - 90
		elif(degree >= -180 and degree < 0):
			degree = (180-abs(degree)) + 90
		else:
			degree =  270 + degree
			
		#print(degree)
		lidar_y = (data.ranges[i])*sin(degree/57.2958)
		lidar_x = (data.ranges[i])*cos(degree/57.2958)

		if degree >= 0 and degree <= 270:
			if lidar_x >= 0.9 and lidar_x < 1.1 and lidar_y > 0.3 and lidar_y <= 0.5:
				#Q2 = "BEWARE"
				q2_beware += 1
				#print("Q2_BEWARE",degree,lidar_x,lidar_y)
			if lidar_x > 0.6 and lidar_x <= 0.9 and lidar_y >= 0 and lidar_y <= 0.3:
				#Q2 = "STOP"
				q2_stop += 1
				#print("Q2_STOP",degree,lidar_x,lidar_y)


			if lidar_x >= 0 and lidar_x < 0.6 and lidar_y > 0.3 and lidar_y <= 0.5:
				#Q3 = "BEWARE"
				q3_beware += 1
				#print("Q3_BEWARE",degree,lidar_x,lidar_y,data.ranges[i])
			if lidar_x >= 0 and lidar_x <= 0.6 and lidar_y >= 0 and lidar_y <= 0.3:
				#Q3 = "STOP"
				q3_stop += 1
				#print("Q3_STOP",degree,lidar_x,lidar_y,data.ranges[i])


			if lidar_x >= -0.4 and lidar_x < -0.2 and lidar_y >= 0.3 and lidar_y < 0.5:
				Q4 = "BEWARE"
				#print("Q4_BEWARE",lidar_x,lidar_y)
			if lidar_x > -0.2 and lidar_x <= 0 and lidar_y >= 0 and lidar_y <= 0.3:
				Q4 = "STOP"
				#print("Q4_BEWARE",lidar_x,lidar_y)

			if lidar_x >= -0.5 and lidar_x < -0.3 and lidar_y > -1.5 and lidar_y < 0:
				q5_beware += 1
				#Q5 = "BEWARE"
				#print("Q5_BEWARE",lidar_x,lidar_y)
			if lidar_x >= -0.3 and lidar_x <= -0.2 and lidar_y >= -1.5 and lidar_y <= 0:
				q5_stop += 1
				#Q5 = "STOP"
				#print("Q5_STOP",lidar_x,lidar_y)

	#print(q3_beware,q3_stop)
	if(q2_beware > 20):
		Q2 = "BEWARE" 
	if(q2_stop > 20):
		Q2 = "STOP" 
	if(q3_beware > 20):
		Q3 = "BEWARE" 
	if(q3_stop > 20):
		Q3 = "STOP" 
	if(q5_beware > 20):
		Q5 = "BEWARE" 
	if(q5_stop > 20):
		Q5 = "STOP" 
		
def callback_ridar2(data):
	global Q1
	global Q6
	global Q7
	global Q8
	global flag_pub
	Q1 = "OK"
	Q6 = "OK"
	Q7 = "OK"
	Q8 = "OK"
	send_msg = {"mobi_vx":0,"mobi_vy":0,"mobi_w":0}
	count = int(math.floor(data.scan_time/data.time_increment))
	q6_stop = 0
	q6_beware =0
	q7_stop = 0
	q7_beware = 0
	q8_stop = 0
	q8_beware = 0
	q1_stop = 0
	q1_beware = 0
	for i in range (0,count):
		degree = (data.angle_min + data.angle_increment * i)*57.2958

		if(degree >= 90 and degree <= 180):
			degree = degree - 90
		elif(degree >= -180 and degree < 0):
			degree = (180-abs(degree)) + 90
		else:
			degree =  270 + degree
		
		lidar_x = (data.ranges[i])*cos(degree/57.2958)
		lidar_y = (data.ranges[i])*sin(degree/57.2958)

		if degree >= 0 and degree <= 270:
			if lidar_x >= 0.9 and lidar_x < 1.1 and lidar_y > 0.3 and lidar_y <= 0.5:
				q6_beware += 1
				#print("Q6_BEWARE",degree,lidar_x,lidar_y)
			if lidar_x > 0.6 and lidar_x <= 0.9 and lidar_y >= 0 and lidar_y <= 0.3:
				q6_stop += 1
				#print("Q6_STOP",degree,lidar_x,lidar_y)


			if lidar_x >= 0 and lidar_x <= 0.6 and lidar_y > 0.3 and lidar_y <= 0.5:
				q7_beware += 1
				#print("Q7_BEWARE",degree,lidar_x,lidar_y)
			if lidar_x >= 0 and lidar_x <= 0.6 and lidar_y >= 0 and lidar_y <= 0.3:
				q7_stop += 1
				#print("Q7_STOP",degree,lidar_x,lidar_y)


			if lidar_x >= -0.4 and lidar_x < -0.2 and lidar_y >= 0.3 and lidar_y < 0.5:
				q8_beware += 1
				#print("Q8_BEWARE",lidar_x,lidar_y)
			if lidar_x > -0.2 and lidar_x <= 0 and lidar_y >= 0 and lidar_y <= 0.3:
				q8_stop += 1
				#print("Q8_BEWARE",lidar_x,lidar_y)


			if lidar_x >= -0.5 and lidar_x < -0.3 and lidar_y > -1.5 and lidar_y < 0:
				q1_beware += 1
				#print("Q1_BEWARE",lidar_x,lidar_y)
			if lidar_x >= -0.3 and lidar_x <= -0.2 and lidar_y >= -1.5 and lidar_y <= 0:
				q1_stop += 1
				#print("Q1_STOP",lidar_x,lidar_y)
	#print(q7_beware,q7_stop)
	if(q6_beware > 20):
		Q6 = "BEWARE" 
	if(q6_stop > 20):
		Q6 = "STOP" 
	if(q7_beware > 20):
		Q7 = "BEWARE" 
	if(q7_stop > 20):
		Q7 = "STOP" 
	if(q8_beware > 20):
		Q8 = "BEWARE" 
	if(q8_stop > 20):
		Q8 = "STOP" 
	if(q1_beware > 7):
		Q1 = "BEWARE" 
	if(q1_stop > 7):
		Q1 = "STOP" 

def talker():
	global send_str
	global flag_pub
	pub = rospy.Publisher('chatter', String, queue_size=10)
	rate = rospy.Rate(50) # 10hz
	while not rospy.is_shutdown():
		send_msg = {"mobi_vx":0,"mobi_vy":0,"mobi_w":0}
		send_str = Q1+' '+Q2+' '+Q3+' '+Q4+' '+Q5+' '+Q6+' '+Q7+' '+Q8
		#print(flag_pub)
		if Q1 == "STOP":		
			if flag_pub == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				flag_pub = 0
			rospy.loginfo(send_str)
			pub.publish(send_str)

		if Q2 == "STOP":
			
			rospy.loginfo(send_str)
			if flag_pub == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				flag_pub = 0
			pub.publish(send_str)

		if Q3 == "STOP":
			
			rospy.loginfo(send_str)
			if flag_pub == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				flag_pub = 0
			pub.publish(send_str)

		if Q4 == "STOP":
			
			rospy.loginfo(send_str)
			if flag_pub == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				flag_pub = 0
			pub.publish(send_str)

		if Q5 == "STOP":
			
			rospy.loginfo(send_str)
			if flag_pub == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				flag_pub = 0
			pub.publish(send_str)

		if Q6 == "STOP":
			
			rospy.loginfo(send_str)
			if flag_pub == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				flag_pub = 0
			pub.publish(send_str)

		if Q7 == "STOP":
			
			rospy.loginfo(send_str)
			if flag_pub == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				flag_pub = 0
			pub.publish(send_str)

		if Q8 == "STOP":
			
			rospy.loginfo(send_str)
			if flag_pub == 1:
				client.publish("unity/mobot/mobility",json.dumps(send_msg,sort_keys=True))
				flag_pub = 0
			pub.publish(send_str)

		rate.sleep()

def listener_ridar():
    rospy.Subscriber('/lidar0/scan', LaserScan, callback_ridar1)
    rospy.Subscriber('/lidar1/scan', LaserScan, callback_ridar2)
    DemoNode()
    talker()
    rospy.spin()

def myhook():
    print("Shutdown and Reintialize")



if __name__ == '__main__':

	rospy.init_node('listener_test')
	client = mqtt.Client()
	client.connect("192.168.200.2",1883)
	#client.connect("broker.hivemq.com",1883)
	try:
		listener_ridar()
	except rospy.ROSInterruptException:
		print("exception thrown")
		pass

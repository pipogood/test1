#!/usr/bin/env python
import paho.mqtt.client as mqtt #import the client1
import rospy
import json
import sys
import os
import time
import math
#import Jetson.GPIO as GPIO
from math import sin, cos
from open_manipulator_msgs.msg import KinematicsPose
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.srv import *

#0.025 0 0.803

home = "2 0 -0.23 0.23 0 0 0 0 5"
pose_1 = "1 0.5 0 0.1 0 0 0 0 5"
pose_2 = "1 0.5 0 -0.2 0 0 0 0 5"
set_stand = "2 0 0 -1.57 0 0 0 0 5"
shutdown = ""
stop = 0
stopend = 0
mode = 1
x = 0.650
y = 0
z = 0.453
pitch = 0
prev_x = 0.650
prev_y = 0
prev_z = 0.453
prev_yaw = 0
prev_pitch = 0
prev_roll = 0
grip_joint = 0
prev_grip = 0
prev_j1 = 0
prev_j2 = 0
prev_j3 = 0
prev_j4 = 0
prev_j5 = 0
prev_j6 = 0
#pre_val = None
#but_count = 0
#before_count = 0
#but_pin = 12
dt = 0.1
topic = ""
flag_stop = 0
flag_overw = 0
statouni = "yes"
MQTT_topic = [("unity/mobot/manipulator",0),("unity/mobot/gripper",0),("unity/mobot/manual",0),("Mobot/stop",0),("Mobot/shutdown",0)]

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
        client.subscribe(MQTT_topic)
        client.publish("manipulator/debug","connect ok",2)
        print("Idle State")
        client.publish("manipulator/debug","Idle State",2)
    else:
        print("Bad connection Returned code=", rc)

def on_disconnect(client, userdata, flags,rc=0):
    print("DisConnected result code" +str(rc))

def on_message(client,userdata,msg):
    global mes
    global stop
    global topic
    global shutdown
    global st
    global grip_joint
    global pitch

    topic = msg.topic
    m_decode=str(msg.payload.decode("utf-8","ignore"))
    mes = m_decode

    #print("message received",m_decode)


def run_begin():
    client.publish("manipulator/debug","Initialize",2)
    service_name = '/goal_joint_space_path'
    target_angle = [0,-0.23,0.23,0,0,0]
    joint_name = ["Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6"]
    rospy.wait_for_service(service_name)
    try:
        set_position = rospy.ServiceProxy(service_name, SetJointPosition)
        arg = SetJointPositionRequest()
        for i in range(0,6):
            arg.joint_position.joint_name.append(joint_name[i])
            arg.joint_position.position.append(target_angle[i])
            arg.path_time = 5
        resp1 = set_position(arg)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def callback_joint(data):
    global joint1
    global joint2
    global joint3
    global joint4
    global joint5
    global joint6

    data_j = data.position
    joint1 = data_j[0]
    joint2 = data_j[1]
    joint3 = data_j[2]
    joint4 = data_j[3]
    joint5 = data_j[4]
    joint6 = data_j[5]

class DemoNode(): #Timer
  def __init__(self):
    self.timer = rospy.Timer(rospy.Duration(dt), self.demo_callback)

  def demo_callback(self, timer):
    global statouni
    send_to_unity = {'mani_x': data_x, 'mani_y' : data_y, 'mani_z': data_z, 'workspace': statouni}
    #print(send_to_unity)
    client.publish("mobot/unity/manipulator",json.dumps(send_to_unity,sort_keys=True))
    # global Time
    # Time += dt


def callback_position(data):
    global data_x
    global data_y
    global data_z
    global data_ox
    global data_oy
    global data_oz
    global data_ow
    data_x = float(data.pose.position.x)
    data_y = float(data.pose.position.y)
    data_z = float(data.pose.position.z)
    data_ox = data.pose.orientation.x
    data_oy = data.pose.orientation.y
    data_oz = data.pose.orientation.z
    data_ow = data.pose.orientation.w

def callback_moving_status(data):
    global status
    status = data.open_manipulator_moving_state


def listener_joint_position():
    rospy.Subscriber('/joint_states', JointState, callback_joint)
    rospy.Subscriber('/gripper/kinematics_pose', KinematicsPose, callback_position)
    rospy.Subscriber('/states', OpenManipulatorState, callback_moving_status)
    DemoNode()
    rospy.sleep(0.05)

def set_state(state):
    service_name2 = '/set_actuator_state'
    rospy.wait_for_service(service_name2)
    set_actuator = rospy.ServiceProxy(service_name2, SetActuatorState)
    arg = SetActuatorStateRequest()
    try:
	arg.set_actuator_state = state
	resp1 = set_actuator(arg)
	return resp1

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def set_inverse_client(x, y, z, yaw ,pitch, roll, grip_joint, dt):
	global flag_overw
	global statouni
	flag_overw = 0
	service_name1 = '/goal_joint_space_path_to_kinematics_pose'
	rospy.wait_for_service(service_name1)
	set_position = rospy.ServiceProxy(service_name1, SetKinematicsPose)
	arg = SetKinematicsPoseRequest()
	arg.end_effector_name = 'gripper'
	try:
		arg.kinematics_pose.pose.position.x = x
		arg.kinematics_pose.pose.position.y = y
		arg.kinematics_pose.pose.position.z = z
		cy = cos(yaw * 0.5)
		sy = sin(yaw * 0.5)
		cp = cos(pitch * 0.5)
		sp = sin(pitch * 0.5)
		cr = cos(roll * 0.5)
		sr = sin(roll * 0.5)
		ow = cr * cp * cy + sr * sp * sy
		ox = sr * cp * cy - cr * sp * sy
		oy = cr * sp * cy + sr * cp * sy
		oz = cr * cp * sy - sr * sp * cy
		arg.kinematics_pose.pose.orientation.w = ow
		arg.kinematics_pose.pose.orientation.x = ox
		arg.kinematics_pose.pose.orientation.y = oy
		arg.kinematics_pose.pose.orientation.z = oz
		arg.path_time = dt
		resp1 = set_position(arg)
		rospy.sleep(0.1)
		#print("Moving State",status)
		if status == '"STOPPED"':
			#statouni = "no"
			flag_overw = 1
		#print("Moving State2",status,flag_overw)
		if status == '"IS_MOVING"' and flag_overw == 0:
			statouni = "yes"
		#print(statouni)
        #rospy.sleep(0.5)

        # ~ while True:
            # ~ #print("Moving State", data_x, data_y, data_z, data_ox, data_oy, data_oz, data_ow)
            # ~ #client.publish("manipulator/debug","Moving State",2)

            # ~ if(status == '"STOPPED"'):
                # ~ print("Reach to Goal Position")
                # ~ print("Idle State")
                # ~ client.publish("manipulator/debug","Reach to Goal Position",2)
                # ~ return resp1

            # ~ #print("status is:",status)

            # ~ if stop == 1:
                # ~ #set_state(False)
                # ~ #set_state(True)
                # ~ print("Stop State")
                # ~ client.publish("manipulator/debug","Stop State",2)
                # ~ #rospy.sleep(0.5)
                # ~ #set_forward_client(joint1,joint2,joint3,joint4,joint5,joint6,0,0.5)
                # ~ arg.kinematics_pose.pose.position.x = data_x
                # ~ arg.kinematics_pose.pose.position.y = data_y
                # ~ arg.kinematics_pose.pose.position.z = data_z
                # ~ arg.kinematics_pose.pose.orientation.w = data_ow
                # ~ arg.kinematics_pose.pose.orientation.x = data_ox
                # ~ arg.kinematics_pose.pose.orientation.y = data_oy
                # ~ arg.kinematics_pose.pose.orientation.z = data_oz
                # ~ arg.path_time = 0.5
                # ~ #rospy.sleep(0.1)
                # ~ resp1 = set_position(arg)
                # ~ break



	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		return False


def Gripper_Control(grip_joint,time):
    service_name = '/goal_tool_control'
    rospy.wait_for_service(service_name)
    try:
        set_position = rospy.ServiceProxy(service_name, SetJointPosition)
        arg = SetJointPositionRequest()
        arg.joint_position.joint_name.append("gripper")
        arg.joint_position.position.append(grip_joint)
        arg.path_time = time
        resp1 = set_position(arg)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

        
def set_forward_client(j1,j2,j3,j4,j5,j6,grip_joint,time):
    service_name = '/goal_joint_space_path'
    target_angle = [j1,j2,j3,j4,j5,j6]
    joint_name = ["Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6"]
    rospy.wait_for_service(service_name)
    try:
        set_position = rospy.ServiceProxy(service_name, SetJointPosition)
        arg = SetJointPositionRequest()
        for i in range(0,6):
            arg.joint_position.joint_name.append(joint_name[i])
            arg.joint_position.position.append(target_angle[i])
            arg.path_time = time
        resp1 = set_position(arg)
        print("Moving State")
        Gripper_Control(grip_joint,time)
        rospy.sleep(0.5)

        while True:
            #print("Moving State", data_x, data_y, data_z, data_ox, data_oy, data_oz, data_ow)
            #client.publish("manipulator/debug","Moving State",2)

            if(status == '"STOPPED"'):

                Gripper_Control(grip_joint,time)
                print("Reach to Goal Position")
                print("Idle State")
                client.publish("manipulator/debug","Reach to Goal Position",2)
                return resp1

            #print("status is:",status)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False


def run_mode():
	global dt
	global prev_yaw
	global prev_pitch
	global prev_roll
	global x
	global y
	global z
	global pitch
	global grip_joint
	global prev_x
	global prev_y
	global prev_z
	global prev_grip
	global prev_j1
	global prev_j2
	global prev_j3
	global prev_j4
	global prev_j5
	global prev_j6
	global mode
	global stop
	##for switch##
	global but_count
	global before_count
	global pre_val
	global flag_stop
	mode = 0

	if topic == 'unity/mobot/manipulator' and flag_stop == 0:
		st = json.loads(mes)
		mode = 1
		x = (st["mani_x"])
		y = (st["mani_y"])
		z = (st["mani_z"])
		print(x,y,z)

	if topic == 'unity/mobot/gripper' and flag_stop == 0:
		st = json.loads(mes)
		grip_joint = (st["gripper"])
		Gripper_Control(grip_joint,1)
		stop = 0

	if topic == 'unity/mobot/manual' and flag_stop == 0:
		st = json.loads(mes)
		mode = 2
		get = (st["manual"])
		if get == "mani_stand":
			set2 = set_stand
		else:
			set2 = home
		
	if topic == 'Mobot/stop':
		if mes == "STOP":
			flag_stop = 1
			set_inverse_client(data_x, data_y, data_z, 0 ,pitch, 0, grip_joint, 0.1)
			stop = 1
		if mes == "OK":
			flag_stop = 0
			stop = 0

	if topic == 'Mobot/shutdown':
		stop = 1
		shutdown = "SHUTDOWN"


	if stop == 0 and flag_stop == 0:
		#print('drive')
		over_limit = 0
		if mode == 1:
			yaw = 0
			pitch = 0
			roll = 0
			dt = 0.8
			if((x != prev_x) or (y != prev_y) or (z != prev_z) or (grip_joint != prev_grip)):

				if(z >= 0.3):
					pitch = 0.0
				else:
					pitch = 0.78
				
				if(x < 0.649): #0.58
					over_limit = 1

				if(z < -0.171):
					over_limit = 1

				if(abs(y) > 0.3):
						over_limit = 1

				if(mes == "STAND"):
					over_limit = 0

				if(over_limit == 1):
					print("Over Limit Workspace")
					client.publish("manipulator/debug","Over Limit Workspace",2)
				else:
					response = set_inverse_client(x, y, z, yaw ,pitch, roll, grip_joint, dt)
					prev_x = x
					prev_y = y
					prev_z = z
					prev_j1 = joint1
					prev_j2 = joint2
					prev_j3 = joint3
					prev_j4 = joint4
					prev_j5 = joint5
					prev_j6 = joint6
					prev_grip = grip_joint
			else:
				#print("Idle State")
				client.publish("manipulator/debug","Idle State",2)

		elif mode == 2:
			split = set2.split()
			j1 = float(split[1])
			j2 = float(split[2])
			j3 = float(split[3])
			j4 = float(split[4])
			j5 = float(split[5])
			j6 = float(split[6])
			grip_joint = grip_joint
			dt = float(split[8])
			if((j1 != prev_j1) or (j2 != prev_j2) or (j3 != prev_j3) or (j4 != prev_j4) or (j5 != prev_j5) or (j6 != prev_j6)):
				set_forward_client(j1,j2,j3,j4,j5,j6,grip_joint,dt)
				prev_j1 = j1
				prev_j2 = j2
				prev_j3 = j3
				prev_j4 = j4
				prev_j5 = j5
				prev_j6 = j6
				prev_grip = grip_joint
				prev_x = data_x
				prev_y = data_y
				prev_z = data_z
	else:
		prev_x = 0
		prev_y = 0
		prev_z = 0

def myhook():
	client.publish("manipulator/debug","Shutdown and Reinitialize",2)
	print("Shutdown and Reintialize")

rospy.init_node('master_publisher')
broker_address="broker.hivemq.com"

client = mqtt.Client("master")
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
print("Conecting to broker",broker_address)

#client.connect(broker_address,2580)
#client.connect(broker_address)
client.connect("192.168.200.2",1883)
run_begin()
listener_joint_position()
client.loop_start()

while True:
    time.sleep(0.1)
    if shutdown == "SHUTDOWN":
		set_forward_client(0,0,-1.57,0,0,-1.57,0,5)
		break
    else:
        run_mode()

os.system("rosnode kill master_publisher")
rospy.on_shutdown(myhook)
client.loop_stop()
client.disconnect()

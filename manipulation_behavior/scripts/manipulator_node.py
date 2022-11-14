#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty as Empty_srv
from std_msgs.msg import Int8
import numpy as np
from std_msgs.msg import String
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive


rtde_c = RTDEControl("192.168.20.35")
rtde_r = RTDEReceive("192.168.20.35")

home_joint = [4.71, -1.57, 0, -1.57, -1.57, 0]

set_case1 = [np.radians(270.34),np.radians(-107.72),np.radians(101.11),np.radians(-173.41),np.radians(-90.01),np.radians(0)]
set_case2 = [4.71, -1.57, 1.57, -1.57, -1.57, 0]

offset = [0, 0, 0.075, 1.5708, 0, 0]

class Ability(Node):
    def __init__(self):
        super().__init__('Manipulation')
        self.enble_service1 = self.create_service(Empty_srv,'/mani_grab/enable',self.mani_grab_enable_callback)
        self.enble_service2 = self.create_service(Empty_srv,'/mani_release/enable',self.mani_release_enable_callback)
        self.mani_grab_isEnable = False
        self.mani_release_isEnable = False
        
        rate = 10
        self.timer = self.create_timer(1/rate,self.timer_callback)

        self.mani_grab_publisher = self.create_publisher(Int8,'mani_grab/status',10)
        self.mani_release_publisher = self.create_publisher(Int8,'mani_release/status',10)
        
        self.mani_grab_status = Int8()
        self.mani_grab_status.data = 0

        self.mani_release_status = Int8()
        self.mani_release_status.data = 0

        rtde_c.moveJ(home_joint,0.5,1,asynchronous = False)
        rtde_c.moveJ(set_case1,1,1,asynchronous = False)

        self.stop = '0'
        self.moving_flag = 0
        self.desired_x = 0.0
        self.desired_y = 0.35
        rtde_c.setTcp(offset)
        self.recieve_timer = self.create_timer(0.008,self.receive_timer_callback) 
        self.subscription = self.create_subscription(String,'/mani_emer',self.emer_listener_callback,10) #service in future
        self.subscription = self.create_subscription(String,'/mani_pose',self.pose_control_callback,10) #service in future
        self.subscription

    def timer_callback(self):
        if self.mani_grab_isEnable:
            # call object service client 
            self.mani_grab_publisher.publish(self.mani_grab_status)

        #self.mani_grab_publisher.publish(self.mani_grab_status)

        if self.mani_release_isEnable:
            self.release_object()
            self.mani_release_publisher.publish(self.mani_release_status)

        #self.mani_release_publisher.publish(self.mani_release_status)

    def mani_grab_enable_callback(self,request,response):
        self.mani_grab_isEnable = True
        return response

    def mani_release_enable_callback(self,request,response):
        self.mani_release_isEnable = True
        return response

    def receive_timer_callback(self):
        self.current_pose = rtde_r.getActualTCPPose()
        self.current_pose = np.around(self.current_pose,3)
        self.joint_state = rtde_r.getActualQ()

        if self.moving_flag == 1:
            if self.current_pose[0] == self.pose[0] and self.current_pose[1] == self.pose[1] and self.current_pose[2] == self.pose[2]:
                self.mani_grab_status.data = 1
                self.mani_grab_isEnable = False
                self.mani_release_status.data = 1
                self.mani_release_isEnable = False
                self.moving_flag = 0 
                print("finish to pos")
        #print("POSE",np.around(self.current_pose,3))
        
        # check_con = rtde_c.toolContact([0,0,0,0,0,0])
        # if check_con > 0:
        #     pass
        #     print(check_con,'##############toolContact#############')

    def emer_listener_callback(self,msg):
        self.stop = msg.data
        print(msg.data)
        if self.stop == '1':
            rtde_c.stopJ(1)

        elif self.stop == '0':
            rtde_c.moveJ(home_joint,0.3,1,asynchronous = True)

    def pose_control_callback(self,msg):
        self.moving_flag = 1
        self.mani_grab_status.data = 0 
        self.flag_overlimit = 0
        mes = msg.data
        self.pose = mes.split()
        self.pose = list(map(float, self.pose))
        for i in range (0,3):
            self.pose.append(0)
        print(self.pose)

        rtde_c.setTcp(offset)

        if self.pose[2] >= 0.16:
            if self.current_pose[2] < 0.16:
                rtde_c.moveJ(set_case1,1,1,asynchronous = False)
            self.pose[3] = 0
            self.pose[4] = 0
            self.pose[5] = 0
            if self.pose[1] < 0.383:
                self.flag_overlimit = 1
                print("poseY fail")
        else:
            if self.current_pose[2] >= 0.16:
                rtde_c.moveJ(set_case2,1,1,asynchronous = False)
            self.pose[3] = 4.711
            self.pose[4] = 0.038
            self.pose[5] = -0.009
            if self.pose[1] < 0.35:
                self.flag_overlimit = 1
                print("poseY fail")

        if self.pose[2] < -0.32:
            self.flag_overlimit = 1
            print("poseZ fail")

        if rtde_c.isPoseWithinSafetyLimits(self.pose):
            if rtde_c.getInverseKinematics(self.pose):
                self.joint = rtde_c.getInverseKinematics(self.pose)
                if np.radians(90.0)-np.radians(45.0) < self.joint[0] < np.radians(270.0)+np.radians(45.0):
                    pass
                else:
                    self.flag_overlimit = 1
                    print("joint1 fail",self.joint[0])

                if self.joint[1] > np.radians(30.0):
                    self.flag_overlimit = 1
                    print("joint2 fail",self.joint[1])

        else:
            self.flag_overlimit = 1
            print("pose fail")


        if self.flag_overlimit == 0:
            rtde_c.moveJ_IK(self.pose,0.5,0.5,asynchronous = True)
        else:
            self.PosToNav(self.pose[0],self.pose[1])

    def PosToNav(self,X,Y):
        self.moveX = X - self.desired_x
        self.moveY = Y - self.desired_y
        self.mani_grab_status.data = 1
        self.mani_grab_isEnable = False 
        #call service client send to Nav. 
        print("To Nav is:",self.moveX, self.moveY)

    def release_object(self):
        self.mani_release_status.data = 0
        rtde_c.moveJ(set_case2,1,1,asynchronous = False)
        rtde_c.moveJ_IK([0.0,0.35,self.pose[2],0,0,0],0.5,0.5,asynchronous = True)

def main(args=None):
    rclpy.init(args=args)
    
    ability = Ability()
    rclpy.spin(ability)
    ability.destroy_node()
    ability.shutdown()
    rclpy.shutdown()


if __name__=='__main__':
    main()

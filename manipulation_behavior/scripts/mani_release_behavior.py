#!/usr/bin/env python3

import py_trees
from std_srvs.srv import Empty
from std_msgs.msg import Int8

class AbilityBehavior(py_trees.behaviour.Behaviour):
    """
    Node Name :
        * node name *
    Client :
        * /ability_name/enable *
        call service ... in ... server node
    Subscription :
        *...*
    ...
    
    """
    print("init_work1")
    def __init__(self):
        super(AbilityBehavior,self).__init__()
        self.mani_release_status = Int8()
        self.mani_release_status.data = 0

        self.node = None
        self.mani_release_subscription = None
        self.mani_release_enable_client = None

    def setup(self,**kwargs):
        self.node = kwargs['node']
        self.mani_release_enable_client = self.node.create_client(Empty,'/mani_release/enable')
        self.mani_release_subscription = self.node.create_subscription(Int8,'/mani_release/status',self.mani_release_subscription_callback, qos_profile = 10)                                                                                                                    

    def initialise(self):
        pass

    def send_enable_request(self):
        req = Empty.Request()
        self.future = self.mani_release_enable_client.call_async(req)

    def mani_release_subscription_callback(self,msg):
        self.mani_release_status = msg

    def update(self) -> py_trees.common.Status:

        if self.mani_release_status.data == 1: #change to be your condition
            return py_trees.common.Status.SUCCESS

        elif self.mani_release_status.data == 0: #change to be your condition
            self.send_enable_request()

            return py_trees.common.Status.RUNNING

        else: #change to be your condition
            return py_trees.common.Status.FAILURE


    def terminate(self, new_status):
        pass
#!/usr/bin/python3

from fun4.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import *
# import roboticstoolbox as rtb
import math
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped
from controller_mode_interface.srv import ControllerMode

class RandomNode(Node):
    def __init__(self):
        super().__init__('random_node')

    # ------------------------------ Parameters ------------------------------- #
        self.declare_parameter('frequency',100)
        self.freq = self.get_parameter('frequency').value
        
        self.declare_parameter('r_max',0.53)
        self.r_max = self.get_parameter('r_max').value 

        self.declare_parameter('r_min',0.03)
        self.r_min = self.get_parameter('r_min').value

        self.declare_parameter('z_offset',0.2)
        self.z_offset = self.get_parameter('z_offset').value # Joint offset from base
        self.create_timer(1 / self.freq,self.timer_callback)

        self.target_pub_ = self.create_publisher(PoseStamped,'target',10)
        self.generate_pose_server = self.create_service(ControllerMode,'/get_random_pos',self.callback_generate_pose)

        self.random_array = [0.0,0.0,0.0]

    

    def axis_workspace(self):
        value = np.random.uniform(-self.r_max, self.r_max)
        return value if -self.r_min <= value <= self.r_max else np.random.uniform(-self.r_max, self.r_max)

    def generate_random_workspace_values(self):
        # Hollow Sphere worksapce
    
       x = self.axis_workspace()
       y = self.axis_workspace()
       z = self.axis_workspace()

       if (x**2 + y**2 + z**2 <= self.r_max**2):
        self.random_array[0] = x
        self.random_array[1] = y
        self.random_array[2] = z + self.z_offset

    def callback_generate_pose(self,request,respond):
        pass

    def timer_callback(self):
        # self.generate_random_workspace_values()

        msg = PoseStamped()
        msg.pose.position.x = self.random_array[0]
        msg.pose.position.y = self.random_array[1]
        msg.pose.position.z = self.random_array[2]
        self.target_pub_.publish(msg)

        


def main(args=None):
    rclpy.init(args=args)
    node = RandomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
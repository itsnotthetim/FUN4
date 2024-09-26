#!/usr/bin/python3

from fun4.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import roboticstoolbox as rtb
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped
from controller_mode_interface.srv import ControllerMode
from sensor_msgs.msg import JointState
from math import pi
from spatialmath import SE3

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('frequency',100)
        self.freq = self.get_parameter('frequency').value

        self.pose_pub_ = self.create_publisher(JointState,'joint_states',10)

        self.create_subscription(PoseStamped,'target',self.pose_callback,10)

        self.create_timer(1/self.freq,self.timer_callback)

        self.chmod_ = self.create_service(ControllerMode,'/change_mode',self.chmod_server_callback)
        
        self.robot_ = rtb.DHRobot(
        [   
            rtb.RevoluteMDH(offset=pi/2,d=0.2),
            rtb.RevoluteMDH(alpha=pi/2,offset=pi/2,d=0.1),
            rtb.RevoluteMDH(a=0.25,d=-0.1)
        ],tool = SE3.Tx(0.28), name="HelloWorld"
        )

        self.r_min = 0.03
        self.r_max = 0.53
        self.configuration_space = [0.42,0.1,0.33]
        self.name = ["joint_1", "joint_2", "joint_3"]
        self.mode = -1
        self.pose_data = [0.0,0.0,0.0]
        self.flag = 0

    def chmod_server_callback(self,request,respond):
        self.mode = request.mode

        if(self.mode == 1):
            self.pose_data = request.pose.position
            x = self.pose_data.x
            y = self.pose_data.y
            z = self.pose_data.z

            if (self.compute_pose(x, y, z) is not False and self.check_possible_workspace(x, y, z) is not False):
                print(f"mode 1 {self.pose_data}")
                self.pose_publishing(self.compute_pose(0.2,0.2,0.2))
                respond.success = True
                respond.joint_pos.position = self.compute_pose(x,y,z)
                
            else:
                respond.success = False
                respond.joint_pos.position = [x,y,z]
        
        elif(self.mode == 2):
            print("mode 2")
            respond.success = True
        
        elif(self.mode == 3):
            print("mode 3")
            self.pose_data = request.pose.position
            respond.success = True

        return respond
    
    def compute_pose(self,x,y,z):
        if (x**2 + y**2 + z**2 <= self.r_max**2):
            T_desired = SE3(x,y,z)
            q = self.q = self.robot_.ik_LM(T_desired)
            return [q[0][0],q[0][1],q[0][2]]
        else:
            return False

    def check_possible_workspace(self, x, y, z):
        return -self.r_min <= x <= self.r_max and -self.r_min <= y <= self.r_max and -self.r_min <= z <= self.r_max


    def pose_callback(self,msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # self.configuration_space = self.compute_pose(x,y,z)

    def pose_publishing(self,pose_array):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(pose_array)):
            msg.position.append(pose_array[i])
            msg.name.append(self.name[i])
        # msg.position = pose_array
        # msg.name = self.name
        self.pose_pub_.publish(msg)

    def timer_callback(self):
       pass
        
        


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

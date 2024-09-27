#!/usr/bin/python3

from fun4.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import roboticstoolbox as rtb
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped
from controller_mode_interface.srv import ControllerMode , CallRandomPos
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from math import pi
from spatialmath import SE3
from scipy.optimize import minimize
import numpy as np

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('frequency',100)
        self.freq = self.get_parameter('frequency').value
  
        self.declare_parameter('r_max',0.53)
        self.r_max = self.get_parameter('r_max').value 

        self.declare_parameter('r_min',0.03)
        self.r_min = self.get_parameter('r_min').value

        self.declare_parameter('z_offset',0.2)
        self.z_offset = self.get_parameter('z_offset').value # Joint offset from base

        self.pose_pub_ = self.create_publisher(JointState,'joint_states',10)

        self.create_subscription(PoseStamped,'target',self.pose_callback,10)

        self.create_timer(1/self.freq,self.timer_callback)

        self.chmod_ = self.create_service(ControllerMode,'/change_mode',self.chmod_server_callback)

        
        self.robot_ = rtb.DHRobot(
        [   
            rtb.RevoluteMDH(offset=pi/2,d=0.2),
            rtb.RevoluteMDH(alpha=pi/2,offset=pi/2,d=0.02),
            rtb.RevoluteMDH(a=0.25)
        ],tool = SE3.Tx(0.28), name="HelloWorld"
        )

        self.configuration_space = [0.42,0.1,0.33]
        self.name = ["joint_1", "joint_2", "joint_3"]
        self.mode = -1
        self.pose_data = [0.0,0.0,0.0]
        self.initial_guess = [0,0,0]
        self.current_pose = [0,0,0]
        self.flag = 0

        # /end-effector
        self.buffer = Buffer()
        self.tf_callback= TransformListener(self.buffer, self)
        self.eff_fraame = "end_effector"
        self.refference_frame = "link_0"

        
    def get_pos_eff(self):
        try:
            now = rclpy.time.Time()
            transform = self.buffer.lookup_transform(
                self.refference_frame , 
                self.eff_fraame,   
                now)
            position = transform.transform.translation
            
            self.current_pose = position.x,position.y,position.z
            print(self.current_pose)

            # self.get_logger().info(f"End Effector Position: {position.x}, {position.y}, {position.z}")
            #self.get_logger().info(f"End Effector Orientation: {orientation.x}, {orientation.y}, {orientation.z}, {orientation.w}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
    

    def call_random_position(self,bool):
        client = self.create_client(CallRandomPos,'get_random_pos')
        random_req = CallRandomPos()
        random_req.is_call = bool
        if random_req.data  == True:
            future = client.call_async(random_req)



    def chmod_server_callback(self,request,respond,random=None):
        self.mode = request.mode

        if(self.mode == 1):
            self.pose_data = request.pose.position
            x = self.pose_data.x
            y = self.pose_data.y
            z = self.pose_data.z
            print(x,y,z)
            if (self.compute_pose(x, y, z) is not False and self.check_possible_workspace(x, y, z) is not False):
                print(f"mode 1 {self.pose_data}")
                self.pose_publishing(self.compute_pose(x,y,z))
                respond.success = True
               
                self.initial_guess = self.compute_pose(x,y,z)
                print(self.initial_guess)
                respond.joint_pos.position = [float(value) for value in self.compute_pose(x, y, z)]

                
            else:
                respond.success = False
                respond.joint_pos.position = [x,y,z]
            return respond
        
        elif(self.mode == 2):
            print("mode 2")
            respond.success = True
            return respond
        
        elif(self.mode == 3):
            print("Auto mode has been started")
            # self.call_random_position(True)
            print(self.pose_data)
            respond.success = True
            return respond
    


    def pose_callback(self,msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.pose_data = [x,y,z]
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
    #    self.get_pos_eff()
        pass
        


    # ---------------------------------------------------------------------------------------------------------------------------- #

    def custom_ikine(self,robot, T_desired, initial_guess):
        # Define the objective function
        def objective(q):
            T_actual = robot.fkine(q)
            return np.linalg.norm(T_actual.A - T_desired.A)
        
        # Run the optimization
        result = minimize(objective, initial_guess, bounds=[(-pi, pi) for _ in initial_guess])
        
        return result.x

    def compute_pose(self,x,y,z):
        if (self.r_min**2 <= x**2 + y**2 + z**2 <= self.r_max**2 ):

            T_desired = SE3(x,y,z+self.z_offset)
            q = self.custom_ikine(self.robot_,T_desired,self.initial_guess)
            return q
        else:
            return False
    

    def check_possible_workspace(self, x, y, z):
        return -self.r_min <= x <= self.r_max and -self.r_min <= y <= self.r_max and -self.r_min  <= z <= self.r_max  


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

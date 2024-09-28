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
import time

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
        self.end_effector_pub = self.create_publisher(PoseStamped,'end_effector',10)

        self.create_subscription(PoseStamped,'target',self.random_pose_callback,10)
        self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)

        self.random_pos_client = self.create_client(CallRandomPos,'get_rand_pos')

        self.create_timer(1/self.freq,self.timer_callback)

        self.chmod_ = self.create_service(ControllerMode,'/change_mode',self.chmod_server_callback)

        
        self.robot_ = rtb.DHRobot(
        [   
            rtb.RevoluteMDH(d=0.2),
            rtb.RevoluteMDH(alpha=pi/2,d=0.02),
            rtb.RevoluteMDH(a=0.25)
        ],tool = SE3.Tx(0.28), name="HelloWorld"
        )

        self.configuration_space = [0.42,0.1,0.33]
        self.name = ["joint_1", "joint_2", "joint_3"]
        self.mode = -1
        self.random_data = [0.1,0.1,0.1]
        self.pose_data = [0.0,0.0,0.0] 
        self.initial_guess = [0,0,0]
        self.current_pose = [0,0,0]
        self.flag = 0

        self.linear_vel = np.array([0.0,0.0,0.0])
        self.toggle_teleop_mode = 0

        self.buffer = Buffer()
        self.tf_callback= TransformListener(self.buffer, self)
        self.eff_fraame = "end_effector"
        self.refference_frame = "link_0"

        self.q_new = np.array([0.9, 0.0, 0.0])  # Initial joint angles
        self.last_pose = [0, 0, 0]
        self.stable_start_time = None
        
        
    def call_random_pos(self,call):
        while not self.random_pos_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Call Random Position Server Starting . . .")
        random_req = CallRandomPos.Request()
        random_req.is_call = call
        self.random_pos_client.call_async(random_req)


    def get_pos_eff(self):
        try:
            now = rclpy.time.Time()
            transform = self.buffer.lookup_transform(
                self.refference_frame , 
                self.eff_fraame,   
                now)
            pos= transform.transform.translation   
            self.current_pose = pos.x,pos.y,pos.z
        
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
    
    def end_effector_publisher(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'end_effector'
        msg.pose.position = Point(x=float(self.current_pose[0]), y=float(self.current_pose[1]), z=float(self.current_pose[2]))
        self.end_effector_pub.publish(msg)


    def chmod_server_callback(self,request,respond,random=None):
        self.mode = request.mode

        if self.mode == 1:
            # Extracting position from the service request
            self.pose_data[0] = request.pose.position.x 
            self.pose_data[1] = request.pose.position.y
            self.pose_data[2] = request.pose.position.z


            if (self.compute_pose(self.pose_data[0], self.pose_data[1], self.pose_data[2]) is not False and 
                self.check_possible_workspace(self.pose_data[0], self.pose_data[1], self.pose_data[2]) is not False):
                self.pose_publishing(self.compute_pose(self.pose_data[0],self.pose_data[1],self.pose_data[2]))
                respond.success = True
                respond.joint_pos.position = [float(value) for value in self.compute_pose(self.pose_data[0], self.pose_data[1], self.pose_data[2])]
                self.get_logger().info(f"pose = {self.pose_data} ,angle = {self.initial_guess}")
                
            else:
                respond.success = False
                respond.joint_pos.position = [self.pose_data[0],self.pose_data[1],self.pose_data[2]]
            return respond

        elif(self.mode == 2):
            print("mode 2")
            respond.success = True
            return respond
        
        elif(self.mode == 3):
            print("Auto mode has been started")
            respond.success = True
            return respond
    
    def cmd_vel_callback(self,msg: Twist):
        self.linear_vel = [msg.linear.x,msg.linear.y,msg.linear.z]
        self.toggle_teleop_mode = msg.angular.z
        # print(self.linear_vel)

    def random_pose_callback(self,msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.random_data = [x,y,z]

    
    def pose_publishing(self,pose_array):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(pose_array)):
            msg.position.append(pose_array[i])
            msg.name.append(self.name[i])
        self.pose_pub_.publish(msg)

    def timer_callback(self):
        self.get_pos_eff()
        self.end_effector_publisher()
        if self.mode == 2:
            self.velo_jacobian_compute(self.linear_vel,self.toggle_teleop_mode)
        elif self.mode == 3:
            self.get_pos_eff()  # Get the current position of the end-effector
            self.jacobian_compute(self.random_data[0],self.random_data[1],self.random_data[2])
            if self.flag == True:
                self.call_random_pos(True)
                self.get_logger().info('COMPLETE')
            else:
                self.get_logger().info(f'\n Current Position: {self.current_pose}' )
            
            

        


    # -------------------------------------------- Compute and Check the conditions---------------------------------------------- #
  
    def compute_pose(self,x,y,z):
        if (self.r_min**2 <= x**2 + y**2 + (z-0.2)**2 <= self.r_max**2 ):

            T_desired = SE3(x,y,z)
            q, *_  = self.robot_.ikine_LM(T_desired,mask=[1,1,1,0,0,0],q0=[0.0,0.0,0.0])
            return q
        else:
            return False
    

    def check_possible_workspace(self, x, y, z):
        return -self.r_min <= x <= self.r_max and -self.r_min <= y <= self.r_max and -self.r_min  <= z <= self.r_max 
    
    def jacobian_compute(self, x, y, z):
        T_desired = SE3(x, y, z)
        q_current = self.initial_guess

        T_current = self.robot_.fkine(q_current)
        delta_x = (T_desired.A - T_current.A)[:3, 3]  

        # Check if delta_x is smaller than the threshold
        if np.linalg.norm(delta_x) < 1e-06:
            # self.get_logger().info("Goal achieved! Error is sufficiently small.")
            self.flag = True
        else:
            self.flag = False

        J_trans = self.robot_.jacob0(q_current)[:3, :]

        condition_number = np.linalg.cond(J_trans)
        if condition_number > 1e6:  
            self.get_logger().warn(f"Jacobian near-singular (cond: {condition_number})")
            damping_factor = 1e-4
            J_damped = J_trans.T @ np.linalg.inv(J_trans @ J_trans.T + damping_factor * np.eye(3))
        else:
            J_damped = np.linalg.pinv(J_trans)

        try:
            dq = 0.1 * (J_damped @ delta_x)  
            q_new = q_current + dq
            self.pose_publishing(q_new)
            self.initial_guess = q_new 
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f"Jacobian inversion failed: {e}")

    def velo_jacobian_compute(self, v_desired,mode):

        q_current = self.initial_guess
        
        # mode == 1: Tranformaiton that reffered by Base frame
        if mode == 1:

            T_current = self.robot_.fkine(q_current)
            
            # Extract the rotation matrix from the current pose (end-effector to base frame)
            R_base_to_ee = T_current.R
            
            # Transform the desired velocity from the end-effector frame to the base frame
            v_desired_base = R_base_to_ee @ v_desired
        else:
            # Directly use to End effector
            v_desired_base = v_desired

        J_trans = self.robot_.jacob0(q_current)[:3, :]
        condition_number = np.linalg.cond(J_trans)

        if condition_number > 1e6:  # Singularity threshold 
            self.get_logger().warn(f"Jacobian near-singular (condition number: {condition_number}).")
            
        else:
            self.get_logger().info(f"\n Vx: {v_desired[0]} Vy: {v_desired[1]} Vx: {v_desired[2]} \n Current Position: {self.current_pose}")
    
        try:
            dq = np.linalg.pinv(J_trans) @ v_desired_base  

            q_new = q_current + dq * (1 / self.freq) 

            self.pose_publishing(q_new)

            self.initial_guess = q_new

        except np.linalg.LinAlgError as e:
            self.get_logger().error(f"Jacobian inversion failed: {e}")



        
        

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

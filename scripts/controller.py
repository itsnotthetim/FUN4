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

        self.create_subscription(PoseStamped,'target',self.pose_callback,10)
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
        # self.flag = 0

        # /end-effector
        self.buffer = Buffer()
        self.tf_callback= TransformListener(self.buffer, self)
        self.eff_fraame = "end_effector"
        self.refference_frame = "link_0"

        self.q_new = np.array([0.9, 0.0, 0.0])  # Initial joint angles
        self.target_position = np.array([0.34, 0.41, 0.2])  # Hypothetical target position
        self.learning_rate = 0.01  # Step size for updating angles
        self.last_pose = [0, 0, 0]
        self.stable_start_time = None
        
    def call_random_pos(self,call):
        while not self.random_pos_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Starting . . .")
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
            position = transform.transform.translation
            
            self.current_pose = position.x,position.y,position.z

            # self.get_logger().info(f"End Effector Position: {position.x}, {position.y}, {position.z}")
            #self.get_logger().info(f"End Effector Orientation: {orientation.x}, {orientation.y}, {orientation.z}, {orientation.w}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
    


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
    


    def pose_callback(self,msg: PoseStamped):
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
        # msg.position = pose_array
        # msg.name = self.name
        self.pose_pub_.publish(msg)

    def timer_callback(self):
        if self.mode == 3:
            check = self.detect_pose_stability()
            self.get_pos_eff()  # Get the current position of the end-effector
            self.jacobian_compute(self.random_data[0],self.random_data[1],self.random_data[2])
            if check == True:
                self.call_random_pos(True)
                self.get_logger().info('COMPLETE')
            else:
                self.get_logger().info(f'NOT YET rand:{self.random_data}' )
            
            

        


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
        if (self.r_min**2 <= x**2 + y**2 + (z-0.2)**2 <= self.r_max**2 ):

            T_desired = SE3(x,y,z)
            # q = self.custom_ikine(self.robot_,T_desired,self.initial_guess)
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

    def detect_pose_stability(self):
        if np.allclose(self.current_pose, self.last_pose, atol=1e-5):
            if self.stable_start_time is None:
                self.stable_start_time = time.time()  # Start timing
            elif time.time() - self.stable_start_time >= 1.0:
                # Pose hasn't changed for 2 seconds
                return True
        else:
            # Pose changed, reset timing
            self.last_pose = self.current_pose
            self.stable_start_time = None
        return False
        
        

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

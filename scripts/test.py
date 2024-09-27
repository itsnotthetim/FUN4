import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize
# Define the MDH Parameter


# robot_ = rtb.DHRobot(
#         [ 
#             rtb.RevoluteMDH(a=0),
#             rtb.RevoluteMDH(offset=pi/2,d=0.2),
#             rtb.RevoluteMDH(alpha=pi/2,offset=pi/2,d=0.1),
#             rtb.RevoluteMDH(a=0.25,d=-0.1)
#         ],tool = SE3.Tx(0.28), name="HelloWorld"
#         )
robot = rtb.DHRobot(
    [   
        rtb.RevoluteMDH(a=0),
        rtb.RevoluteMDH(alpha=-pi/2,offset=-pi/2,d=0.2),
        rtb.RevoluteMDH(a=0.7,offset=(3*pi)/4,d=0.0),
        rtb.RevoluteMDH(a=0.7,offset=-pi/4,d=0.0),
        rtb.RevoluteMDH(alpha=-pi/2,offset=pi/2,d=0.3),
        rtb.RevoluteMDH(alpha=pi/2,offset=pi,d=0.5)
    ],
    name = "RRR_Robot"
)
# robot_ = rtb.DHRobot(
#         [   
#             rtb.RevoluteMDH(a= 0.0  ,   alpha=0.0,     offset=pi/2,    d=0.2),
#             rtb.RevoluteMDH(a = 0.0,    alpha=pi/2 ,   offset=pi/2,    d=0.02),
#             rtb.RevoluteMDH(a=0.25)
#         ],tool = SE3.Tx(0.28), name="HelloWorld"
#         )
# # Print the MDH-Table
# print(robot_)

robot_1 = rtb.DHRobot(
        [   
            rtb.RevoluteMDH(a= 0.0 ,    alpha=0.0,  offset=0.0, d=0.2),
            rtb.RevoluteMDH(a= 0.0 ,    alpha=pi/2, offset=0.0, d=0.2),
            rtb.RevoluteMDH(a= 0.2 ,    alpha=0.0, offset=0.0, d=0.0),
        ], name="HelloWorld" #,tool = SE3.Tx(0.2)
        )
# Print the MDH-Table
print(robot_1)

desired_position = np.array([0.2, -0.2, 0.2])  # Example target position

# Create a transformation matrix for the target position
target_T = SE3(desired_position[0], desired_position[1], desired_position[2])
print(target_T)
# Use inverse kinematics to compute joint angles (q1, q2, q3)
solution = robot_1.ik_LM(target_T)[0]
print(solution)
# Check if the solver converged
# Extract joint angles
print(f"Joint angles found: q1 = {solution[0]}, q2 = {solution[1]}") #, q2 = {solution[1]}, q3 = {solution[2]}

# Now use forward kinematics to verify the end-effector position
actual_T = robot_1.fkine(solution)

# Extract the position part from the transformation matrix
actual_position = actual_T.t

print(f"Computed end-effector position: {actual_position}")
print(f"Desired end-effector position: {desired_position}")

# Check if the actual position matches the desired position
position_error = np.linalg.norm(desired_position - actual_position)
print(f"Position error: {position_error}")

if position_error < 1e-3:  # Set a tolerance for acceptable error
    print("The computed position matches the desired position.")
else:
    print("The computed position does not match the desired position.")




    

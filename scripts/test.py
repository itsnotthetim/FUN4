import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
# Define the MDH Parameter


robot_ = rtb.DHRobot(
        [ 
            rtb.RevoluteMDH(a=0),
            rtb.RevoluteMDH(offset=pi/2,d=0.2),
            rtb.RevoluteMDH(alpha=pi/2,offset=pi/2,d=0.1),
            rtb.RevoluteMDH(a=0.25,d=-0.1)
        ],tool = SE3.Tx(0.28), name="HelloWorld"
        )
# Print the MDH-Table
print(robot_)

T_desired = SE3(2,2,2)
q = robot_.ik_LM(T_desired)
# q = [0,0,0,0]
print(q)
# robot_.plot(q,block=True)

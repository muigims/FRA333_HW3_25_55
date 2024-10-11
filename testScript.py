# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย

from spatialmath import SE3
from HW3_utils import FKHW3
from math import pi

import roboticstoolbox as rtb
import numpy as np
from FRA333_HW3_25_55 import endEffectorJacobianHW3,checkSingularityHW3,computeEffortHW3

'''
ชื่อ_รหัส(ex: ธนวัฒน์_6541)                                                        
1.
2.
3.
'''

d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
q_initial = np.array([0.0, 0.0, 0.0])
q_singularity = np.array([0.0, pi/4, 3.13])

w_initial = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0] #(Fx, Fy, Fz, Tx, Ty, Tz)

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha = 0.0     ,a = 0.0      ,d = d_1    ,offset = pi ),
        rtb.RevoluteMDH(alpha = pi/2    ,a = 0.0      ,d = 0.0    ,offset = 0.0),
        rtb.RevoluteMDH(alpha = 0.0     ,a = -a_2     ,d = 0.0    ,offset = 0.0),
    ],
    tool = SE3([
    [0, 0, -1, -(a_3 + d_6)],
    [0, 1, 0, -d_5],
    [1, 0, 0, d_4],
    [0, 0, 0, 1]]),
    name = "3DOF_Robot"
)

#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
def checkEndEffectorJacobianHW3(q):
    print("-------------------check Jacobian ----------------------")
    # Manual calculation using function from FRA333_HW3_25_55
    J_manual = endEffectorJacobianHW3(q)

    # Robotic toolbox calculation
    J_toolbox = robot.jacobe(q)

    # Compare Jacobians
    print("Jacobian from manual calculation (FRA333_HW3_25_55):\n", J_manual)
    print("Jacobian from toolbox (roboticstoolbox):\n", J_toolbox)

    # Difference between the two
    J_diff = J_toolbox - J_manual
    print("Difference in Jacobian:\n", J_diff)

# Call the function to check the Jacobian
checkEndEffectorJacobianHW3(q_initial)

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def checkSingularityFunction(q):
    print("-------------------check Singularity ----------------------")
    # Manual singularity check using function from FRA333_HW3_25_55
    singularity_manual = checkSingularityHW3(q)

    # Robotic toolbox singularity check
    J_toolbox = robot.jacobe(q)
    J_linear_toolbox = J_toolbox[:3, :]
    manipularity = abs(np.linalg.det(J_linear_toolbox))
    epsilon = 0.001
    singularity_toolbox = manipularity < epsilon

    # Compare singularity results
    print("Singularity from manual calculation (FRA333_HW3_25_55):", singularity_manual)
    print("Singularity from toolbox (roboticstoolbox):", singularity_toolbox)

    if singularity_manual == singularity_toolbox:
        print("Singularity check matches!")
    else:
        print("Singularity check differs!")

# Call the function to check singularity
checkSingularityFunction(q_singularity)


#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def checkComputeEffortHW3(q, w):
    print("-------------------check ComputeEffort ----------------------")
    # Manual effort calculation using function from FRA333_HW3_25_55
    tau_manual = computeEffortHW3(q, w)

    # Robotic toolbox effort calculation
    J_toolbox = robot.jacobe(q)
    tau_toolbox = np.dot(J_toolbox.T, w)

    # Compare efforts
    print("Effort (tau) from manual calculation (FRA333_HW3_25_55):\n", tau_manual)
    print("Effort (tau) from toolbox (roboticstoolbox):\n", tau_toolbox)

    # Difference between the two
    tau_diff = tau_toolbox - tau_manual
    print("Difference in Effort (tau):\n", tau_diff)

# Call the function to check the effort
checkComputeEffortHW3(q_initial, w_initial)


#==============================================================================================================#
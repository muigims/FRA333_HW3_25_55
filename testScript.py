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
1.ทัศน์พล สินเมือง 65340500025
2.สิริมณี มณ๊เวศย์วโรดม 65340500055
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

# Define the joint angles to test
qs1 = [-1.91970470e-15, -8.35883143e-01, 2.80232546e+00]
qs2 = [-0.24866892, 0.22598268, -0.19647569]
qs3 = [1.70275090e-17, -1.71791355e-01, -1.95756090e-01]
qs4 = [0.0, 0.0, 0.0]


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
# ฟังก์ชันสำหรับเช็ค Singularities สำหรับ qs1, qs2, qs3 พร้อมแสดงค่า determinant
def checkSingularity():
    print("-------------------check Singularity ----------------------")

    # ฟังก์ชันเพื่อแสดงผล Singularities
    def printSingularityResult(q, name):
        # คำนวณ Jacobian ของหุ่นยนต์จากฟังก์ชันที่พัฒนาเอง
        J_manual = endEffectorJacobianHW3(q)
        J_linear_manual = J_manual[:3, :]  # ตัดส่วน Linear (3x3)

        # คำนวณ Jacobian จาก Robotic Toolbox
        J_toolbox = robot.jacobe(q)
        J_linear_toolbox = J_toolbox[:3, :]  # ตัดส่วน Linear (3x3)

        # คำนวณ determinant ของทั้งสอง Jacobian
        manipularity_manual = abs(np.linalg.det(J_linear_manual))
        manipularity_toolbox = abs(np.linalg.det(J_linear_toolbox))

        # คำนวณความแตกต่างระหว่างผลลัพธ์จากทั้งสอง
        J_diff = J_linear_toolbox - J_linear_manual

        # ตรวจสอบว่าอยู่ในสถานะ Singularity หรือไม่
        epsilon = 0.001
        singularity_manual = manipularity_manual < epsilon
        singularity_toolbox = manipularity_toolbox < epsilon

        # แสดงผลลัพธ์
        print(f"Results for {name}:")
        print("\nManual Calculation:")
        print(f"Jacobian Linear Part (3x3):\n{J_linear_manual}")
        print(f"Determinant: {manipularity_manual}")
        print(f"Singularity Status: {'Singularity Detected' if singularity_manual else 'No Singularity'}")

        print("\nRobotic Toolbox Calculation:")
        print(f"Jacobian Linear Part (3x3):\n{J_linear_toolbox}")
        print(f"Determinant: {manipularity_toolbox}")
        print(f"Singularity Status: {'Singularity Detected' if singularity_toolbox else 'No Singularity'}")

        print("\nDifference between Toolbox and Manual:")
        print(J_diff)
        print("\n")

    # ทดสอบ Singularities สำหรับชุดมุมต่าง ๆ
    printSingularityResult(qs1, "qs1")
    printSingularityResult(qs2, "qs2")
    printSingularityResult(qs3, "qs3")
    printSingularityResult(qs4, "qs4")

# เรียกฟังก์ชันเพื่อตรวจสอบ Singularities
checkSingularity()



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
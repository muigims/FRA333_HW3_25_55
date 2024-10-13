# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
import roboticstoolbox as rtb
import numpy as np
from math import pi
from spatialmath import SE3
from HW3_utils import FKHW3

'''
ชื่อ_รหัส(ธนวัฒน์_6461)
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

q = np.array([0.0, 0.0, 0.0])
q_singularity = np.array([0.0,0.0,0.0])
w_initial = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  #(Fx, Fy, Fz, Tx, Ty, Tz)



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
    name = "robot"
)
print(robot)
#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    # คำนวณ Forward Kinematics ด้วย FKHW3
    R, P, R_e, p_e = FKHW3(q)

    # สร้าง Jacobian ขนาด 6x3 สำหรับหุ่นยนต์ 3DOF
    # กำหนดเมทริกซ์ Jacobian ขนาด 6x3 (หุ่นยนต์มี 3 ข้อต่อ) ที่เต็มไปด้วยค่า 0 เพื่อเตรียมพร้อมสำหรับการคำนวณ โดย 6 แถว
    J = np.zeros((6, len(q)))
    
    # ลูปเพื่อคำนวณ Jacobian สำหรับแต่ละข้อต่อ (i=0, 1, 2)
    for i in range(len(q)):
        P_i = P[:, i]  # ตำแหน่งของข้อต่อที่ i จากเมทริกซ์ตำแหน่ง P
        Z_i = R[:, 2, i]  # แกน Z ของข้อต่อที่ i จากเมทริกซ์การหมุน R เพื่อใช้คำนวณความเร็วเชิงเส้นและเชิงมุมของข้อต่อนั้น ๆ

        # 3 แถวแรกของ Jacobian ส่วนของความเร็วเชิงเส้น (Linear velocity)
        J[:3, i] = np.cross(Z_i, p_e - P_i)@ R_e

        # 3 แถวหลังของ Jacobian ส่วนของความเร็วเชิงมุม (Angular velocity)
        J[3:, i] = Z_i@ R_e
        # print("Jacobian Matrix:")
    return J
J = endEffectorJacobianHW3(q)
print("-----------------answer 1 -------------------")
print("Jacobian from manual calculation:")
print(J)
print("Jacobian Matrix from toolbox:")
print(robot.jacobe(q))
print("Difference between library and custom Jacobian:")
print((robot.jacobe(q))-J)


#pass
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q: list[float]) -> bool:
    # คำนวณ Jacobian ของหุ่นยนต์
    J = endEffectorJacobianHW3(q)
    # ตัดเฉพาะส่วนของ Jacobian ที่เป็นความเร็วเชิงเส้น (3x3)
    # ตัดเฉพาะส่วนของ Jacobian ที่เป็นความเร็วเชิงเส้น (3x3)
    J_linear = J[:3, :]
    # คำนวณ Determinant ของ Jacobian ที่ลดรูปแล้ว
    manipularity = abs(np.linalg.det(J_linear))
    singularity = False
    
    # ตรวจสอบว่า determinant มีค่าน้อยกว่า epsilon หรือไม่
    epsilon = 0.001
    if manipularity < epsilon:
        singularity = True
  # หุ่นยนต์อยู่ในสถานะ Singularity
    return singularity
print("-----------------answer 2 -------------------")
print("Singularity :",checkSingularityHW3(q))
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q: list[float], w: list[float]) -> list[float]:
    # คำนวณ Jacobian
    J = endEffectorJacobianHW3(q)
    
    # คำนวณ Jacobian Transpose โดยใช้ np.dot() แทนการใช้ @
    J_T = np.dot(J.T, w)

    # คืนค่า tau
    return J_T

# เรียกใช้ฟังก์ชันทดสอบ
result = computeEffortHW3(q, w_initial)
print("-----------------answer 3 -------------------")
print("Computed Effort (tau):", result)

#==============================================================================================================#

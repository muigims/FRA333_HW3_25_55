# FRA333_HW3_25_55
 โปรเจกต์นี้เกี่ยวข้องกับการคำนวณ Jacobian, การตรวจสอบ Singularity และการคำนวณแรงบิด (Effort) ของหุ่นยนต์ 3DOF โดยใช้ Python และไลบรารี Robotics Toolbox รวมถึงการทดสอบผลลัพธ์ผ่านการคำนวณเชิงลึก

 ## **การติดตั้ง Environment**
 **ขั้นตอนการติดตั้ง**
 
 1.ติดตั้ง Python
 - ตรวจสอบว่าระบบมี Python เวอร์ชัน 3.8 ขึ้นไปแล้ว (หากยังไม่มีสามารถดาวน์โหลดได้จาก https://www.python.org)
 - ตรวจสอบเวอร์ชันของ Python โดยใช้คำสั่ง:
 ```
 python --version
 ```
 2.numpy: สำหรับการคำนวณเชิงตัวเลข เช่น เมทริกซ์และเวกเตอร์
  ```
 pip install numpy
  ```
 3.scipy: สำหรับฟังก์ชันทางคณิตศาสตร์และการประมวลผลทางวิทยาศาสตร์
  ```
 pip install scipy
  ```
4.Robotics Toolbox for Python
  ```
 pip install roboticstoolbox-python
  ```
5.SpatialMath ใช้สำหรับการจัดการกับการแปลงทางเรขาคณิต (SE3) เช่น เมทริกซ์การหมุนและการเคลื่อนที่ของหุ่นยนต์
 ```
 pip install spatialmath-python
  ```
6.ตรวจสอบว่าไฟล์ HW3_utils.py ,  FRA333_HW3_25_55.py  , testScript.py อยู่ในไดเรกทอรีเดียวกัน ก่อนรันโปรแกรม

**clone github**
1.Clone the repository
 ```
git clone https://github.com/muigims/FRA333_HW3_25_55.git
  ```
 ```
 cd FRA333_HW3-main/FRA333_HW3-main/FRA333_HW3_25_55
  ```
2. Create a Python Virtual Environment
เพื่อให้การจัดการ dependencies แยกจากระบบหลัก ให้สร้างและเปิดใช้งาน Virtual Environment
 ```
python -m venv env
env\Scripts\activate
  ```
# **ทฤษฎีที่เกี่ยวข้อง**
**1. Kinematics (จลนศาสตร์ของหุ่นยนต์)**
Kinematics เป็นการศึกษาเกี่ยวกับการเคลื่อนที่ของหุ่นยนต์โดยไม่สนใจแรงที่กระทำ แต่เน้นที่การเคลื่อนที่ของข้อต่อและตำแหน่งของ end-effector (ส่วนปลายของหุ่นยนต์) เราสามารถแบ่งออกเป็น 2 ส่วนหลัก ๆ:
1.1 Forward Kinematics (FK) – จลนศาสตร์เดินหน้า
Forward Kinematics ใช้สำหรับหาตำแหน่งและการหมุนของ end-effector เมื่อทราบมุมของข้อต่อแต่ละข้อ การคำนวณนี้จำเป็นสำหรับการคาดการณ์ตำแหน่งที่หุ่นยนต์จะเคลื่อนไปเมื่อปรับมุมของข้อต่อ

สมการทั่วไปของ FK:

$$T_{0,n} = T_{0,1} \cdot T_{1,2} \cdot \dots \cdot T_{n-1,n}$$

โดยที่:
- `T(i, j)` คือเมทริกซ์การแปลง (Transformation Matrix) ระหว่างข้อต่อ \(i\) และ \(j\).
---

**2. Jacobian Matrix (เมทริกซ์จาโคเบียน)**

Jacobian เป็นเมทริกซ์ที่แสดงความสัมพันธ์เชิงเส้นระหว่างความเร็วของข้อต่อและความเร็วของ end-effector 
ในเชิงลำดับชั้น โดยทั่วไปเมทริกซ์นี้มีขนาด `6 x n` (สำหรับหุ่นยนต์ที่มี `n` ข้อต่อ) ซึ่งแยกเป็นสองส่วน:
- **Linear velocity (ความเร็วเชิงเส้น):** ระบุการเคลื่อนที่เชิงตำแหน่งของ end-effector
- **Angular velocity (ความเร็วเชิงมุม):** ระบุการหมุนของ end-effector
- 
$$
\begin{bmatrix} 
\mathbf{v} \\ 
\boldsymbol{\omega} 
\end{bmatrix} 
= \mathbf{J}(\mathbf{q}) \cdot \dot{\mathbf{q}}
$$

### โดยที่:
- **v** = ความเร็วเชิงเส้นของ end-effector  
- **ω** = ความเร็วเชิงมุมของ end-effector  
- **q** = เวกเตอร์มุมของข้อต่อแต่ละข้อ  
- **ẏ** = เวกเตอร์ความเร็วเชิงมุมของข้อต่อแต่ละข้อ  

### Jacobian มีความสำคัญเพราะใช้สำหรับ:
- คำนวณความเร็วของ end-effector จากความเร็วของข้อต่อ
- คำนวณแรงบิดที่ข้อต่อเมื่อมีแรงกระทำกับ end-effector


**3. Singularity (ภาวะเอกฐานของหุ่นยนต์)**

ภาวะ Singularity เกิดขึ้นเมื่อหุ่นยนต์สูญเสียอิสระในการเคลื่อนที่ (Degrees of Freedom) ในบางทิศทาง ซึ่งอาจทำให้หุ่นยนต์ไม่สามารถเคลื่อนที่ไปในทิศทางที่ต้องการได้หรือเคลื่อนได้อย่างผิดพลาด

### การตรวจสอบ Singularity
เราสามารถตรวจสอบภาวะ Singularity ได้โดยดูจากค่า determinant ของเมทริกซ์ Jacobian ส่วนความเร็วเชิงเส้น (Linear Velocity)

$$
\det(\mathbf{J}_{\text{linear}}) \approx 0
$$

หากค่า determinant มีค่าใกล้ 0 แสดงว่าหุ่นยนต์อยู่ในภาวะ Singularity

### ตัวอย่างผลกระทบจาก Singularity:
- หุ่นยนต์อาจไม่สามารถหมุนได้ในบางทิศทาง  
- การเคลื่อนที่อาจมีการเปลี่ยนแปลงผิดคาด เช่น การกระตุกหรือผิดตำแหน่ง

**4. Effort (Torque) Calculation – การคำนวณแรงบิดที่ข้อต่อ**

Torque หรือ Effort คือแรงบิดที่ข้อต่อต้องลงทุนนุนเพื่อต้องใช้ในการขยับเมื่อมีแรงกระทำกับ end-effector  
วิธีการคำนวณใช้ Jacobian Transpose เพื่อแปลงแรงที่กระทำกับ end-effector ไปเป็นแรงบิดที่ข้อต่อ

$$
\tau = \mathbf{J}^T \cdot \mathbf{W}
$$

### โดยที่:
- **τ** = เวกเตอร์แรงบิดที่ข้อต่อ  
- **W** = แรงและทอร์กที่กระทำกับ end-effector  
- **Jᵀ** = Transpose ของ Jacobian  

# **Implement code**

เริ่มจากการนำตัวแปรที่ได้จากไฟล์ HW3_utils.py มาใส่ แล้วทำการสร้างตาราง MDH Parameters จาก roboticstoolbox
 ```
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
```

### DHRobot: robot, 3 joints (RRR), dynamics, modified DH parameters

| \(a_{j-1}\) | \(\alpha_{j-1}\) | \(\theta_j\)        | \(d_j\) |
|-------------|------------------|---------------------|--------|
| 0.0         | 0.0°             | \(q_1 + 180^\circ\) | 0.0892 |
| 0.0         | 90.0°            | \(q_2\)             | 0.0    |
| -0.425      | 0.0°             | \(q_3\)             | 0.0    |

### tool
| **t**                      | **rpy/xyz**                     |
|----------------------------|----------------------------------|
| -0.47, -0.093, 0.11        | \(0^\circ, -90^\circ, 0^\circ\) |



## ข้อที่ 1 การคำนวณหา Jacobian
```
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
```
จากนั้นทำการแสดงผลค่าที่ได้จากการคำนวณ
```
#define q parameter
q = np.array([0.0, 0.0, 0.0])
J = endEffectorJacobianHW3(q)
print("-----------------answer 1 -------------------")
print("Jacobian from manual calculation:")
print(J)
```

![image](https://github.com/user-attachments/assets/01bd340f-0cbe-43b2-af21-4df5c8be4bf8)

### **ตรวจคำตอบ**
 
 จากค่า J ที่ได้จาก endEffectorJacobianHW3(q) ใน FRA333_HW3_25_55.py
 ```
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
```
จะได้ผลลัพธ์

![image](https://github.com/user-attachments/assets/bd633e4a-9d89-4365-9ec3-6a96acc8ffa7)

เมื่อนำมาเปรียบเทียบกันโดยหาผลต่าง ทำให้ทราบว่า ค่าที่ได้จากการคำนวณเองและจาก robotictoolbox มีค่าเท่ากัน เนื่องจากผลจต่างมีค่าเท่ากับ 0 

## ข้อที่ 2 การหา Singularity
เนื่องจาก การจะเกิด Singularity ต้องมีค่า det อยู่ในช่วง มากกว่า 0 แต่น้อยกว่า 0.001 ตามโจทย์
```
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
```
จากนั้นทำการแสดงผลค่าที่ได้จากการคำนวณ
```
print("-----------------answer 2 -------------------")
print("Singularity :",checkSingularityHW3(q))
```

![image](https://github.com/user-attachments/assets/61a514f3-9f5a-4d97-8145-e8b1676b2835)

### **ตรวจคำตอบ**
ในการตรวจคำตอบ ผู้จัดทำได้ทำการหาค่า q ที่จะทำให้แขนกลอยู่ในสถานะ Singularity ได้ 3 ค่าเพื่อทำการทดสอบ คือ qs1,qs2,qs3 และใส่ค่าเริ่มต้น คือ qs4 เพื่อทำการทดสอบ
 ```
# Define the joint angles to test
qs1 = [-1.91970470e-15, -8.35883143e-01, 2.80232546e+00]
qs2 = [-0.24866892, 0.22598268, -0.19647569]
qs3 = [1.70275090e-17, -1.71791355e-01, -1.95756090e-01]
qs4 = [0.0, 0.0, 0.0]
```
```
def checkSingularity():
    print("-------------------check Singularity ----------------------")

    # ฟังก์ชันเพื่อแสดงผล Singularities
    def printSingularityResult(q, name):
        # คำนวณ Jacobian ของหุ่นยนต์
        J = endEffectorJacobianHW3(q)
        
        # ตัดเฉพาะส่วนที่เป็น Jacobian เชิงเส้น (3x3)
        J_linear = J[:3, :]

        # คำนวณ determinant ของ Jacobian เชิงเส้น
        manipularity = abs(np.linalg.det(J_linear))
        epsilon = 0.001

        # ตรวจสอบค่า singularity
        singularity = manipularity < epsilon
        
        # แสดงผลลัพธ์
        print(f"Results for {name}:")
        print(f"Jacobian Linear Part (3x3):\n{J_linear}")
        print(f"Determinant: {manipularity}")
        print(f"Singularity Status: {'Singularity Detected' if singularity else 'No Singularity'}")
        print("\n")

    # Test qs1
    printSingularityResult(qs1, "qs1")
    # Test qs2
    printSingularityResult(qs2, "qs2")
    # Test qs3
    printSingularityResult(qs3, "qs3")
    # Test qs4
    printSingularityResult(qs4, "qs4")

# เรียกฟังก์ชันเพื่อเช็ค Singularities
checkSingularity()
```
จะได้ผลลัพธ์

![image](https://github.com/user-attachments/assets/135f77db-41d2-48c9-8285-34534f4391f4)

## ข้อที่ 3 Computed Effort
```
def computeEffortHW3(q: list[float], w: list[float]) -> list[float]:
    # คำนวณ Jacobian
    J = endEffectorJacobianHW3(q)
    
    # คำนวณ Jacobian Transpose โดยใช้ np.dot() แทนการใช้ @
    J_T = np.dot(J.T, w)

    # คืนค่า tau
    return J_T
```
จากนั้นทำการแสดงผลค่าที่ได้จากการคำนวณ
```
result = computeEffortHW3(q, w_initial)
print("-----------------answer 3 -------------------")
print("Computed Effort (tau):", result)
```
จะได้ผลลัพธ์

![image](https://github.com/user-attachments/assets/492c8c2f-e7e5-4311-a908-3564c27eed64)

### **ตรวจคำตอบ**
 การตรวจคำตอบ ทำได้โดยการ นำค่า tau ที่ได้จากการคำนวณมาเปรียบเทียบกับค่า tau ที่ได้จาก robotictoolbox
```
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
```
จะได้ผลลัพธ์

![image](https://github.com/user-attachments/assets/807b44d7-bca2-4465-81fe-0de1abac7a52)

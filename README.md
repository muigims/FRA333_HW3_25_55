# FRA333_HW3_6525_6555
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
---

**หมายเหตุ หากไม่สามารถลง robotictoolbox ใน window ได้ ให้ทำการลงบน ระบบปฏิบัติการ linux แทน**

**clone github**
1.Clone the repository หรือ ทำการ Download zip บน github แล้วแตกไฟล์
 ```
git clone https://github.com/muigims/FRA333_HW3_25_55.git
  ```
2.เปิดโปรแกรม Visual Studio Code แล้วทำการ open folder FRA333_HW3_6525_6555 (ที่อยู่ไฟล์ตามตำแหน่งที่เลือกมา)
3.ทำการรันไฟล์ FRA333_HW3_25_55.py และ testScript.py ตามลำดับ 
---
# **ทฤษฎีที่เกี่ยวข้อง**
**1. Kinematics (จลนศาสตร์ของหุ่นยนต์)**
Kinematics เป็นการศึกษาเกี่ยวกับการเคลื่อนที่ของหุ่นยนต์โดยไม่สนใจแรงที่กระทำ แต่เน้นที่การเคลื่อนที่ของข้อต่อและตำแหน่งของ end-effector (ส่วนปลายของหุ่นยนต์) เราสามารถแบ่งออกเป็น 2 ส่วนหลัก ๆ:
1.1 Forward Kinematics (FK) – จลนศาสตร์เดินหน้า
Forward Kinematics ใช้สำหรับหาตำแหน่งและการหมุนของ end-effector เมื่อทราบมุมของข้อต่อแต่ละข้อ การคำนวณนี้จำเป็นสำหรับการคาดการณ์ตำแหน่งที่หุ่นยนต์จะเคลื่อนไปเมื่อปรับมุมของข้อต่อ

สมการทั่วไปของ FK:

$$T_{0,n} = T_{0,1} \cdot T_{1,2} \cdot \dots \cdot T_{n-1,n}$$

โดยที่:
- `T(i, j)` คือเมทริกซ์การแปลง (Transformation Matrix) ระหว่างข้อต่อ \(i\) และ \(j\).

**2. Jacobian Matrix (เมทริกซ์จาโคเบียน)**
Jacobian Matrix เป็นเมทริกซ์ที่แสดงความสัมพันธ์ระหว่างความเร็วเชิงมุมของข้อต่อ และความเร็วของ end-effector ของหุ่นยนต์ โดยสามารถเขียนเป็นสมการได้ดังนี้:


$$
\
\dot{X} = J(q) \cdot \dot{q}
\
$$

ในเชิงลำดับชั้น โดยทั่วไปเมทริกซ์นี้มีขนาด `6 x n` (สำหรับหุ่นยนต์ที่มี `n` ข้อต่อ) ซึ่งแยกเป็นสองส่วน:
- **Linear velocity (ความเร็วเชิงเส้น):** ระบุการเคลื่อนที่เชิงตำแหน่งของ end-effector
- **Angular velocity (ความเร็วเชิงมุม):** ระบุการหมุนของ end-effector

  
$$
\begin{bmatrix} 
\mathbf{v} \\ 
\boldsymbol{\omega} 
\end{bmatrix} 
= \mathbf{J}(\mathbf{q}) \cdot \dot{\mathbf{q}}
$$

### โดยที่:
- **$v$** = ความเร็วเชิงเส้นของ end-effector  
- **$ω$** = ความเร็วเชิงมุมของ end-effector  
- **$q$** = เวกเตอร์มุมของข้อต่อแต่ละข้อ  
- **$\dot{q}$** = เวกเตอร์ความเร็วเชิงมุมของข้อต่อแต่ละข้อ  

### การหาค่า \($J_v$\) ของหุ่นยนต์ 3R

**Translational Jacobian (\($J_v$\))** แสดงความสัมพันธ์ระหว่างความเร็วของข้อต่อกับความเร็วเชิงเส้นของ end-effector สำหรับหุ่นยนต์ 3R การคำนวณ \($J_v$\) สามารถทำได้ดังนี้:

1. **Revolute Joint Contribution (ผลกระทบจากข้อต่อหมุน):**  
   สำหรับแต่ละข้อต่อเชิงหมุน \($i$\) ผลกระทบต่อความเร็วเชิงเส้นของ end-effector สามารถคำนวณได้โดยใช้ **Cross Product** ระหว่างแกนการหมุน \($z_i$\) และเวกเตอร์จากข้อต่อไปยังตำแหน่งของ end-effector:
   
$$
   \
   J_v = z_i \times (p_e - p_i)
   \
$$

   **โดยที่:**
   - \($z_i$\) คือแกนการหมุนของข้อต่อที่ \($i$\) (มักเป็นแกน \($z$\) ของเฟรมข้อต่อนั้น)  
   - \($p_e$\) คือตำแหน่งของ end-effector (ปลายหุ่นยนต์)  
   - \($p_i$\) คือตำแหน่งของข้อต่อที่ \($i$\)  

3. **Jv สำหรับหลายข้อต่อ:**  
   สำหรับหุ่นยนต์ 3R เราจะคำนวณ \($J_{v1}, J_{v2}, J_{v3}$\) ซึ่งแสดงผลกระทบจากข้อต่อที่ 1, 2 และ 3 ตามลำดับต่อความเร็วเชิงเส้นของ end-effector

4. **รูปแบบเมทริกซ์:**  
   **Translational Jacobian** ของหุ่นยนต์ 3R จะสร้างขึ้นโดยการรวมผลลัพธ์จากแต่ละข้อต่อดังนี้:
   
$$
   \
   J_v =
   \begin{bmatrix}
   z_1 \times (p_e - p_1) & z_2 \times (p_e - p_2) & z_3 \times (p_e - p_3)
   \end{bmatrix}
   \
$$

### การหาค่า \($J_\omega$\) ของหุ่นยนต์ 3R

**Angular Velocity Jacobian (\($J_\omega$\))** แสดงความสัมพันธ์ระหว่างความเร็วเชิงมุมของข้อต่อกับความเร็วเชิงมุมของ end-effector:

$$
\
J_\omega =
\begin{bmatrix}
z_1 & z_2 & z_3
\end{bmatrix}
\
$$


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
---
# **Implement code**
จากไฟล์ `HW3_utils.py`  ที่ได้รับมา ทำให้ได้ค่า return ออกมา 4 ค่า ได้แก่ $R,p,R_e,P_e$
เริ่มจากการนำตัวแปรที่ได้จากไฟล์ `HW3_utils.py` มาใส่ แล้วทำการสร้างตาราง MDH Parameters จาก roboticstoolbox
![image](https://github.com/user-attachments/assets/76930ad5-9368-479a-b89a-e39e9e66a93c)

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
จากสูตรด้านบน ทำให้เราได้ jaconbian matrix ที่เทีนยบกับ fram 0 จึงต้องทำการเปลี่ยน fram เทียบ fraam e ซึ่งทำได้โดยการนำ `z_i` มาคูณกับ rotation matrix frame 0 เทียบกับ e `(R_e)` 

- **Input:**
  - `q: list[float]` – เวกเตอร์แสดงมุมของข้อต่อ เช่น `[q1, q2, q3]`

- **Output:**
  - `J: list[float]` – Jacobian Matrix ขนาด \$(6 \times 3\)$ ซึ่งแสดงความสัมพันธ์ระหว่างความเร็วของข้อต่อและความเร็วของ end-effector

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
print("Jacobian from manual calculation:")
print(J)
```
### **จะได้ผลลัพธ์**

#### Jacobian from manual calculation:

$$
\
J = 
\begin{bmatrix}
8.9943 \times 10^{-1} & -4.9379 \times 10^{-17} & -2.3356 \times 10^{-17} \\
-1.1682 \times 10^{-16} & -8.9943 \times 10^{-1} & -4.7443 \times 10^{-1} \\
-1.0900 \times 10^{-1} & -9.3000 \times 10^{-2} & -9.3000 \times 10^{-2} \\
1.2246 \times 10^{-16} & 1.0000 \times 10^{0} & 1.0000 \times 10^{0} \\
1.0000 \times 10^{0} & -6.1232 \times 10^{-17} & -6.1232 \times 10^{-17} \\
-6.1232 \times 10^{-17} & 6.1232 \times 10^{-17} & 6.1232 \times 10^{-17}
\end{bmatrix}
\
$$

### **ตรวจคำตอบ**
 
 จากค่า J ที่ได้จาก endEffectorJacobianHW3(q) ใน FRA333_HW3_25_55.py

 ```
def checkEndEffectorJacobianHW3(q):
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
### **จะได้ผลลัพธ์**
#### Jacobian from manual calculation (FRA333_HW3_25_55):

$$
\
\begin{bmatrix}
8.9943 \times 10^{-1} & -4.9379 \times 10^{-17} & -2.3356 \times 10^{-17} \\
-1.1682 \times 10^{-16} & -8.9943 \times 10^{-1} & -4.7443 \times 10^{-1} \\
-1.0900 \times 10^{-1} & -9.3000 \times 10^{-2} & -9.3000 \times 10^{-2} \\
1.2246 \times 10^{-16} & 1.0000 \times 10^{0} & 1.0000 \times 10^{0} \\
1.0000 \times 10^{0} & -6.1232 \times 10^{-17} & -6.1232 \times 10^{-17} \\
-6.1232 \times 10^{-17} & 6.1232 \times 10^{-17} & 6.1232 \times 10^{-17}
\end{bmatrix}
\
$$

#### Jacobian from toolbox (roboticstoolbox):

$$
\
\begin{bmatrix}
8.9943 \times 10^{-1} & 0.0000 \times 10^{0} & 0.0000 \times 10^{0} \\
-5.5074 \times 10^{-35} & -8.9943 \times 10^{-1} & -4.7443 \times 10^{-1} \\
-1.0900 \times 10^{-1} & -9.3000 \times 10^{-2} & -9.3000 \times 10^{-2} \\
1.0000 \times 10^{0} & 0.0000 \times 10^{0} & 0.0000 \times 10^{0} \\
1.0000 \times 10^{0} & 0.0000 \times 10^{0} & 0.0000 \times 10^{0} \\
0.0000 \times 10^{0} & 0.0000 \times 10^{0} & 1.0000 \times 10^{0}
\end{bmatrix}
\
$$

#### Difference in Jacobian:

$$
\
\begin{bmatrix}
0.0000 \times 10^{0} & 4.9379 \times 10^{-17} & 2.3356 \times 10^{-17} \\
6.1748 \times 10^{-17} & 0.0000 \times 10^{0} & 5.5511 \times 10^{-17} \\
5.5511 \times 10^{-17} & 5.5511 \times 10^{-17} & 2.7756 \times 10^{-17} \\
0.0000 \times 10^{0} & 0.0000 \times 10^{0} & 0.0000 \times 10^{0} \\
-6.1232 \times 10^{-17} & 6.1232 \times 10^{-17} & 0.0000 \times 10^{0} \\
6.1232 \times 10^{-17} & -6.1232 \times 10^{-17} & -6.1232 \times 10^{-17}
\end{bmatrix}
\
$$

เมื่อนำมาเปรียบเทียบกันโดยหาผลต่าง ทำให้ทราบว่า ค่าที่ได้จากการคำนวณเองและจาก robotictoolbox มีค่าเท่ากัน เนื่องจากผลต่างมีค่าเทียบเท่า 0 

## ข้อที่ 2 การหา Singularity
ทำการสุ่มหาค่า q ที่ทำให้เกิด singularity โดยค่าที่นำมาทดลองคือ
 ```
qs1 = [-1.91970470e-15, -8.35883143e-01, 2.80232546e+00]
qs2 = [-0.24866892, 0.22598268, -0.19647569]
qs3 = [1.70275090e-17, -1.71791355e-01, -1.95756090e-01]
qs4 = [0.0, 0.0, 0.0]
 ```
- **Input:**
  - `q: list[float]` – เวกเตอร์แสดงมุมของข้อต่อ เช่น `[q1, q2, q3]`

- **Output:**
  - **ผลลัพธ์เป็น Boolean:** – แสดงว่าหุ่นยนต์อยู่ในสถานะ Singularity หรือไม่
    - `True`: หุ่นยนต์อยู่ในสถานะ Singularity  
    - `False`: หุ่นยนต์ไม่ได้อยู่ในสถานะ Singularity
   
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
print("Singularity1 :",checkSingularityHW3(qs1))
print("Singularity2 :",checkSingularityHW3(qs2))
print("Singularity3 :",checkSingularityHW3(qs3))
print("Singularity4 :",checkSingularityHW3(qs4))
```

### **จะได้ผลลัพธ์**
- **Singularity1** : `True`  
- **Singularity2** : `True`  
- **Singularity3** : `True`  
- **Singularity4** : `False`

ความหมายของผลลัพธ์
- หากค่าเป็น `True` หรือ 1 หมายความว่าหุ่นยนต์อยู่ในสถานะ **Singularity** ซึ่งอาจเกิดข้อจำกัดในเชิงการเคลื่อนไหว
- หากค่าเป็น `False` หรือ 0 หมายความว่าหุ่นยนต์ **ไม่ได้อยู่ในสถานะ Singularity** และสามารถเคลื่อนไหวได้อย่างอิสระในตำแหน่งนั้น ๆ

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

```
### **จะได้ผลลัพธ์**

#### Results for qs1:

##### Manual Calculation:
##### Jacobian Linear Part (3x3):

$$
\
\begin{bmatrix}
1.63092362e-02 & -7.47610646e-18 & -2.33559107e-17 \\
-1.00579509e-01 & -7.36555128e-02 & -4.74429999e-01 \\
4.20090740e-02 & 4.84383615e-02 & -9.30000031e-02
\end{bmatrix}
\
$$

##### Determinant: 0.00048651388752052766  
##### Singularity Status: Singularity Detected

##### Robotic Toolbox Calculation:
##### Jacobian Linear Part (3x3):

$$
\
\begin{bmatrix}
0.01630924 & 0.00000000 & 0.00000000 \\
-0.10057951 & -0.07365551 & -0.47443 \\
0.04200907 & 0.04843836 & -0.093
\end{bmatrix}
\
$$

##### Determinant: 0.0004865138888846027  
##### Singularity Status: Singularity Detected

##### Difference between Toolbox and Manual:

$$
\
\begin{bmatrix}
7.97972799e-17 & 7.47610646e-18 & 2.33559107e-17 \\
1.45909992e-10 & 2.27564703e-10 & -1.30026162e-09 \\
7.45823320e-10 & 5.70952723e-10 & 3.10985616e-09
\end{bmatrix}
\
$$

---
#### Results for qs2:

##### Manual Calculation:
##### Jacobian Linear Part (3x3):

$$
\
\begin{bmatrix}
8.85673870e-01 & -5.65866492e-17 & -2.62649613e-17 \\
-3.21579522e-03 & -8.91253311e-01 & -4.74430016e-01 \\
-1.08952552e-01 & -1.75965973e-01 & -9.30000019e-02
\end{bmatrix}
\
$$

##### Determinant: 0.0005287293674209878  
##### Singularity Status: Singularity Detected

##### Robotic Toolbox Calculation:
##### Jacobian Linear Part (3x3):

$$
\
\begin{bmatrix}
0.88567385 & -0.00000000 & 0.00000000 \\
-0.0032158 & -0.89125328 & -0.47443 \\
-0.10895255 & -0.17596597 & -0.093
\end{bmatrix}
\
$$

##### Determinant: 0.0005287293084899891  
##### Singularity Status: Singularity Detected

##### Difference between Toolbox and Manual:

$$
\
\begin{bmatrix}
-2.36743580e-08 & 5.65866492e-17 & 2.62649613e-17 \\
6.43629204e-12 & 3.00508782e-08 & 1.60838307e-08 \\
-4.45042739e-10 & 3.59153099e-09 & 1.89488747e-09
\end{bmatrix}
\
$$

---
#### Results for qs3:

##### Manual Calculation:
##### Jacobian Linear Part (3x3):

$$
\
\begin{bmatrix}
8.94905028e-01 & -4.38207280e-17 & -2.33558489e-17 \\
3.91667262e-02 & -8.91312897e-01 & -4.74430011e-01 \\
-1.01720048e-01 & -1.75665999e-01 & -9.29999984e-02
\end{bmatrix}
\
$$

##### Determinant: 0.0004019232076573776  
##### Singularity Status: Singularity Detected

##### Robotic Toolbox Calculation:
##### Jacobian Linear Part (3x3):

$$
\
\begin{bmatrix}
0.89490503 & -0.00000000 & 0.00000000 \\
0.03916673 & -0.89131288 & -0.47443 \\
-0.10172005 & -0.175666 & -0.093
\end{bmatrix}
\
$$


##### Determinant: 0.00040192318994076645  
##### Singularity Status: Singularity Detected

##### Difference between Toolbox and Manual:

$$
\
\begin{bmatrix}
-1.11022302e-16 & 4.38207280e-17 & 2.33558489e-17 \\
-7.81986351e-11 & 2.10001000e-08 & 1.11740912e-08 \\
2.54416412e-09 & -3.00710165e-09 & -1.61173239e-09
\end{bmatrix}
\
$$

---
#### Results for qs4:

##### Manual Calculation:
##### Jacobian Linear Part (3x3):

$$
\
\begin{bmatrix}
8.99430000e-01 & -4.93795981e-17 & -2.33558524e-17 \\
-1.16822737e-16 & -8.99430000e-01 & -4.74430000e-01 \\
-1.09000000e-01 & -9.30000000e-02 & -9.30000000e-02
\end{bmatrix}
\
$$

##### Determinant: 0.03554997075000001  
##### Singularity Status: No Singularity

##### Robotic Toolbox Calculation:
##### Jacobian Linear Part (3x3):

$$
\
\begin{bmatrix}
8.99430000e-01 & 0.00000000e+00 & 0.00000000e+00 \\
-5.50742035e-17 & -8.99430000e-01 & -4.74430000e-01 \\
-1.09000000e-01 & -9.30000000e-02 & -9.30000000e-02
\end{bmatrix}
\
$$

##### Determinant: 0.03554997074999999  
##### Singularity Status: No Singularity

##### Difference between Toolbox and Manual:

$$
\
\begin{bmatrix}
0.00000000e+00 & 4.93795981e-17 & 2.33558524e-17 \\
6.17485337e-17 & 0.00000000e+00 & -5.55111512e-17 \\
-5.55111512e-17 & -5.55111512e-17 & -2.77555756e-17
\end{bmatrix}
\
$$

จากผลลัพธ์ข้างต้น พบว่าหุ่นยนต์จะอยู่ในสถานะ Singularity เมื่อค่า determinant มีค่าน้อยกว่า 0.001 และจะไม่อยู่ในสถานะ Singularity หากค่า determinant มากกว่า 0.001 และเมื่อเช็คจากการนำค่าที่คำนวณเองมาเปรียบเทียบกับค่าที่คำนวณจาก robotictoolbox ทำให้ด้ค่าออกมาเทีัยบเท่ากับ 0 

## ข้อที่ 3 Computed Effort

- **Input:**
  - `q: list[float]` – เวกเตอร์แสดงมุมของข้อต่อ เช่น `[q1, q2, q3]`
  - `w: list[float]` – แรงกระทำและโมเมนต์ที่กระทำกับ end-effector เช่น `[Fx, Fy, Fz, Tx, Ty, Tz]`

- **Output:**
  - `tau: list[float]` – เวกเตอร์แสดงแรงบิดที่ต้องใช้กับแต่ละข้อต่อ เพื่อรักษาสมดุลกับแรงที่กระทำบน end-effector

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
print("Computed Effort (tau):", result)
```
### **จะได้ผลลัพธ์**

#### Computed Effort (tau):

$$
\
[0.0 \quad 0.0 \quad 0.0]
\
$$

### **ตรวจคำตอบ**
โดยการหาค่า effort ทำได้โดยการ หาค่า q และ w
 การตรวจคำตอบ ทำได้โดยการ นำค่า tau ที่ได้จากการคำนวณมาเปรียบเทียบกับค่า tau ที่ได้จาก robotictoolbox 
```
def checkComputeEffortHW3(q, w):
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
### **จะได้ผลลัพธ์**


#### Effort (tau) from manual calculation (FRA333_HW3_25_55):
$$
\
\begin{bmatrix}
  0.57243 & -2.07786 & -1.22786
\end{bmatrix}
\
$$

#### Effort (tau) from toolbox (roboticstoolbox):

$$
\
\begin{bmatrix}
  0.57243 & -2.07786 & -1.22786
\end{bmatrix}
\
$$

#### Difference in Effort (tau):

$$
\
\begin{bmatrix}
 -1.11022302e-16 & 0.00000000e+00 & -4.44089210e-16
\end{bmatrix}
\
$$

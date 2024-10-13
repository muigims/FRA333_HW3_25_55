# FRA333_HW3_25_55
 โปรเจกต์นี้เกี่ยวข้องกับการคำนวณ Jacobian, การตรวจสอบ Singularity และการคำนวณแรงบิด (Effort) ของหุ่นยนต์ 3DOF โดยใช้ Python และไลบรารี Robotics Toolbox รวมถึงการทดสอบผลลัพธ์ผ่านการคำนวณเชิงลึก

 ## **การติดตั้ง Environment**
 *ขั้นตอนการติดตั้ง**
 
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

$T_{0,n} = T_{0,1} \cdot T_{1,2} \cdot \dots \cdot T_{n-1,n}$

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

## ** Implement code**
ข้อที่ 1 การคำนวณหา Jacobian
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
ทำการตรวจคำตอบด้วยการนำค่าที่ได้จากการคำนวณโดย roboticstoolbox มาเปรียบเทียบโดยการนำมาหาผลต่าง
```
print("Jacobian Matrix from toolbox:")
print(robot.jacobe(q))
print("Difference between library and custom Jacobian:")
print((robot.jacobe(q))-J)
```
ทำให้ได้ผลลัพธ์






 # **ตรวจคำตอบ**

 1. เริ่มจากการสร้างตาราง MDH Parameters จาก roboticstoolbox
 ```
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




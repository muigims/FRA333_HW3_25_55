# FRA333_HW3_25_55
 โปรเจกต์นี้เกี่ยวข้องกับการคำนวณ Jacobian, การตรวจสอบ Singularity และการคำนวณแรงบิด (Effort) ของหุ่นยนต์ 3DOF โดยใช้ Python และไลบรารี Robotics Toolbox รวมถึงการทดสอบผลลัพธ์ผ่านการคำนวณเชิงลึก

 # **การติดตั้ง Environment**
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

# **clone github**
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
1. Kinematics (จลนศาสตร์ของหุ่นยนต์)
Kinematics เป็นการศึกษาเกี่ยวกับการเคลื่อนที่ของหุ่นยนต์โดยไม่สนใจแรงที่กระทำ แต่เน้นที่การเคลื่อนที่ของข้อต่อและตำแหน่งของ end-effector (ส่วนปลายของหุ่นยนต์) เราสามารถแบ่งออกเป็น 2 ส่วนหลัก ๆ:
1.1 Forward Kinematics (FK) – จลนศาสตร์เดินหน้า
Forward Kinematics ใช้สำหรับหาตำแหน่งและการหมุนของ end-effector เมื่อทราบมุมของข้อต่อแต่ละข้อ การคำนวณนี้จำเป็นสำหรับการคาดการณ์ตำแหน่งที่หุ่นยนต์จะเคลื่อนไปเมื่อปรับมุมของข้อต่อ

สมการทั่วไปของ FK:
$$
\mathbf{T}_{0,n} = \mathbf{T}_{0,1} \cdot \mathbf{T}_{1,2} \cdot \ldots \cdot \mathbf{T}_{n-1,n}
$$


![image](https://github.com/user-attachments/assets/7f0b64c0-2743-43b9-a228-7a3ffcccbafa)

โดยที่:
- `T(i, j)` คือเมทริกซ์การแปลง (Transformation Matrix) ระหว่างข้อต่อ \(i\) และ \(j\).

---

## 2. Jacobian Matrix (เมทริกซ์จาโคเบียน)
Jacobian แสดงความสัมพันธ์ระหว่างความเร็วเชิงเส้นและความเร็วเชิงมุมของ end-effector กับความเร็วเชิงมุมของข้อต่อแต่ละข้อ:


2. Jacobian Matrix (เมทริกซ์จาโคเบียน)


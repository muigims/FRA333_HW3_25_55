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


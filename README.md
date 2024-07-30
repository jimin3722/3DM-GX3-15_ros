# ROS Data Acquisition Driver for 3DM-GX3-15 IMU module (Single Byte Protocol)

This program is from Nathan Michael's original ROS driver for the Lord Corporation Microstrain 3DM GX3 15 IMU 

Adding and compiling:

1. cd ~/catkin_ws/src

2. git clone --branch ros1 https://github.com/jimin3722/3DM_GX3_15_ros.git

3. rosdep install 3dm_gx3_15_ros

4. cd ..

5. catkin_make 


Usage:

rosrun imu_3dm_gx3 imu_3dm_gx3 port:=/dev/ttyACM0 decimation:=2

imu_data_rate = 1000/decimation HZ


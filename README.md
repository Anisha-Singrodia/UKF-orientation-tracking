# UKF-orientation-tracking
Implementation of an Unscented Kalman Filter (UKF) to track the orientation of a robot in three-dimensions.

Given observations from an inertial measurement unit (IMU) that consists of a gyroscope (to measure angular velocity) 
and an accelerometer (to measure accelerations in body frame) as well as data from a motion-capture system called “Vicon”,

We treat the Vicon data as the “ground-truth”. We developed the UKF for the IMU data and used the Vicon data for 
calibration and tuning of this filter.

The IMU datasets are present in the imu folder. 
The estimate_rot.py tracks the orientation using the IMU data and UKF.

Results:

![vicon_pitch_vs_pred](https://user-images.githubusercontent.com/114776023/223570818-b444b103-8ab7-471f-83a6-bc00c0d5b3d4.png)
![vicon_roll_vs_pred](https://user-images.githubusercontent.com/114776023/223570821-cccd8954-131f-4ba1-b16e-fda97647b3c1.png)
![vicon_yaw_vs_pred](https://user-images.githubusercontent.com/114776023/223570823-4c0f97f7-c38b-4afe-8f9c-d6abe7a0a1a0.png)

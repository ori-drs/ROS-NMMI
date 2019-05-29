# State message valid for IMU

int32 id
time stamp

#  - imu_id
#  - sensors: accelerations [g], gyroscopes [deg/s], magnetometers [uT], quaternion and temperature [°C]
nmmi_msgs/ResourceData[] s

# Reliability of the retrieved measurements
bool is_reliable
int32 consecutive_failures
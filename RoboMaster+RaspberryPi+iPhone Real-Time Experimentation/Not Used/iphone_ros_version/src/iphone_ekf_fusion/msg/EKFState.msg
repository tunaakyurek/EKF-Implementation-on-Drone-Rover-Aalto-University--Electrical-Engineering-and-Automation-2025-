# 15-DOF EKF State Message
# State vector following RoboMaster EKF Formulary 15-DOF model

Header header

# Position (NED frame) - 3 DOF
geometry_msgs/Point position              # [x, y, z] in meters

# Velocity (NED frame) - 3 DOF  
geometry_msgs/Vector3 velocity            # [vx, vy, vz] in m/s

# Orientation (NED frame) - 3 DOF
geometry_msgs/Quaternion orientation      # Quaternion representation
geometry_msgs/Vector3 euler_angles       # [roll, pitch, yaw] in radians

# Angular velocity (body frame) - 3 DOF
geometry_msgs/Vector3 angular_velocity    # [wx, wy, wz] in rad/s

# IMU Biases - 3 DOF (making it 15-DOF total)
geometry_msgs/Vector3 accelerometer_bias  # [bax, bay, baz] in m/s^2
geometry_msgs/Vector3 gyroscope_bias     # [bgx, bgy, bgz] in rad/s

# Covariance (15x15 matrix, row-major)
float64[225] covariance

# Additional state info
float64 covariance_trace
uint32 update_count
float64 timestamp

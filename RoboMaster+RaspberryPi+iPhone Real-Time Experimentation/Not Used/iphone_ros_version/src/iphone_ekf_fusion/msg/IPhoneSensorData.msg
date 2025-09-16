# iPhone Sensor Data Message
# All sensor measurements from iPhone
# Following RoboMaster EKF Formulary specifications

Header header

# IMU Data (body frame)
geometry_msgs/Vector3 linear_acceleration  # m/s^2
geometry_msgs/Vector3 angular_velocity     # rad/s

# Magnetometer (body frame)
geometry_msgs/Vector3 magnetic_field       # Tesla

# GPS Data (if available)
float64 latitude                          # degrees
float64 longitude                         # degrees
float64 altitude                          # meters
float64 gps_accuracy                      # meters
bool gps_valid                           # GPS fix status

# Barometer
float64 pressure                          # Pascals
float64 barometric_altitude               # meters

# Device Motion (from Core Motion)
geometry_msgs/Quaternion orientation      # Device orientation quaternion
geometry_msgs/Vector3 gravity            # Gravity vector in device frame
geometry_msgs/Vector3 user_acceleration  # Linear acceleration without gravity

# Quality indicators
float64 magnetic_field_accuracy          # 0-1, 1 being best
uint8 motion_quality                     # 0-100 quality score

"""
Minimal Sensor Interface for RoboMaster S1 Direct Execution

This module provides a simplified sensor interface that works within
the constraints of the RoboMaster S1's internal environment.

LIMITATIONS:
- No advanced sensor fusion
- Reduced sampling rates (5-20 Hz max)
- Limited error handling due to memory constraints
- No external library dependencies
"""

import time
import math

class S1SensorInterface:
    """
    Minimal sensor interface for RoboMaster S1 internal execution
    
    This class attempts to access sensor data directly from the S1's
    internal systems, with fallbacks for different access levels.
    """
    
    def __init__(self, access_level="app"):
        """
        Initialize sensor interface
        
        Args:
            access_level: "app", "adb", or "root"
        """
        self.access_level = access_level
        self.is_initialized = False
        self.robot = None
        
        # Sensor data storage
        self.latest_imu = {}
        self.latest_chassis = {}
        
        # Timing
        self.last_sensor_time = 0
        self.sensor_frequency = 10  # Hz
        
        # Error tracking
        self.sensor_errors = 0
        self.connection_lost = False
        
        print(f"S1SensorInterface initialized with access level: {access_level}")
    
    def initialize(self):
        """Initialize sensor connections based on access level"""
        try:
            if self.access_level == "app":
                success = self._init_app_level()
            elif self.access_level == "adb":
                success = self._init_adb_level()
            elif self.access_level == "root":
                success = self._init_root_level()
            else:
                print(f"Unknown access level: {self.access_level}")
                return False
            
            if success:
                self.is_initialized = True
                print("Sensor interface initialized successfully")
            else:
                print("Failed to initialize sensor interface")
            
            return success
            
        except Exception as e:
            print(f"Error initializing sensors: {e}")
            return False
    
    def _init_app_level(self):
        """Initialize using RoboMaster app SDK (safest method)"""
        try:
            # Try to import and use the official SDK
            import robot
            
            self.robot = robot.Robot()
            self.robot.initialize(conn_type="ap")
            
            # Subscribe to sensor data
            self.robot.sensor.sub_gyroscope(freq=self.sensor_frequency)
            self.robot.sensor.sub_accelerometer(freq=self.sensor_frequency)
            self.robot.chassis.sub_position(freq=self.sensor_frequency)
            
            print("App-level sensor access initialized")
            return True
            
        except ImportError:
            print("RoboMaster SDK not available")
            return False
        except Exception as e:
            print(f"App-level initialization failed: {e}")
            return False
    
    def _init_adb_level(self):
        """Initialize using ADB access (requires sandbox escape)"""
        try:
            # Attempt to access internal sensor interfaces
            # This would require successful ADB access
            
            print("ADB-level sensor access not fully implemented")
            print("Would require direct access to S1's internal sensor drivers")
            
            # For now, fall back to simulation
            return self._init_simulation()
            
        except Exception as e:
            print(f"ADB-level initialization failed: {e}")
            return False
    
    def _init_root_level(self):
        """Initialize using root access (highest risk)"""
        try:
            # Direct hardware access would go here
            # Requires successful rooting of the S1
            
            print("Root-level sensor access not implemented")
            print("Would require direct hardware driver access")
            
            # For now, fall back to simulation
            return self._init_simulation()
            
        except Exception as e:
            print(f"Root-level initialization failed: {e}")
            return False
    
    def _init_simulation(self):
        """Initialize with simulated sensor data (fallback)"""
        print("Using simulated sensor data")
        self.simulation_start_time = time.time()
        return True
    
    def read_sensors(self):
        """Read current sensor data"""
        if not self.is_initialized:
            print("Sensor interface not initialized")
            return None
        
        try:
            current_time = time.time()
            
            # Check if enough time has passed since last reading
            if current_time - self.last_sensor_time < 1.0 / self.sensor_frequency:
                return None
            
            if self.access_level == "app" and self.robot:
                data = self._read_app_sensors()
            else:
                data = self._read_simulated_sensors()
            
            self.last_sensor_time = current_time
            
            if data:
                data['timestamp'] = current_time
                data['frequency'] = self.sensor_frequency
            
            return data
            
        except Exception as e:
            self.sensor_errors += 1
            print(f"Error reading sensors: {e}")
            return None
    
    def _read_app_sensors(self):
        """Read sensors using app-level SDK"""
        try:
            # Get IMU data
            gyro = self.robot.sensor.get_gyroscope()
            accel = self.robot.sensor.get_accelerometer()
            
            # Get chassis data
            position = self.robot.chassis.get_position()
            
            return {
                'imu': {
                    'gyro_x': math.radians(gyro[0]) if gyro else 0.0,
                    'gyro_y': math.radians(gyro[1]) if gyro else 0.0,
                    'gyro_z': math.radians(gyro[2]) if gyro else 0.0,
                    'accel_x': accel[0] if accel else 0.0,
                    'accel_y': accel[1] if accel else 0.0,
                    'accel_z': accel[2] if accel else 9.81
                },
                'chassis': {
                    'x': position[0] / 1000.0 if position else 0.0,  # mm to m
                    'y': position[1] / 1000.0 if position else 0.0,  # mm to m
                    'yaw': math.radians(position[3]) if position else 0.0  # deg to rad
                }
            }
            
        except Exception as e:
            print(f"Error reading app sensors: {e}")
            return None
    
    def _read_simulated_sensors(self):
        """Generate simulated sensor data for testing"""
        if not hasattr(self, 'simulation_start_time'):
            self.simulation_start_time = time.time()
        
        t = time.time() - self.simulation_start_time
        
        # Simulate robot moving in a figure-8 pattern
        scale = 0.5
        omega = 0.2  # rad/s
        
        # Position
        x = scale * math.sin(omega * t)
        y = scale * math.sin(2 * omega * t)
        yaw = math.atan2(
            2 * omega * scale * math.cos(2 * omega * t),
            omega * scale * math.cos(omega * t)
        )
        
        # Add some noise
        noise_scale = 0.05
        x += noise_scale * math.sin(t * 17.3)
        y += noise_scale * math.cos(t * 23.1)
        yaw += noise_scale * math.sin(t * 31.7)
        
        # Simulate accelerometer (gravity + motion)
        accel_x = -9.81 * math.sin(yaw) + 0.1 * math.sin(t * 10)
        accel_y = 9.81 * math.cos(yaw) + 0.1 * math.cos(t * 15)
        accel_z = 9.81 + 0.05 * math.sin(t * 20)
        
        # Simulate gyroscope
        gyro_x = 0.02 * math.sin(t * 5)
        gyro_y = 0.02 * math.cos(t * 7)
        gyro_z = omega + 0.05 * math.sin(t * 13)
        
        return {
            'imu': {
                'gyro_x': gyro_x,
                'gyro_y': gyro_y,
                'gyro_z': gyro_z,
                'accel_x': accel_x,
                'accel_y': accel_y,
                'accel_z': accel_z
            },
            'chassis': {
                'x': x,
                'y': y,
                'yaw': yaw
            }
        }
    
    def get_sensor_stats(self):
        """Get sensor performance statistics"""
        return {
            'access_level': self.access_level,
            'is_initialized': self.is_initialized,
            'sensor_frequency': self.sensor_frequency,
            'sensor_errors': self.sensor_errors,
            'connection_lost': self.connection_lost,
            'last_sensor_time': self.last_sensor_time
        }
    
    def set_frequency(self, frequency):
        """Set sensor reading frequency (Hz)"""
        if frequency <= 0 or frequency > 50:
            print(f"Invalid frequency: {frequency}. Must be between 1-50 Hz")
            return False
        
        self.sensor_frequency = frequency
        print(f"Sensor frequency set to {frequency} Hz")
        
        # If using app-level access, update subscriptions
        if self.access_level == "app" and self.robot:
            try:
                self.robot.sensor.unsub_gyroscope()
                self.robot.sensor.unsub_accelerometer()
                self.robot.chassis.unsub_position()
                
                self.robot.sensor.sub_gyroscope(freq=frequency)
                self.robot.sensor.sub_accelerometer(freq=frequency)
                self.robot.chassis.sub_position(freq=frequency)
                
                print("App-level sensor subscriptions updated")
            except Exception as e:
                print(f"Error updating sensor frequency: {e}")
                return False
        
        return True
    
    def close(self):
        """Clean up sensor connections"""
        try:
            if self.access_level == "app" and self.robot:
                self.robot.sensor.unsub_gyroscope()
                self.robot.sensor.unsub_accelerometer()
                self.robot.chassis.unsub_position()
                self.robot.close()
                print("App-level sensor connections closed")
            
            self.is_initialized = False
            print("Sensor interface closed")
            
        except Exception as e:
            print(f"Error closing sensor interface: {e}")


class SimpleComplementaryFilter:
    """
    Simple complementary filter for attitude estimation
    
    This is even simpler than the minimal EKF and might be more
    suitable for the S1's limited computational resources.
    """
    
    def __init__(self, alpha=0.98):
        """
        Initialize complementary filter
        
        Args:
            alpha: Filter coefficient (0-1, higher = trust gyro more)
        """
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = None
    
    def update(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt):
        """Update attitude estimate"""
        if dt <= 0:
            return
        
        # Integrate gyroscope (high-frequency estimate)
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt
        
        # Calculate attitude from accelerometer (low-frequency estimate)
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        
        if accel_magnitude > 0.1:  # Avoid division by zero
            accel_roll = math.atan2(accel_y, accel_z)
            accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
            
            # Complementary filter
            self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch
        
        # Normalize angles
        self.roll = self._normalize_angle(self.roll)
        self.pitch = self._normalize_angle(self.pitch)
        self.yaw = self._normalize_angle(self.yaw)
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_attitude(self):
        """Get current attitude estimate"""
        return {
            'roll': self.roll,
            'pitch': self.pitch,
            'yaw': self.yaw
        }


def test_sensor_interface():
    """Test the sensor interface"""
    print("=== Testing S1 Sensor Interface ===")
    
    # Try different access levels
    for access_level in ["app", "adb", "root"]:
        print(f"\n--- Testing {access_level} access level ---")
        
        sensor_interface = S1SensorInterface(access_level)
        
        if sensor_interface.initialize():
            print("Initialization successful, reading sensors...")
            
            # Read sensors for a few iterations
            for i in range(10):
                data = sensor_interface.read_sensors()
                
                if data:
                    imu = data['imu']
                    chassis = data['chassis']
                    
                    print(f"Reading {i+1}:")
                    print(f"  IMU: accel=({imu['accel_x']:.3f}, {imu['accel_y']:.3f}, {imu['accel_z']:.3f})")
                    print(f"       gyro=({imu['gyro_x']:.3f}, {imu['gyro_y']:.3f}, {imu['gyro_z']:.3f})")
                    print(f"  Chassis: pos=({chassis['x']:.3f}, {chassis['y']:.3f}), yaw={chassis['yaw']:.3f}")
                
                time.sleep(0.1)
            
            # Show statistics
            stats = sensor_interface.get_sensor_stats()
            print(f"Stats: errors={stats['sensor_errors']}, freq={stats['sensor_frequency']} Hz")
            
            sensor_interface.close()
        else:
            print("Initialization failed")


def test_complementary_filter():
    """Test the complementary filter with simulated data"""
    print("\n=== Testing Complementary Filter ===")
    
    filter = SimpleComplementaryFilter(alpha=0.98)
    start_time = time.time()
    
    for i in range(50):
        current_time = time.time()
        dt = 0.02  # 50 Hz
        
        # Simulate some rotation
        t = current_time - start_time
        true_roll = 0.2 * math.sin(t)
        true_pitch = 0.1 * math.cos(t * 1.5)
        true_yaw = 0.5 * t
        
        # Simulate noisy sensor data
        noise = 0.1
        accel_x = -9.81 * math.sin(true_pitch) + noise * math.sin(t * 20)
        accel_y = 9.81 * math.sin(true_roll) * math.cos(true_pitch) + noise * math.cos(t * 25)
        accel_z = 9.81 * math.cos(true_roll) * math.cos(true_pitch) + noise * math.sin(t * 30)
        
        gyro_x = 0.2 * math.cos(t) + 0.05 * math.sin(t * 15)
        gyro_y = -0.15 * math.sin(t * 1.5) + 0.05 * math.cos(t * 18)
        gyro_z = 0.5 + 0.1 * math.sin(t * 12)
        
        # Update filter
        filter.update(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt)
        
        # Print results every 10 iterations
        if i % 10 == 0:
            attitude = filter.get_attitude()
            print(f"t={t:.1f}s: roll={attitude['roll']:.3f}, pitch={attitude['pitch']:.3f}, yaw={attitude['yaw']:.3f}")
        
        time.sleep(dt)


if __name__ == "__main__":
    test_sensor_interface()
    test_complementary_filter()
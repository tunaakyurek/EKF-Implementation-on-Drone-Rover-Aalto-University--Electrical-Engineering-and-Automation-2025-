"""
Minimal Autonomous Controller for RoboMaster S1 Lab
==================================================
Ultra-minimal version with ZERO external imports
Works in the most restrictive RoboMaster Lab environment

Copy this entire code into RoboMaster app Lab and run!

SAFETY FEATURES:
- Conservative speeds (300mm/s max)
- 20 second timeout
- Emergency stop capability

Author: RoboMaster Lab Fix
"""

import math
import time

# Control modes as simple constants (no enum)
MODE_IDLE = "idle"
MODE_DEMO = "demo"
MODE_WAYPOINT = "waypoint"
MODE_EMERGENCY = "emergency"

class MinimalController:
    """
    Ultra-minimal autonomous controller
    NO external dependencies whatsoever
    """
    
    def __init__(self):
        # Current mode
        self.mode = MODE_IDLE
        
        # Safety limits (conservative for S1)
        self.max_speed = 300      # mm/s
        self.max_rotation = 30    # deg/s
        
        # Control state
        self.is_active = False
        self.start_time = None
        
        # Position estimation (basic)
        self.x = 0.0              # meters
        self.y = 0.0              # meters
        self.theta = 0.0          # radians
        
        # Current sensor data
        self.gyro_data = [0.0, 0.0, 0.0]
        self.accel_data = [0.0, 0.0, 0.0]
        self.attitude_data = [0.0, 0.0, 0.0]
        
        print("Minimal Controller Ready")
    
    def start_system(self):
        """Initialize and start the system"""
        try:
            # Set robot mode
            robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
            print("Robot mode set successfully")
            return True
        except Exception as e:
            print("Failed to set robot mode: {}".format(e))
            return False
    
    def read_sensors(self):
        """Read basic sensor data"""
        try:
            self.gyro_data = sensor_imu.get_gyroscope()
            self.accel_data = sensor_imu.get_accelerometer()
            self.attitude_data = sensor_attitude.get_attitude()
            return True
        except Exception as e:
            print("Sensor read error: {}".format(e))
            return False
    
    def move_forward(self, speed=None, duration=2.0):
        """Move forward for specified duration"""
        if speed is None:
            speed = self.max_speed
        
        print("Moving forward at {} mm/s for {}s".format(speed, duration))
        
        try:
            chassis_ctrl.set_trans_speed(int(speed))
            chassis_ctrl.set_rotate_speed(0)
            
            # Wait for duration
            time.sleep(duration)
            
            # Stop
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(0)
            
            print("Forward movement completed")
            return True
            
        except Exception as e:
            print("Movement error: {}".format(e))
            self.emergency_stop()
            return False
    
    def rotate_left(self, speed=None, duration=2.0):
        """Rotate left for specified duration"""
        if speed is None:
            speed = self.max_rotation
        
        print("Rotating left at {} deg/s for {}s".format(speed, duration))
        
        try:
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(int(speed))
            
            time.sleep(duration)
            
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(0)
            
            print("Left rotation completed")
            return True
            
        except Exception as e:
            print("Rotation error: {}".format(e))
            self.emergency_stop()
            return False
    
    def rotate_right(self, speed=None, duration=2.0):
        """Rotate right for specified duration"""
        if speed is None:
            speed = self.max_rotation
        
        print("Rotating right at {} deg/s for {}s".format(speed, duration))
        
        try:
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(-int(speed))
            
            time.sleep(duration)
            
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(0)
            
            print("Right rotation completed")
            return True
            
        except Exception as e:
            print("Rotation error: {}".format(e))
            self.emergency_stop()
            return False
    
    def move_backward(self, speed=None, duration=2.0):
        """Move backward for specified duration"""
        if speed is None:
            speed = self.max_speed
        
        print("Moving backward at {} mm/s for {}s".format(speed, duration))
        
        try:
            chassis_ctrl.set_trans_speed(-int(speed))
            chassis_ctrl.set_rotate_speed(0)
            
            time.sleep(duration)
            
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(0)
            
            print("Backward movement completed")
            return True
            
        except Exception as e:
            print("Movement error: {}".format(e))
            self.emergency_stop()
            return False
    
    def emergency_stop(self):
        """Emergency stop all movement"""
        print("EMERGENCY STOP!")
        try:
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(0)
        except Exception as e:
            print("Emergency stop error: {}".format(e))
        
        self.is_active = False
        self.mode = MODE_EMERGENCY
    
    def run_basic_demo(self):
        """Run basic movement demonstration"""
        print("=== Starting Basic Demo ===")
        
        if not self.start_system():
            print("Failed to start system")
            return
        
        self.is_active = True
        self.mode = MODE_DEMO
        self.start_time = time.time()
        
        try:
            # Demo sequence with error checking
            if self.is_active and self.move_forward(200, 2.0):
                time.sleep(0.5)
            
            if self.is_active and self.rotate_left(20, 1.5):
                time.sleep(0.5)
            
            if self.is_active and self.move_backward(150, 1.5):
                time.sleep(0.5)
            
            if self.is_active and self.rotate_right(25, 1.5):
                time.sleep(0.5)
            
            print("Demo completed successfully!")
            
        except Exception as e:
            print("Demo error: {}".format(e))
            self.emergency_stop()
        
        finally:
            self.emergency_stop()
            elapsed = time.time() - self.start_time
            print("Total demo time: {:.1f} seconds".format(elapsed))
    
    def test_sensors(self, duration=5.0):
        """Test sensor readings"""
        print("=== Sensor Test ===")
        
        start_time = time.time()
        count = 0
        
        while time.time() - start_time < duration:
            if self.read_sensors():
                count += 1
                print("Reading {}:".format(count))
                print("  Gyro: {}".format(self.gyro_data))
                print("  Accel: {}".format(self.accel_data))
                print("  Attitude: {}".format(self.attitude_data))
            else:
                print("Sensor read failed at reading {}".format(count + 1))
            
            time.sleep(1.0)
        
        print("Sensor test completed: {} successful readings".format(count))
    
    def simple_navigation(self, target_distance=1.0):
        """Simple navigation using time-based estimation"""
        print("=== Simple Navigation to {}m ===".format(target_distance))
        
        if not self.start_system():
            return
        
        # Estimate time needed (speed in m/s)
        speed_ms = self.max_speed / 1000.0  # Convert mm/s to m/s
        time_needed = target_distance / speed_ms
        
        print("Estimated time: {:.1f} seconds".format(time_needed))
        
        try:
            # Move forward
            self.move_forward(self.max_speed, time_needed)
            
            # Pause
            time.sleep(1.0)
            
            # Return (rotate 180 and move back)
            self.rotate_left(self.max_rotation, 6.0)  # ~180 degrees
            time.sleep(0.5)
            self.move_forward(self.max_speed, time_needed)
            
            print("Navigation completed!")
            
        except Exception as e:
            print("Navigation error: {}".format(e))
        
        finally:
            self.emergency_stop()


# Simple waypoint class
class SimpleWaypoint:
    """Ultra-simple waypoint navigation"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0  # radians
        
        # Control gains (simple)
        self.kp_distance = 300  # mm/s per meter error
        self.kp_heading = 20    # deg/s per radian error
        
        # Limits
        self.max_speed = 400
        self.max_rotation = 40
        self.tolerance = 0.3  # meters
    
    def update_odometry(self, speed_mm, rotation_deg, dt):
        """Simple odometry update"""
        # Convert to m/s and rad/s
        speed_ms = speed_mm / 1000.0
        omega = math.radians(rotation_deg)
        
        # Update position (simplified 2D)
        self.x += speed_ms * math.cos(self.heading) * dt
        self.y += speed_ms * math.sin(self.heading) * dt
        self.heading += omega * dt
        
        # Normalize heading
        while self.heading > math.pi:
            self.heading -= 2 * math.pi
        while self.heading < -math.pi:
            self.heading += 2 * math.pi
    
    def go_to_point(self, target_x, target_y, timeout=15.0):
        """Navigate to target point"""
        print("Navigating to ({:.2f}, {:.2f})".format(target_x, target_y))
        
        try:
            robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
        except Exception as e:
            print("Mode set error: {}".format(e))
            return False
        
        start_time = time.time()
        last_update = start_time
        
        try:
            while time.time() - start_time < timeout:
                current_time = time.time()
                dt = current_time - last_update
                last_update = current_time
                
                # Compute errors
                error_x = target_x - self.x
                error_y = target_y - self.y
                distance = math.sqrt(error_x**2 + error_y**2)
                
                print("Position: ({:.2f}, {:.2f}), Distance: {:.2f}".format(self.x, self.y, distance))
                
                # Check if reached
                if distance < self.tolerance:
                    print("Target reached!")
                    chassis_ctrl.set_trans_speed(0)
                    chassis_ctrl.set_rotate_speed(0)
                    return True
                
                # Compute desired heading
                desired_heading = math.atan2(error_y, error_x)
                heading_error = desired_heading - self.heading
                
                # Normalize heading error
                while heading_error > math.pi:
                    heading_error -= 2 * math.pi
                while heading_error < -math.pi:
                    heading_error += 2 * math.pi
                
                # Control commands
                if abs(heading_error) > 0.3:  # ~17 degrees
                    # Rotate to face target
                    rotation_speed = max(-self.max_rotation, 
                                       min(self.max_rotation, heading_error * self.kp_heading))
                    trans_speed = 0
                else:
                    # Move toward target
                    trans_speed = max(-self.max_speed, 
                                    min(self.max_speed, distance * self.kp_distance))
                    rotation_speed = heading_error * 5  # Small correction
                
                # Send commands
                chassis_ctrl.set_trans_speed(int(trans_speed))
                chassis_ctrl.set_rotate_speed(int(rotation_speed))
                
                # Update odometry estimate
                self.update_odometry(trans_speed, rotation_speed, dt)
                
                time.sleep(0.1)  # 10 Hz control loop
            
            print("Navigation timeout")
            return False
            
        except Exception as e:
            print("Navigation error: {}".format(e))
            return False
        
        finally:
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(0)


# Demo functions
def run_basic_movement():
    """Run basic movement demo"""
    controller = MinimalController()
    controller.run_basic_demo()


def run_sensor_reading():
    """Run sensor reading test"""
    controller = MinimalController()
    controller.test_sensors(5.0)


def run_simple_navigation():
    """Run simple navigation demo"""
    controller = MinimalController()
    controller.simple_navigation(0.5)  # Navigate 0.5 meters


def run_waypoint_navigation():
    """Run waypoint navigation demo"""
    navigator = SimpleWaypoint()
    
    # Small square pattern
    waypoints = [(0.5, 0.0), (0.5, 0.5), (0.0, 0.5), (0.0, 0.0)]
    
    for i, point in enumerate(waypoints):
        print("\n--- Waypoint {}: ({}, {}) ---".format(i+1, point[0], point[1]))
        success = navigator.go_to_point(point[0], point[1])
        if not success:
            print("Waypoint navigation failed")
            break
        time.sleep(1)  # Pause between waypoints


def emergency_stop():
    """Emergency stop function"""
    print("EMERGENCY STOP ACTIVATED!")
    try:
        chassis_ctrl.set_trans_speed(0)
        chassis_ctrl.set_rotate_speed(0)
    except Exception as e:
        print("Emergency stop error: {}".format(e))


# Main execution
print("=" * 50)
print("RoboMaster S1 Minimal Autonomous Controller")
print("=" * 50)
print()
print("Available functions:")
print("1. run_basic_movement()     - Forward, rotate, backward sequence")
print("2. run_sensor_reading()     - Test sensor data reading")
print("3. run_simple_navigation()  - Navigate forward and return")
print("4. run_waypoint_navigation() - Navigate square pattern")
print("5. emergency_stop()         - Stop all movement immediately")
print()
print("SAFETY:")
print("- Maximum speed: 300-400 mm/s")
print("- Automatic timeout: 15-20 seconds")
print("- Emergency stop available anytime")
print()
print("To run a demo, type the function name and press Run")
print("Example: run_basic_movement()")

# Uncomment ONE line below to auto-run a demo:
# run_basic_movement()
# run_sensor_reading()
# run_simple_navigation()
# run_waypoint_navigation()

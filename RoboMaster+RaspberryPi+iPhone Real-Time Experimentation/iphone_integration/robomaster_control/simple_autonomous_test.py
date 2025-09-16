"""
Simple Autonomous Test for RoboMaster S1 Lab Environment
========================================================
Minimal test version that can be copy-pasted directly into RoboMaster app Lab

INSTRUCTIONS:
1. Open RoboMaster app
2. Connect to S1
3. Go to Lab > Python
4. Copy this entire code
5. Paste and run

SAFETY: This code includes emergency stops and conservative speeds
"""

import math
import time

class SimpleAutonomous:
    """Minimal autonomous controller for RoboMaster S1 Lab"""
    
    def __init__(self):
        self.is_active = False
        self.start_time = None
        self.mode = "idle"
        
        # Safety limits
        self.max_speed = 300    # mm/s (conservative)
        self.max_rotation = 30  # deg/s (conservative)
        
        print("Simple Autonomous Controller Ready")
    
    def start_demo(self):
        """Start autonomous demo"""
        print("Starting autonomous demo...")
        
        try:
            # Set robot mode
            robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
            
            self.is_active = True
            self.start_time = time.time()
            
            # Demo sequence: forward, rotate, backward, rotate
            self.run_demo_sequence()
            
        except Exception as e:
            print("Demo error: {}".format(e))
        finally:
            self.stop_all()
    
    def run_demo_sequence(self):
        """Run simple movement sequence"""
        sequences = [
            ("forward", 3.0),    # Move forward 3 seconds
            ("stop", 1.0),       # Stop 1 second
            ("rotate_left", 2.0), # Rotate left 2 seconds
            ("stop", 1.0),       # Stop 1 second
            ("backward", 2.0),   # Move backward 2 seconds
            ("stop", 1.0),       # Stop 1 second
            ("rotate_right", 2.0), # Rotate right 2 seconds
            ("stop", 2.0)        # Final stop
        ]
        
        for action, duration in sequences:
            if not self.is_active:
                break
                
            print("Action: {} for {}s".format(action, duration))
            self.execute_action(action)
            
            # Wait for duration
            action_start = time.time()
            while time.time() - action_start < duration and self.is_active:
                time.sleep(0.1)
                
                # Safety timeout (30 seconds total)
                if time.time() - self.start_time > 30:
                    print("Safety timeout reached")
                    self.is_active = False
                    break
        
        print("Demo sequence completed")
    
    def execute_action(self, action):
        """Execute specific action"""
        try:
            if action == "forward":
                chassis_ctrl.set_trans_speed(self.max_speed)
                chassis_ctrl.set_rotate_speed(0)
            
            elif action == "backward":
                chassis_ctrl.set_trans_speed(-self.max_speed)
                chassis_ctrl.set_rotate_speed(0)
            
            elif action == "rotate_left":
                chassis_ctrl.set_trans_speed(0)
                chassis_ctrl.set_rotate_speed(self.max_rotation)
            
            elif action == "rotate_right":
                chassis_ctrl.set_trans_speed(0)
                chassis_ctrl.set_rotate_speed(-self.max_rotation)
            
            elif action == "stop":
                chassis_ctrl.set_trans_speed(0)
                chassis_ctrl.set_rotate_speed(0)
            
        except Exception as e:
            print("Action execution error: {}".format(e))
    
    def stop_all(self):
        """Emergency stop all movement"""
        self.is_active = False
        try:
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(0)
            print("All movement stopped")
        except Exception as e:
            print("Stop error: {}".format(e))
    
    def get_sensor_data(self):
        """Get basic sensor readings"""
        try:
            gyro = sensor_imu.get_gyroscope()
            accel = sensor_imu.get_accelerometer()
            attitude = sensor_attitude.get_attitude()
            
            print("Gyro: {}".format(gyro))
            print("Accel: {}".format(accel))
            print("Attitude: {}".format(attitude))
            
        except Exception as e:
            print("Sensor error: {}".format(e))


# Waypoint navigation class
class SimpleWaypoint:
    """Simple waypoint navigation using odometry estimation"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = time.time()
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.tolerance = 0.3  # meters
        
        # Simple PID gains
        self.kp = 400  # Proportional gain for speed control
        self.max_speed = 400  # mm/s
        
    def update_position(self, vx_mm, rotation_deg, dt):
        """Simple odometry update"""
        # Convert to m/s and rad/s
        vx = vx_mm / 1000.0
        omega = math.radians(rotation_deg)
        
        # Update position (simplified)
        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt
        self.yaw += omega * dt
        
        # Normalize yaw
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi
    
    def go_to_point(self, target_x, target_y):
        """Navigate to target point"""
        self.target_x = target_x
        self.target_y = target_y
        
        print("Navigating to ({:.2f}, {:.2f})".format(target_x, target_y))
        
        start_time = time.time()
        last_command = time.time()
        
        try:
            robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
            
            while time.time() - start_time < 20:  # 20 second timeout
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time
                
                # Compute errors
                error_x = self.target_x - self.x
                error_y = self.target_y - self.y
                distance = math.sqrt(error_x**2 + error_y**2)
                
                print("Position: ({:.2f}, {:.2f}), Distance: {:.2f}".format(self.x, self.y, distance))
                
                # Check if reached
                if distance < self.tolerance:
                    print("Target reached!")
                    break
                
                # Compute desired heading
                desired_yaw = math.atan2(error_y, error_x)
                yaw_error = desired_yaw - self.yaw
                
                # Normalize yaw error
                while yaw_error > math.pi:
                    yaw_error -= 2 * math.pi
                while yaw_error < -math.pi:
                    yaw_error += 2 * math.pi
                
                # Control commands
                if abs(yaw_error) > 0.2:  # 0.2 rad ≈ 11 degrees
                    # Rotate to face target
                    rotation_speed = max(-30, min(30, yaw_error * 20))  # deg/s
                    trans_speed = 0
                else:
                    # Move toward target
                    trans_speed = max(-self.max_speed, min(self.max_speed, distance * self.kp))
                    rotation_speed = yaw_error * 10  # Small correction
                
                # Send commands (limit frequency)
                if current_time - last_command > 0.1:  # 10 Hz
                    chassis_ctrl.set_trans_speed(int(trans_speed))
                    chassis_ctrl.set_rotate_speed(int(rotation_speed))
                    last_command = current_time
                    
                    # Update position estimate
                    self.update_position(trans_speed, rotation_speed, dt)
                
                time.sleep(0.05)  # 20 Hz loop
            
        except Exception as e:
            print("Navigation error: {}".format(e))
        
        finally:
            # Stop movement
            chassis_ctrl.set_trans_speed(0)
            chassis_ctrl.set_rotate_speed(0)


# Main demo functions
def run_basic_movement_demo():
    """Run basic movement demo"""
    print("=== Basic Movement Demo ===")
    controller = SimpleAutonomous()
    controller.start_demo()


def run_sensor_test():
    """Run sensor reading test"""
    print("=== Sensor Test ===")
    controller = SimpleAutonomous()
    
    for i in range(10):
        print("\n--- Reading {} ---".format(i+1))
        controller.get_sensor_data()
        time.sleep(1)


def run_waypoint_demo():
    """Run waypoint navigation demo"""
    print("=== Waypoint Navigation Demo ===")
    navigator = SimpleWaypoint()
    
    # Navigate to several points
    waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    
    for point in waypoints:
        navigator.go_to_point(point[0], point[1])
        time.sleep(2)  # Pause between waypoints


def emergency_stop():
    """Emergency stop function"""
    print("EMERGENCY STOP!")
    try:
        chassis_ctrl.set_trans_speed(0)
        chassis_ctrl.set_rotate_speed(0)
    except:
        pass


# ==== MAIN EXECUTION ====
# Uncomment ONE of these lines to run a demo:

print("RoboMaster S1 Simple Autonomous Test Ready")
print("Available demos:")
print("1. run_basic_movement_demo()  - Basic forward/rotate sequence")
print("2. run_sensor_test()          - Read sensor data")
print("3. run_waypoint_demo()        - Simple waypoint navigation")
print("4. emergency_stop()           - Stop all movement")
print("\nTo run a demo, type the function name and press Run")

# Example: Uncomment to run basic demo automatically
# run_basic_movement_demo()

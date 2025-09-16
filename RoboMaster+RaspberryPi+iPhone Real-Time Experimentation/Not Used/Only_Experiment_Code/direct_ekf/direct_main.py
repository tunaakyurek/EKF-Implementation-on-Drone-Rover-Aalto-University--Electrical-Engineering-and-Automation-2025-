"""
Direct EKF Execution on RoboMaster S1

This is the main execution script for running a simplified EKF directly
on the RoboMaster S1's internal computing unit.

WARNINGS:
- This approach has severe limitations (see README.md)
- Performance will be poor compared to Raspberry Pi approach
- Not recommended for serious applications
- Educational/research purposes only

USAGE:
- Upload this file to S1 via app Lab or ADB
- Execute: /data/python_files/bin/python direct_main.py
"""

import sys
import time
import math

# Try to add the current directory to path for imports
try:
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.append(current_dir)
except:
    pass

# Import our minimal implementations
try:
    from minimal_ekf import MinimalEKF, SensorReading
    from sensor_minimal import S1SensorInterface, SimpleComplementaryFilter
    print("Successfully imported minimal EKF modules")
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure all files are in the same directory")
    sys.exit(1)


class S1DirectEKFSystem:
    """
    Complete EKF system for direct execution on RoboMaster S1
    
    This integrates the minimal EKF with the sensor interface to provide
    a complete (though limited) state estimation system.
    """
    
    def __init__(self, access_level="app", use_ekf=True):
        """
        Initialize the S1 EKF system
        
        Args:
            access_level: "app", "adb", or "root"
            use_ekf: If False, use complementary filter instead
        """
        self.access_level = access_level
        self.use_ekf = use_ekf
        
        # Initialize components
        self.sensor_interface = S1SensorInterface(access_level)
        
        if use_ekf:
            self.estimator = MinimalEKF()
            self.estimator_type = "EKF"
        else:
            self.estimator = SimpleComplementaryFilter()
            self.estimator_type = "Complementary Filter"
        
        # System state
        self.is_running = False
        self.start_time = None
        self.iteration_count = 0
        
        # Performance monitoring
        self.loop_times = []
        self.max_loop_time = 0.0
        self.memory_usage = []
        
        # Data storage (limited)
        self.state_history = []
        self.max_history_size = 1000  # Limit due to memory constraints
        
        print(f"S1DirectEKFSystem initialized:")
        print(f"  Access level: {access_level}")
        print(f"  Estimator: {self.estimator_type}")
    
    def initialize(self):
        """Initialize the system"""
        print("Initializing S1 Direct EKF System...")
        
        # Initialize sensor interface
        if not self.sensor_interface.initialize():
            print("Failed to initialize sensor interface")
            return False
        
        # Set reasonable sensor frequency for S1's capabilities
        target_freq = 10  # Hz - conservative for S1 hardware
        if not self.sensor_interface.set_frequency(target_freq):
            print(f"Warning: Could not set sensor frequency to {target_freq} Hz")
        
        # Initialize estimator (EKF needs no special initialization)
        if hasattr(self.estimator, 'reset'):
            self.estimator.reset()
        
        print("System initialization completed successfully")
        return True
    
    def run(self, duration_seconds=60):
        """
        Run the EKF system for specified duration
        
        Args:
            duration_seconds: How long to run (default 60s)
        """
        print(f"Starting {self.estimator_type} system for {duration_seconds} seconds...")
        
        if not self.initialize():
            print("Initialization failed, cannot start system")
            return False
        
        self.is_running = True
        self.start_time = time.time()
        target_loop_time = 1.0 / 10.0  # 10 Hz target
        
        try:
            while self.is_running and (time.time() - self.start_time) < duration_seconds:
                loop_start = time.time()
                
                # Process one iteration
                self._process_iteration()
                
                # Calculate loop timing
                loop_time = time.time() - loop_start
                self.loop_times.append(loop_time)
                if loop_time > self.max_loop_time:
                    self.max_loop_time = loop_time
                
                # Maintain target frequency
                sleep_time = target_loop_time - loop_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif loop_time > target_loop_time * 2:
                    print(f"Warning: Loop time {loop_time*1000:.1f}ms exceeds target")
                
                self.iteration_count += 1
                
                # Print status periodically
                if self.iteration_count % 50 == 0:  # Every 5 seconds at 10Hz
                    self._print_status()
                
                # Memory management
                if len(self.state_history) > self.max_history_size:
                    # Remove oldest 100 entries to prevent memory overflow
                    self.state_history = self.state_history[100:]
        
        except KeyboardInterrupt:
            print("\nSystem stopped by user")
        except Exception as e:
            print(f"System error: {e}")
        finally:
            self._shutdown()
        
        # Print final summary
        self._print_final_summary()
        return True
    
    def _process_iteration(self):
        """Process one iteration of the main loop"""
        # Read sensor data
        sensor_data = self.sensor_interface.read_sensors()
        
        if sensor_data is None:
            return  # No new data available
        
        # Process with estimator
        if self.use_ekf:
            self._process_with_ekf(sensor_data)
        else:
            self._process_with_complementary_filter(sensor_data)
        
        # Store state history (memory limited)
        current_state = self._get_current_state()
        self.state_history.append({
            'timestamp': sensor_data['timestamp'],
            'state': current_state,
            'iteration': self.iteration_count
        })
    
    def _process_with_ekf(self, sensor_data):
        """Process sensor data with minimal EKF"""
        # Convert to EKF format
        reading = SensorReading()
        reading.timestamp = sensor_data['timestamp']
        reading.accel_x = sensor_data['imu']['accel_x']
        reading.accel_y = sensor_data['imu']['accel_y']
        reading.accel_z = sensor_data['imu']['accel_z']
        reading.gyro_x = sensor_data['imu']['gyro_x']
        reading.gyro_y = sensor_data['imu']['gyro_y']
        reading.gyro_z = sensor_data['imu']['gyro_z']
        reading.chassis_x = sensor_data['chassis']['x']
        reading.chassis_y = sensor_data['chassis']['y']
        reading.chassis_yaw = sensor_data['chassis']['yaw']
        
        # Process with EKF
        self.estimator.process_sensor_data(reading)
    
    def _process_with_complementary_filter(self, sensor_data):
        """Process sensor data with complementary filter"""
        current_time = sensor_data['timestamp']
        
        # Calculate dt
        if hasattr(self, 'last_filter_time'):
            dt = current_time - self.last_filter_time
        else:
            dt = 0.1  # Default
        
        self.last_filter_time = current_time
        
        # Update filter
        self.estimator.update(
            sensor_data['imu']['accel_x'],
            sensor_data['imu']['accel_y'],
            sensor_data['imu']['accel_z'],
            sensor_data['imu']['gyro_x'],
            sensor_data['imu']['gyro_y'],
            sensor_data['imu']['gyro_z'],
            dt
        )
    
    def _get_current_state(self):
        """Get current state estimate"""
        if self.use_ekf:
            return self.estimator.get_state()
        else:
            attitude = self.estimator.get_attitude()
            # Convert to common format
            return {
                'position': [0.0, 0.0, 0.0],
                'velocity': [0.0, 0.0, 0.0],
                'orientation': [attitude['roll'], attitude['pitch'], attitude['yaw']],
                'angular_velocity': [0.0, 0.0, 0.0]
            }
    
    def _print_status(self):
        """Print current system status"""
        elapsed_time = time.time() - self.start_time
        current_state = self._get_current_state()
        
        print(f"\n=== Status Update (t={elapsed_time:.1f}s, iter={self.iteration_count}) ===")
        
        # Position and orientation
        pos = current_state['position']
        ori = current_state['orientation']
        print(f"Position: x={pos[0]:.3f}m, y={pos[1]:.3f}m, z={pos[2]:.3f}m")
        print(f"Orientation: roll={ori[0]:.3f}, pitch={ori[1]:.3f}, yaw={ori[2]:.3f}")
        
        # Performance stats
        if self.loop_times:
            avg_loop_time = sum(self.loop_times[-50:]) / min(50, len(self.loop_times))
            frequency = 1.0 / avg_loop_time if avg_loop_time > 0 else 0
            print(f"Performance: {frequency:.1f} Hz avg, {self.max_loop_time*1000:.1f}ms max")
        
        # Memory usage (approximate)
        state_memory = len(self.state_history) * 200  # Rough estimate in bytes
        print(f"Memory: ~{state_memory/1024:.1f}KB state history, {len(self.state_history)} records")
        
        # Sensor stats
        sensor_stats = self.sensor_interface.get_sensor_stats()
        print(f"Sensors: {sensor_stats['sensor_errors']} errors, access_level={sensor_stats['access_level']}")
        
        # EKF-specific stats
        if self.use_ekf and hasattr(self.estimator, 'get_performance_stats'):
            ekf_stats = self.estimator.get_performance_stats()
            uncertainty = self.estimator.get_covariance_trace()
            print(f"EKF: {ekf_stats['total_updates']} updates, uncertainty={uncertainty:.3f}")
    
    def _print_final_summary(self):
        """Print final system summary"""
        elapsed_time = time.time() - self.start_time
        
        print(f"\n=== Final Summary ===")
        print(f"Total runtime: {elapsed_time:.1f} seconds")
        print(f"Total iterations: {self.iteration_count}")
        print(f"Average frequency: {self.iteration_count/elapsed_time:.1f} Hz")
        
        if self.loop_times:
            avg_loop_time = sum(self.loop_times) / len(self.loop_times)
            print(f"Average loop time: {avg_loop_time*1000:.1f}ms")
            print(f"Maximum loop time: {self.max_loop_time*1000:.1f}ms")
        
        print(f"State history records: {len(self.state_history)}")
        
        # Final state
        if self.state_history:
            final_state = self.state_history[-1]['state']
            pos = final_state['position']
            ori = final_state['orientation']
            print(f"Final position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            print(f"Final orientation: ({ori[0]:.3f}, {ori[1]:.3f}, {ori[2]:.3f})")
    
    def _shutdown(self):
        """Shutdown the system cleanly"""
        print("Shutting down system...")
        self.is_running = False
        
        try:
            self.sensor_interface.close()
        except Exception as e:
            print(f"Error closing sensor interface: {e}")
        
        print("System shutdown complete")
    
    def save_data(self, filename=None):
        """Save collected data to file (if possible)"""
        if not self.state_history:
            print("No data to save")
            return False
        
        if filename is None:
            filename = f"s1_ekf_data_{int(time.time())}.txt"
        
        try:
            with open(filename, 'w') as f:
                f.write(f"# S1 Direct EKF Data - {self.estimator_type}\n")
                f.write(f"# Access Level: {self.access_level}\n")
                f.write(f"# Total Records: {len(self.state_history)}\n")
                f.write("# timestamp,x,y,z,roll,pitch,yaw,iteration\n")
                
                for record in self.state_history:
                    t = record['timestamp']
                    state = record['state']
                    pos = state['position']
                    ori = state['orientation']
                    iter_num = record['iteration']
                    
                    f.write(f"{t:.3f},{pos[0]:.6f},{pos[1]:.6f},{pos[2]:.6f},")
                    f.write(f"{ori[0]:.6f},{ori[1]:.6f},{ori[2]:.6f},{iter_num}\n")
            
            print(f"Data saved to {filename}")
            return True
        
        except Exception as e:
            print(f"Error saving data: {e}")
            return False


def main():
    """Main entry point for S1 direct execution"""
    print("=== RoboMaster S1 Direct EKF Execution ===")
    print("This is an experimental implementation with severe limitations.")
    print("See README.md for details and warnings.\n")
    
    # System information
    try:
        import os
        print(f"Python executable: {sys.executable}")
        print(f"Current directory: {os.getcwd()}")
        print(f"Python version: {sys.version}")
        
        # Try to get memory info
        try:
            with open('/proc/meminfo', 'r') as f:
                meminfo = f.readline().strip()
                print(f"Memory info: {meminfo}")
        except:
            print("Could not read memory info")
    except:
        pass
    
    print("\n" + "="*50)
    
    # Configuration
    access_level = "app"  # Default to safest option
    use_ekf = True       # Use EKF by default
    duration = 30        # Run for 30 seconds
    
    # Parse simple command line arguments
    if len(sys.argv) > 1:
        if "adb" in sys.argv:
            access_level = "adb"
        elif "root" in sys.argv:
            access_level = "root"
        
        if "filter" in sys.argv:
            use_ekf = False
        
        for arg in sys.argv:
            if arg.startswith("time="):
                try:
                    duration = int(arg.split("=")[1])
                except:
                    pass
    
    print(f"Configuration:")
    print(f"  Access level: {access_level}")
    print(f"  Estimator: {'EKF' if use_ekf else 'Complementary Filter'}")
    print(f"  Duration: {duration} seconds")
    print()
    
    # Create and run system
    system = S1DirectEKFSystem(access_level=access_level, use_ekf=use_ekf)
    
    try:
        success = system.run(duration_seconds=duration)
        
        if success:
            print("\nAttempting to save data...")
            system.save_data()
    
    except Exception as e:
        print(f"System error: {e}")
        import traceback
        traceback.print_exc()
    
    print("\nExecution completed.")


if __name__ == "__main__":
    main()
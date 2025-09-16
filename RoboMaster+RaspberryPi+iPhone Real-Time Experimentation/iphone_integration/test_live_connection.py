#!/usr/bin/env python3
"""
Live connection test for iPhone sensor data
Run this while sending data from iPhone to test the complete pipeline
"""

import sys
import os
import time
import signal
import json

# Add the path to import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'pi_phone_connection'))

from iphone_sensor_receiver import iPhoneDataReceiver, iPhoneDataProcessor, iPhoneSensorData
from ekf_8dof_formulary import EKF8DOF, EKF8State

class LiveTest:
    def __init__(self, port=5555):
        self.port = port
        self.receiver = None
        self.processor = None
        self.ekf = None
        self.running = False
        
        # Statistics
        self.packet_count = 0
        self.last_print_time = time.time()
        self.start_time = time.time()
        
    def setup(self):
        """Setup the test components"""
        print(f"Setting up live test on port {self.port}...")
        
        # Create receiver
        self.receiver = iPhoneDataReceiver(connection_type='udp', port=self.port)
        
        # Create processor
        self.processor = iPhoneDataProcessor()
        
        # Create EKF
        ekf_config = {
            'q_position': 0.01,
            'q_velocity': 0.1,
            'q_orientation': 0.05,
            'q_angular_velocity': 0.1,
            'use_drone_velocity': False
        }
        self.ekf = EKF8DOF(ekf_config)
        
        print("✅ Setup complete")
    
    def data_callback(self, data: iPhoneSensorData):
        """Called when new sensor data arrives"""
        self.packet_count += 1
        
        current_time = time.time()
        
        # Print status every 2 seconds
        if current_time - self.last_print_time >= 2.0:
            self.print_status(data)
            self.last_print_time = current_time
        
        # Process through EKF (simplified)
        try:
            # Simple EKF processing
            dt = 0.02  # Assume 50Hz
            self.ekf.predict(dt)
            
            import numpy as np
            accel = np.array([data.accel_x, data.accel_y, data.accel_z])
            gyro = np.array([data.gyro_x, data.gyro_y, data.gyro_z])
            
            self.ekf.update_imu(accel, gyro)
            
            # Get current state
            state = self.ekf.get_state()
            
            # You can add more processing here
            
        except Exception as e:
            print(f"EKF processing error: {e}")
    
    def print_status(self, latest_data: iPhoneSensorData):
        """Print current status"""
        elapsed = time.time() - self.start_time
        rate = self.packet_count / elapsed if elapsed > 0 else 0
        
        print("\n" + "="*80)
        print(f"📊 Status (Runtime: {elapsed:.1f}s, Packets: {self.packet_count}, Rate: {rate:.1f} Hz)")
        print("="*80)
        
        # Sensor data
        print(f"📱 Latest sensor data:")
        print(f"   Accel (m/s²): [{latest_data.accel_x:6.3f}, {latest_data.accel_y:6.3f}, {latest_data.accel_z:6.3f}]")
        print(f"   Gyro (rad/s): [{latest_data.gyro_x:8.6f}, {latest_data.gyro_y:8.6f}, {latest_data.gyro_z:8.6f}]")
        
        if latest_data.mag_x is not None:
            print(f"   Mag (μT):     [{latest_data.mag_x:6.1f}, {latest_data.mag_y:6.1f}, {latest_data.mag_z:6.1f}]")
        
        if latest_data.gps_lat is not None:
            print(f"   GPS:          Lat={latest_data.gps_lat:.6f}, Lon={latest_data.gps_lon:.6f}, Alt={latest_data.gps_alt:.1f}m")
        
        if latest_data.pressure is not None:
            print(f"   Baro:         P={latest_data.pressure:.1f} Pa, Alt={latest_data.altitude:.2f}m")
        
        if latest_data.roll is not None:
            print(f"   Orient (°):   Roll={latest_data.roll*57.3:.1f}, Pitch={latest_data.pitch*57.3:.1f}, Yaw={latest_data.yaw*57.3:.1f}")
        
        # EKF state
        if self.ekf:
            state = self.ekf.get_state()
            print(f"🎯 EKF State:")
            print(f"   Position (m): [{state.x:6.2f}, {state.y:6.2f}, {state.z:6.2f}]")
            print(f"   Velocity:     vz={state.vz:6.3f} m/s")
            print(f"   Orient (°):   Roll={state.roll*57.3:5.1f}, Pitch={state.pitch*57.3:5.1f}, Yaw={state.yaw*57.3:5.1f}")
            print(f"   Yaw rate:     {state.yaw_rate*57.3:6.2f} °/s")
            
            # Uncertainties
            pos_unc = self.ekf.get_position_uncertainty()
            orient_unc = self.ekf.get_orientation_uncertainty()
            print(f"   Pos Unc (m):  [{pos_unc[0]:.3f}, {pos_unc[1]:.3f}, {pos_unc[2]:.3f}]")
            print(f"   Orient Unc:   [{orient_unc[0]*57.3:.1f}°, {orient_unc[1]*57.3:.1f}°, {orient_unc[2]*57.3:.1f}°]")
        
        # Receiver stats
        stats = self.receiver.get_statistics()
        print(f"📡 Network stats:")
        print(f"   Packets received: {stats['packets_received']}")
        print(f"   Packets dropped:  {stats['packets_dropped']}")
        print(f"   Data rate:        {stats['data_rate']:.1f} Hz")
        print(f"   Queue size:       {stats['queue_size']}")
    
    def run(self):
        """Run the live test"""
        if not self.receiver:
            print("❌ Not setup. Call setup() first.")
            return
        
        self.running = True
        
        print(f"\n🚀 Starting live test...")
        print(f"   Listening on UDP port {self.port}")
        print(f"   Send iPhone sensor data to this port")
        print(f"   Press Ctrl+C to stop")
        print("="*80)
        
        try:
            # Start receiver
            self.receiver.start(callback=self.data_callback)
            
            # Wait for data
            print("⏳ Waiting for iPhone data...")
            
            while self.running:
                time.sleep(0.1)
                
                # Check if we have data
                latest = self.receiver.get_latest_data()
                if latest and self.packet_count == 1:
                    print("✅ First packet received! Processing started...")
        
        except KeyboardInterrupt:
            print("\n🛑 Interrupted by user")
        except Exception as e:
            print(f"\n❌ Error: {e}")
        finally:
            self.stop()
    
    def stop(self):
        """Stop the test"""
        self.running = False
        if self.receiver:
            self.receiver.stop()
        
        print(f"\n📈 Final Statistics:")
        print(f"   Total packets: {self.packet_count}")
        print(f"   Total runtime: {time.time() - self.start_time:.1f}s")
        if self.packet_count > 0:
            print(f"   Average rate:  {self.packet_count / (time.time() - self.start_time):.1f} Hz")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Live iPhone sensor data test')
    parser.add_argument('--port', type=int, default=5555, help='UDP port to listen on')
    
    args = parser.parse_args()
    
    # Create and run test
    test = LiveTest(port=args.port)
    test.setup()
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\n🛑 Stopping...")
        test.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    test.run()

if __name__ == "__main__":
    main()

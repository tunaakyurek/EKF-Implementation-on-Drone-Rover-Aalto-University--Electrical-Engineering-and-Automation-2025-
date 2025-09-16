"""
Enhanced Main Integration Module for iPhone-EKF System
======================================================
Using 9-DOF Full Drone EKF with complete dynamics:
- 9-DOF Full Drone EKF: [x, y, z, vx, vy, vz, roll, pitch, yaw]

Follows RoboMaster EKF Formulary specifications with proper sensor integration
Author: RoboMaster EKF Integration System
Date: 2025
"""

import json
import logging
import time
import threading
import queue
import os
import sys
from datetime import datetime
from typing import Optional, Dict, Any, Union
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Import sensor receiver
try:
    from .iphone_sensor_receiver import iPhoneDataReceiver, iPhoneDataProcessor, iPhoneSensorData
except ImportError:
    from iphone_sensor_receiver import iPhoneDataReceiver, iPhoneDataProcessor, iPhoneSensorData

# Import 9-DOF EKF only
try:
    from .ekf_drone_9dof import EKFDrone9DOF, EKFDroneState
except ImportError:
    from ekf_drone_9dof import EKFDrone9DOF, EKFDroneState

# Optional autonomous controller
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'robomaster_control'))
try:
    from autonomous_controller import AutonomousController, ControlMode, Waypoint
    AUTONOMOUS_AVAILABLE = True
except ImportError:
    AUTONOMOUS_AVAILABLE = False
    logger.warning("Autonomous controller not available")


class EnhancediPhoneEKFIntegration:
    """
    Enhanced integration class using 9-DOF Full Drone EKF
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize the enhanced integration system with 9-DOF EKF
        
        Args:
            config_file: Path to configuration file
        """
        # Load configuration
        self.config = self._load_config(config_file)
        
        # Initialize sensor components
        self.receiver = iPhoneDataReceiver(
            connection_type=self.config.get('connection_type', 'udp'),
            port=self.config.get('port', 5555)
        )
        
        self.processor = iPhoneDataProcessor()
        
        # Initialize 9-DOF EKF
        self.ekf = EKFDrone9DOF(self.config.get('ekf_config', {}))
        
        # Optional autonomous controller
        self.autonomous_controller = None
        if AUTONOMOUS_AVAILABLE and self.config.get('enable_autonomous', False):
            self.autonomous_controller = AutonomousController(
                self.config.get('autonomous_config', {})
            )
            logger.info("Autonomous controller initialized for 9-DOF EKF")
        
        # Data logging (save in iphone_integration/data/)
        self.log_dir = self.config.get('log_dir', './data')
        self.log_file = None
        self.log_writer = None
        
        # State publishing
        self.state_queue = queue.Queue(maxsize=100)
        self.state_callback = None
        
        # Control flags
        self.is_running = False
        self.ekf_thread = None
        self.log_thread = None
        
        # Calibration
        self.is_calibrated = False
        self.calibration_data = []
        self.calibration_start_time = None
        
        # Statistics
        self.stats = {
            'ekf_updates': 0,
            'packets_processed': 0,
            'start_time': None,
            'last_update_time': None
        }
        
        # GPS reference for coordinate conversion
        self.gps_reference = {
            'lat': None,
            'lon': None,
            'set': False
        }
        
        logger.info("Enhanced iPhone-EKF integration initialized (9-DOF Full Drone)")
    
    def _load_config(self, config_file: Optional[str]) -> Dict[str, Any]:
        """Load configuration from file or use defaults"""
        default_config = {
            'connection_type': 'udp',
            'port': 5555,
            'ekf_config': {
                # 9-DOF EKF parameters
                'q_position': 0.01,
                'q_velocity': 0.1,
                'q_orientation': 0.01,
                'r_accel': 0.1,
                'r_gyro': 0.01,
                'r_mag': 0.05,
                'r_gps': 1.0,
                'r_baro': 0.5
            },
            'calibration': {
                'duration': 5.0,
                'min_samples': 100,
                'auto_calibrate': True
            },
            'logging': {
                'enabled': True,
                'rate': 50  # Hz
            }
        }
        
        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    config = json.load(f)
                # Merge with defaults
                for key, value in default_config.items():
                    if key not in config:
                        config[key] = value
                    elif isinstance(value, dict):
                        for subkey, subvalue in value.items():
                            if subkey not in config[key]:
                                config[key][subkey] = subvalue
                return config
            except Exception as e:
                logger.warning(f"Failed to load config file: {e}")
        
        return default_config
    
    def calibrate(self):
        """Perform sensor calibration"""
        calibration_config = self.config.get('calibration', {})
        duration = calibration_config.get('duration', 5.0)
        min_samples = calibration_config.get('min_samples', 100)
        # Store for callback window
        self._calibration_duration = duration
        
        logger.info(f"🔧 Starting {duration}s calibration - keep device STATIONARY")
        logger.info("   This will calibrate accelerometer, gyroscope, and magnetometer")
        
        self.calibration_data = []
        self.calibration_start_time = time.time()
        
        # Start receiver for calibration
        self.receiver.start(callback=self._calibration_callback)
        
        # Wait for calibration: require both time >= duration and samples >= min_samples,
        # but allow an extended max duration to tolerate slow links
        max_duration = max(duration * 3.0, duration + 10.0)
        last_log_count = 0
        while True:
            elapsed = time.time() - self.calibration_start_time
            time.sleep(0.1)
            if len(self.calibration_data) >= min_samples and elapsed >= duration:
                logger.info(f"✅ Calibration requirements met: samples={len(self.calibration_data)}, elapsed={elapsed:.1f}s")
                break
            if elapsed >= max_duration:
                logger.warning(f"⚠️ Calibration reached max duration {max_duration:.1f}s with samples={len(self.calibration_data)}")
                break
            if len(self.calibration_data) >= last_log_count + 50:
                last_log_count = len(self.calibration_data)
                logger.info(f"✅ Collected {len(self.calibration_data)} calibration samples (elapsed {elapsed:.1f}s)")
        
        # Stop receiver
        self.receiver.stop()
        
        # Process calibration data
        if len(self.calibration_data) >= min_samples:
            self._process_calibration_data()
            self.is_calibrated = True
            logger.info("🔧 Calibration complete - biases estimated")
        else:
            logger.error(f"❌ Insufficient calibration data: {len(self.calibration_data)} < {min_samples}")
            raise RuntimeError("Calibration failed")
    
    def _calibration_callback(self, data: iPhoneSensorData):
        """Callback for calibration data collection"""
        if not self.calibration_start_time:
            return
        # Accept samples within configured calibration window (no hard 10s cap)
        if (time.time() - self.calibration_start_time) < getattr(self, '_calibration_duration', 5.0) * 3.0:
            self.calibration_data.append(data)
    
    def _process_calibration_data(self):
        """Process calibration data to estimate biases"""
        if not self.calibration_data:
            return
        
        # Process calibration and reset EKF with calibrated state
        self.processor.calibrate(self.calibration_data, len(self.calibration_data) * 0.02)
        
        # Set initial orientation from calibration data
        # Use first sample for initial orientation estimate
        first_sample = self.calibration_data[0]
        initial_state = EKFDroneState(
            x=0.0, y=0.0, z=0.0,
            vx=0.0, vy=0.0, vz=0.0,
            roll=getattr(first_sample, 'roll', 0.0),
            pitch=getattr(first_sample, 'pitch', 0.0),
            yaw=getattr(first_sample, 'yaw', 0.0)
        )
        
        self.ekf.reset(initial_state)
        logger.info("9-DOF EKF reset with calibrated initial state")
    
    def start(self, state_callback: Optional[callable] = None):
        """Start the integration system"""
        if not self.is_calibrated and self.config.get('calibration', {}).get('auto_calibrate', True):
            self.calibrate()
        
        self.state_callback = state_callback
        self.is_running = True
        self.stats['start_time'] = time.time()
        
        # Create log file
        if self.config.get('logging', {}).get('enabled', True):
            self._create_log_file()
        
        # Start receiver
        self.receiver.start(callback=self._sensor_data_callback)
        
        # Start EKF processing thread
        self.ekf_thread = threading.Thread(target=self._ekf_processing_loop)
        self.ekf_thread.daemon = True
        self.ekf_thread.start()
        
        # Start logging thread
        if self.log_file:
            self.log_thread = threading.Thread(target=self._logging_loop)
            self.log_thread.daemon = True
            self.log_thread.start()
        
        logger.info("✅ Enhanced integration system started (9-DOF)")
    
    def stop(self):
        """Stop the integration system"""
        self.is_running = False
        
        # Stop receiver
        self.receiver.stop()
        
        # Wait for threads
        if self.ekf_thread:
            self.ekf_thread.join(timeout=2)
        if self.log_thread:
            self.log_thread.join(timeout=2)
        
        # Close log file
        if self.log_writer:
            self.log_writer.close()
        
        # Print final statistics
        self._print_final_stats()
        
        logger.info("🛑 Enhanced integration system stopped")
    
    def _sensor_data_callback(self, data: iPhoneSensorData):
        """Process incoming sensor data"""
        if not self.is_running:
            return
        
        # Process raw data
        processed_data = self.processor.process(data)
        
        # Add to queue for EKF processing
        if not self.state_queue.full():
            self.state_queue.put((data, processed_data))
            self.stats['packets_processed'] += 1
        else:
            logger.warning("State queue full, dropping data")
    
    def _ekf_processing_loop(self):
        """Main EKF processing loop"""
        last_time = time.time()
        
        while self.is_running:
            try:
                # Get sensor data
                raw_data, processed_data = self.state_queue.get(timeout=0.1)
                current_time = time.time()
                dt = current_time - last_time
                
                # Prediction step
                self.ekf.predict(dt)
                
                # Update with IMU
                self._update_with_imu(raw_data, processed_data)
                
                # Update with magnetometer
                self._update_with_magnetometer(raw_data, processed_data)
                
                # Update with GPS if available
                self._update_with_gps(raw_data, processed_data)
                
                # Update with barometer if available
                self._update_with_barometer(raw_data, processed_data)
                
                # Apply constraint updates for robust yaw estimation
                self.ekf.apply_constraint_updates()
                
                # Get current state
                current_state = self.ekf.get_state()
                
                # Update autonomous controller if available
                if self.autonomous_controller and AUTONOMOUS_AVAILABLE:
                    self._update_autonomous_controller(current_state)
                
                # Call state callback
                if self.state_callback:
                    self.state_callback(current_state)
                
                # Update statistics
                self.stats['ekf_updates'] += 1
                self.stats['last_update_time'] = current_time
                last_time = current_time
                
                # Periodic status print
                if self.stats['ekf_updates'] % 100 == 0:
                    self._print_status(current_state)
            
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"EKF processing error: {e}")
                import traceback
                traceback.print_exc()
    
    def _update_with_imu(self, raw_data: iPhoneSensorData, processed_data: Dict):
        """Update EKF with IMU measurements"""
        if 'accel' in processed_data and 'gyro' in processed_data:
            accel = np.array(processed_data['accel'])
            gyro = np.array(processed_data['gyro'])
            
            self.ekf.update_imu(accel, gyro)
    
    def _update_with_magnetometer(self, raw_data: iPhoneSensorData, processed_data: Dict):
        """Update EKF with magnetometer measurements"""
        if 'mag' in processed_data:
            mag = np.array(processed_data['mag'])
            # Use the new magnetometer yaw update method
            self.ekf.update_magnetometer_yaw(mag)
    
    def _update_with_gps(self, raw_data: iPhoneSensorData, processed_data: Dict):
        """Update EKF with GPS measurements"""
        if 'gps' in processed_data and processed_data['gps']:
            gps_data = processed_data['gps']
            if all(k in gps_data for k in ['lat', 'lon']):
                self._handle_gps_update(gps_data)
    
    def _handle_gps_update(self, gps_data: Dict):
        """Handle GPS updates with coordinate conversion"""
        lat, lon = gps_data['lat'], gps_data['lon']
        
        # Set reference point on first GPS fix
        if not self.gps_reference['set']:
            self.gps_reference['lat'] = lat
            self.gps_reference['lon'] = lon
            self.gps_reference['set'] = True
            logger.info(f"GPS reference set: {lat:.6f}, {lon:.6f}")
        
        # Convert to local coordinates
        lat_to_m = 111320.0
        lon_to_m = 111320.0 * np.cos(np.radians(self.gps_reference['lat']))
        
        x_gps = (lat - self.gps_reference['lat']) * lat_to_m
        y_gps = (lon - self.gps_reference['lon']) * lon_to_m
        z_gps = gps_data.get('altitude', 0.0)
        
        # Update position
        gps_pos = np.array([x_gps, y_gps])
        self.ekf.update_gps_position(gps_pos)
        
        # Update velocity if available
        if 'velocity' in gps_data and len(gps_data['velocity']) >= 2:
            vx_gps = gps_data['velocity'][0]
            vy_gps = gps_data['velocity'][1]
            gps_vel = np.array([vx_gps, vy_gps])
            self.ekf.update_gps_velocity(gps_vel)
        
        # Apply constraint updates after GPS updates
        self.ekf.apply_constraint_updates()
    
    def _update_with_barometer(self, raw_data: iPhoneSensorData, processed_data: Dict):
        """Update EKF with barometer measurements"""
        if 'baro' in processed_data:
            altitude = processed_data['baro']
            self.ekf.update_barometer(altitude)
    
    def _update_autonomous_controller(self, state: EKFDroneState):
        """Update autonomous controller with current state"""
        if self.autonomous_controller:
            # Convert drone state to controller format
            state_dict = {
                'x': state.x,
                'y': state.y,
                'z': state.z,
                'vx': state.vx,
                'vy': state.vy,
                'vz': state.vz,
                'roll': state.roll,
                'pitch': state.pitch,
                'yaw': state.yaw
            }
            self.autonomous_controller.update_state(state_dict)
    
    def _create_log_file(self):
        """Create CSV log file"""
        os.makedirs(self.log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"enhanced_ekf_9dof_log_{timestamp}.csv"
        self.log_file = os.path.join(self.log_dir, filename)
        
        # CSV header for 9-DOF state
        header = ("timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw,"
                 "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
                 "mag_x,mag_y,mag_z,cov_trace\n")
        
        try:
            self.log_writer = open(self.log_file, 'w')
            self.log_writer.write(header)
            logger.info(f"📝 Enhanced 9-DOF log file created: {self.log_file}")
        except Exception as e:
            logger.error(f"Failed to create log file: {e}")
            self.log_file = None
    
    def _logging_loop(self):
        """Background logging loop"""
        log_rate = self.config.get('logging', {}).get('rate', 50)
        log_interval = 1.0 / log_rate
        last_log_time = 0
        
        while self.is_running and self.log_writer:
            current_time = time.time()
            
            if current_time - last_log_time >= log_interval:
                try:
                    self._write_log_entry()
                    last_log_time = current_time
                except Exception as e:
                    logger.error(f"Logging error: {e}")
            
            time.sleep(0.001)  # Small sleep to prevent busy waiting
    
    def _write_log_entry(self):
        """Write current state to log file"""
        if not self.log_writer:
            return
        
        try:
            state = self.ekf.get_state()
            cov_trace = np.trace(self.ekf.get_covariance())
            latest_data = self.receiver.get_latest_data()
            
            if latest_data:
                # Get sensor data
                accel = [latest_data.accel_x, latest_data.accel_y, latest_data.accel_z]
                gyro = [latest_data.gyro_x, latest_data.gyro_y, latest_data.gyro_z]
                mag = [getattr(latest_data, 'mag_x', 0), getattr(latest_data, 'mag_y', 0), getattr(latest_data, 'mag_z', 0)]
                
                # Format log entry
                log_entry = (f"{time.time()},{state.x},{state.y},{state.z},"
                           f"{state.vx},{state.vy},{state.vz},"
                           f"{state.roll},{state.pitch},{state.yaw},"
                           f"{accel[0]},{accel[1]},{accel[2]},"
                           f"{gyro[0]},{gyro[1]},{gyro[2]},"
                           f"{mag[0]},{mag[1]},{mag[2]},{cov_trace}\n")
                
                self.log_writer.write(log_entry)
                self.log_writer.flush()
        
        except Exception as e:
            logger.error(f"Log write error: {e}")
    
    def _print_status(self, state: EKFDroneState):
        """Print periodic status updates"""
        runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        rate = self.stats['ekf_updates'] / runtime if runtime > 0 else 0
        
        logger.info(f"Stats: EKF updates={self.stats['ekf_updates']}, "
                   f"Packets={self.stats['packets_processed']}, Rate={rate:.1f} Hz")
        logger.info(f"State: {state}")
    
    def _print_final_stats(self):
        """Print final statistics"""
        runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        avg_rate = self.stats['ekf_updates'] / runtime if runtime > 0 else 0
        
        logger.info("📊 Final Enhanced Statistics:")
        logger.info(f"   Runtime: {runtime:.1f} seconds")
        logger.info(f"   EKF updates: {self.stats['ekf_updates']}")
        logger.info(f"   Packets processed: {self.stats['packets_processed']}")
        logger.info(f"   Average rate: {avg_rate:.1f} Hz")
    
    def get_current_state(self) -> EKFDroneState:
        """Get current EKF state"""
        return self.ekf.get_state()
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get system statistics"""
        runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        
        return {
            'runtime': runtime,
            'ekf_updates': self.stats['ekf_updates'],
            'packets_processed': self.stats['packets_processed'],
            'average_rate': self.stats['ekf_updates'] / runtime if runtime > 0 else 0,
            'is_calibrated': self.is_calibrated,
            'ekf_mode': '9dof'
        }


def main():
    """Main function for enhanced integration"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Enhanced iPhone-EKF Integration (9-DOF)')
    parser.add_argument('--config', help='Configuration file path')
    parser.add_argument('--no-calibrate', action='store_true', help='Skip auto-calibration')
    
    args = parser.parse_args()
    
    def state_callback(state: EKFDroneState):
        """State callback for printing"""
        print(f"State: {state}")
    
    # Create enhanced integration system
    integration = EnhancediPhoneEKFIntegration(args.config)
    
    if args.no_calibrate:
        integration.config['calibration']['auto_calibrate'] = False
        integration.is_calibrated = True
    
    try:
        # Start system
        integration.start(state_callback)
        
        logger.info("🚀 Enhanced 9-DOF system running. Press Ctrl+C to stop...")
        logger.info("📱 Start iPhone sensor streaming to this device")
        
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        logger.info("🛑 Stopping...")
        integration.stop()


if __name__ == "__main__":
    main()
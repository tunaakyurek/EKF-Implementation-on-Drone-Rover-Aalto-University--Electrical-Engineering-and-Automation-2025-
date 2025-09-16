"""
Main integration script for RoboMaster S1 EKF system
Run this on the Raspberry Pi
"""

import time
import signal
import sys
import threading
import logging
from typing import Optional

from ekf.ekf_core import ExtendedKalmanFilter, SensorData
from robomaster_interface.robomaster_client import RoboMasterClient
from data_collection.data_logger import DataLogger
from communication.network_client import NetworkClient, NetworkConfig

class EKFSystem:
    """Main EKF system integration class"""
    
    def __init__(self, config_file: str = "config/system_config.json"):
        # Load configuration
        self.config = self._load_config(config_file)
        
        # Initialize components
        self.ekf = ExtendedKalmanFilter()
        # Apply validated sensor-to-body alignment and yaw offset (from offline diagnostics)
        # IMU-to-body swap X/Y corresponds to a +90 deg rotation about Z
        import numpy as np
        self.ekf.R_imu_to_body = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0],
        ], dtype=float)
        self.ekf.yaw_mount_offset = np.radians(60.0)
        # Conservative magnetometer gating/noise (can be tuned further)
        self.ekf.mag_heading_gate_deg = 35.0
        self.ekf.mag_norm_gate_rel = 0.25
        self.ekf.R_mag = np.eye(3) * 2.5
        self.robomaster = RoboMasterClient(callback=self._process_sensor_data)
        self.data_logger = DataLogger(log_directory=self.config.get('log_directory', 'logs'))
        
        # Network communication
        network_config = NetworkConfig.load_config()
        self.network_client = NetworkClient(
            ground_pc_ip=network_config['ground_pc_ip'],
            port=network_config['port']
        )
        
        # System state
        self.is_running = False
        self.start_time = None
        
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('logs/system.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _load_config(self, config_file: str) -> dict:
        """Load system configuration"""
        import json
        import os
        
        default_config = {
            "sensor_frequency": 50,  # Hz
            "log_directory": "logs",
            "enable_network": True,
            "enable_logging": True,
            "ekf_params": {
                "process_noise_scale": 1.0,
                "measurement_noise_scale": 1.0
            }
        }
        
        try:
            if os.path.exists(config_file):
                with open(config_file, 'r') as f:
                    config = json.load(f)
                    default_config.update(config)
        except Exception as e:
            self.logger.warning(f"Could not load config file {config_file}: {e}")
        
        return default_config
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.logger.info(f"Received signal {signum}, shutting down...")
        self.shutdown()
        sys.exit(0)
    
    def _process_sensor_data(self, sensor_data: SensorData):
        """Process incoming sensor data with EKF"""
        try:
            # Run EKF update
            self.ekf.process_sensor_data(sensor_data)
            
            # Get current EKF state
            ekf_state = self.ekf.get_state()
            covariance_trace = float(self.ekf.P.trace())
            
            # Log data if enabled
            if self.config.get('enable_logging', True):
                self.data_logger.log_sensor_data(sensor_data)
                self.data_logger.log_ekf_state(sensor_data.timestamp, ekf_state, covariance_trace)
            
            # Send to ground PC if enabled
            if self.config.get('enable_network', True) and self.network_client.is_connected:
                self.network_client.send_sensor_data(sensor_data)
                self.network_client.send_ekf_state(sensor_data.timestamp, ekf_state, self.ekf.P)
            
            # Periodic status updates
            if hasattr(self, 'last_status_time'):
                if time.time() - self.last_status_time > 5.0:  # Every 5 seconds
                    self._send_status_update()
                    self.last_status_time = time.time()
            else:
                self.last_status_time = time.time()
        
        except Exception as e:
            self.logger.error(f"Error processing sensor data: {e}")
    
    def _send_status_update(self):
        """Send system status update"""
        try:
            robot_info = self.robomaster.get_robot_info()
            
            status = {
                "system_uptime": time.time() - self.start_time if self.start_time else 0,
                "ekf_state": {
                    "position": self.ekf.x[0:3].tolist(),
                    "covariance_trace": float(self.ekf.P.trace())
                },
                "robot_status": robot_info,
                "data_logger": {
                    "sensor_count": self.data_logger.metadata.get("sensor_count", 0),
                    "ekf_count": self.data_logger.metadata.get("ekf_count", 0)
                }
            }
            
            if self.network_client.is_connected:
                self.network_client.send_status_update(status)
        
        except Exception as e:
            self.logger.error(f"Error sending status update: {e}")
    
    def start(self) -> bool:
        """Start the EKF system"""
        self.logger.info("Starting RoboMaster S1 EKF System...")
        
        try:
            # Connect to RoboMaster S1
            self.logger.info("Connecting to RoboMaster S1...")
            if not self.robomaster.connect():
                self.logger.error("Failed to connect to RoboMaster S1")
                return False
            
            # Enable SDK mode
            self.robomaster.enable_sdk_mode()
            
            # Optional IMU calibration
            if self.config.get('calibrate_imu', False):
                self.logger.info("Performing IMU calibration...")
                self.robomaster.calibrate_imu()
            
            # Start sensor streaming
            sensor_freq = self.config.get('sensor_frequency', 50)
            if not self.robomaster.start_sensor_stream(frequency=sensor_freq):
                self.logger.error("Failed to start sensor streaming")
                return False
            
            # Connect to ground PC if enabled
            if self.config.get('enable_network', True):
                self.logger.info("Connecting to Ground PC...")
                if self.network_client.connect():
                    self.network_client.start_streaming()
                    self.logger.info("Connected to Ground PC")
                else:
                    self.logger.warning("Failed to connect to Ground PC - continuing without network")
            
            self.is_running = True
            self.start_time = time.time()
            self.logger.info("EKF System started successfully!")
            
            return True
        
        except Exception as e:
            self.logger.error(f"Error starting system: {e}")
            return False
    
    def run(self):
        """Main run loop"""
        if not self.start():
            return False
        
        try:
            self.logger.info("EKF System running... Press Ctrl+C to stop")
            
            while self.is_running:
                time.sleep(1.0)
                
                # Check system health
                if not self.robomaster.is_running:
                    self.logger.warning("RoboMaster sensor stream stopped")
                    break
        
        except KeyboardInterrupt:
            self.logger.info("Received keyboard interrupt")
        
        except Exception as e:
            self.logger.error(f"Error in main loop: {e}")
        
        finally:
            self.shutdown()
        
        return True
    
    def shutdown(self):
        """Shutdown the system gracefully"""
        if not self.is_running:
            return
        
        self.logger.info("Shutting down EKF System...")
        self.is_running = False
        
        # Stop sensor streaming
        if hasattr(self, 'robomaster'):
            self.robomaster.stop_sensor_stream()
            self.robomaster.disconnect()
        
        # Close network connection
        if hasattr(self, 'network_client'):
            self.network_client.disconnect()
        
        # Close data logger
        if hasattr(self, 'data_logger'):
            self.data_logger.close()
        
        self.logger.info("EKF System shutdown complete")

def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='RoboMaster S1 EKF System')
    parser.add_argument('--config', type=str, default='config/system_config.json',
                       help='Configuration file path')
    parser.add_argument('--log-level', type=str, default='INFO',
                       choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       help='Logging level')
    
    args = parser.parse_args()
    
    # Set logging level
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    # Create and run system
    system = EKFSystem(config_file=args.config)
    success = system.run()
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
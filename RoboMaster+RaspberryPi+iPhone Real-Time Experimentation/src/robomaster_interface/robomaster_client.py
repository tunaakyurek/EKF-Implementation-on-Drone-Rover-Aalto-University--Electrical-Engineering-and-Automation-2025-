"""
RoboMaster S1 interface for sensor data collection
"""

import time
import numpy as np
import threading
from typing import Callable, Optional
from robomaster import robot, camera, gimbal, chassis
from dataclasses import dataclass
import logging

from ..ekf.ekf_core import SensorData

class RoboMasterClient:
    """Interface to RoboMaster S1 for sensor data collection"""
    
    def __init__(self, callback: Optional[Callable[[SensorData], None]] = None):
        self.robot = None
        self.callback = callback
        self.is_running = False
        self.sensor_thread = None
        
        # Sensor data accumulation
        self.latest_imu = None
        self.latest_chassis = None
        
        # Configure logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
    def connect(self) -> bool:
        """Connect to RoboMaster S1"""
        try:
            self.robot = robot.Robot()
            self.robot.initialize(conn_type="ap")
            
            # Initialize sensor modules
            self.robot.sensor_adaptor.process_function = self._sensor_data_handler
            
            self.logger.info("Successfully connected to RoboMaster S1")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect to RoboMaster S1: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from RoboMaster S1"""
        if self.robot:
            self.stop_sensor_stream()
            self.robot.close()
            self.logger.info("Disconnected from RoboMaster S1")
    
    def start_sensor_stream(self, frequency: int = 50):
        """Start streaming sensor data at specified frequency (Hz)"""
        if not self.robot:
            self.logger.error("Robot not connected")
            return False
        
        try:
            # Subscribe to sensor data
            self.robot.sensor.sub_attitude(freq=frequency, callback=self._attitude_handler)
            self.robot.sensor.sub_gyroscope(freq=frequency, callback=self._gyro_handler)
            self.robot.sensor.sub_accelerometer(freq=frequency, callback=self._accel_handler)
            self.robot.chassis.sub_position(freq=frequency, callback=self._position_handler)
            
            self.is_running = True
            
            # Start sensor fusion thread
            self.sensor_thread = threading.Thread(target=self._sensor_fusion_loop)
            self.sensor_thread.daemon = True
            self.sensor_thread.start()
            
            self.logger.info(f"Started sensor streaming at {frequency} Hz")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start sensor stream: {e}")
            return False
    
    def stop_sensor_stream(self):
        """Stop sensor data streaming"""
        if self.robot and self.is_running:
            self.is_running = False
            
            # Unsubscribe from all sensors
            self.robot.sensor.unsub_attitude()
            self.robot.sensor.unsub_gyroscope()
            self.robot.sensor.unsub_accelerometer()
            self.robot.chassis.unsub_position()
            
            if self.sensor_thread:
                self.sensor_thread.join(timeout=1.0)
            
            self.logger.info("Stopped sensor streaming")
    
    def _attitude_handler(self, attitude_info):
        """Handle attitude sensor data (roll, pitch, yaw)"""
        # attitude_info contains: pitch, roll, yaw in degrees
        if not hasattr(self, 'latest_attitude'):
            self.latest_attitude = {}
        
        self.latest_attitude.update({
            'timestamp': time.time(),
            'roll': np.radians(attitude_info[1]),    # Convert to radians
            'pitch': np.radians(attitude_info[0]),   # Convert to radians
            'yaw': np.radians(attitude_info[2])      # Convert to radians
        })
    
    def _gyro_handler(self, gyro_info):
        """Handle gyroscope data"""
        # gyro_info contains: [wx, wy, wz] in degrees/second
        if not hasattr(self, 'latest_gyro'):
            self.latest_gyro = {}
        
        self.latest_gyro.update({
            'timestamp': time.time(),
            'wx': np.radians(gyro_info[0]),  # Convert to rad/s
            'wy': np.radians(gyro_info[1]),
            'wz': np.radians(gyro_info[2])
        })
    
    def _accel_handler(self, accel_info):
        """Handle accelerometer data"""
        # accel_info contains: [ax, ay, az] in m/s²
        if not hasattr(self, 'latest_accel'):
            self.latest_accel = {}
        
        self.latest_accel.update({
            'timestamp': time.time(),
            'ax': accel_info[0],
            'ay': accel_info[1],
            'az': accel_info[2]
        })
    
    def _position_handler(self, position_info):
        """Handle chassis position data"""
        # position_info contains: [x, y, z, yaw] where z is not reliable
        if not hasattr(self, 'latest_position'):
            self.latest_position = {}
        
        self.latest_position.update({
            'timestamp': time.time(),
            'x': position_info[0] / 1000.0,  # Convert mm to meters
            'y': position_info[1] / 1000.0,  # Convert mm to meters
            'yaw': np.radians(position_info[3])  # Convert to radians
        })
    
    def _sensor_data_handler(self, sensor_data):
        """Handle raw sensor adapter data (if available)"""
        # This is called when sensor adapter data is available
        # Implementation depends on specific sensor adapter capabilities
        pass
    
    def _sensor_fusion_loop(self):
        """Main sensor fusion loop that combines all sensor data"""
        while self.is_running:
            try:
                # Check if we have recent data from all sensors
                current_time = time.time()
                timeout = 0.1  # 100ms timeout for recent data
                
                # Collect latest data
                accel_data = getattr(self, 'latest_accel', None)
                gyro_data = getattr(self, 'latest_gyro', None)
                attitude_data = getattr(self, 'latest_attitude', None)
                position_data = getattr(self, 'latest_position', None)
                
                # Check if data is recent enough
                if (accel_data and gyro_data and 
                    (current_time - accel_data['timestamp']) < timeout and
                    (current_time - gyro_data['timestamp']) < timeout):
                    
                    # Create sensor data object
                    sensor_data = SensorData(
                        timestamp=current_time,
                        accel=np.array([accel_data['ax'], accel_data['ay'], accel_data['az']]),
                        gyro=np.array([gyro_data['wx'], gyro_data['wy'], gyro_data['wz']]),
                        mag=None,  # RoboMaster S1 doesn't have magnetometer
                        pressure=None,  # No barometer
                        altitude=None,
                        gps_lat=None,  # No GPS on RoboMaster S1
                        gps_lon=None,
                        gps_alt=None,
                        chassis_x=position_data['x'] if position_data else None,
                        chassis_y=position_data['y'] if position_data else None,
                        chassis_yaw=position_data['yaw'] if position_data else None
                    )
                    
                    # Send to callback if available
                    if self.callback:
                        self.callback(sensor_data)
                
            except Exception as e:
                self.logger.error(f"Error in sensor fusion loop: {e}")
            
            time.sleep(0.02)  # 50 Hz processing rate
    
    def get_robot_info(self) -> dict:
        """Get robot information and status"""
        if not self.robot:
            return {"connected": False}
        
        try:
            # Get battery level
            battery_level = self.robot.battery.get_battery()
            
            return {
                "connected": True,
                "battery_level": battery_level,
                "is_streaming": self.is_running
            }
        except Exception as e:
            self.logger.error(f"Error getting robot info: {e}")
            return {"connected": False, "error": str(e)}
    
    def enable_sdk_mode(self):
        """Enable SDK mode for programmatic control"""
        if self.robot:
            try:
                self.robot.set_robot_mode(mode=robot.PLAY_MODE)
                self.logger.info("SDK mode enabled")
            except Exception as e:
                self.logger.error(f"Failed to enable SDK mode: {e}")
    
    def get_camera_stream(self):
        """Get camera stream (for visual odometry if needed)"""
        if self.robot:
            try:
                self.robot.camera.start_video_stream(display=False)
                return self.robot.camera
            except Exception as e:
                self.logger.error(f"Failed to start camera stream: {e}")
                return None
        return None
    
    def calibrate_imu(self) -> bool:
        """Perform IMU calibration"""
        if not self.robot:
            return False
        
        try:
            self.logger.info("Starting IMU calibration - keep robot stationary")
            
            # Collect calibration data
            calibration_samples = []
            sample_count = 100
            
            for i in range(sample_count):
                if hasattr(self, 'latest_accel') and hasattr(self, 'latest_gyro'):
                    calibration_samples.append({
                        'accel': [self.latest_accel['ax'], self.latest_accel['ay'], self.latest_accel['az']],
                        'gyro': [self.latest_gyro['wx'], self.latest_gyro['wy'], self.latest_gyro['wz']]
                    })
                time.sleep(0.05)  # 20 Hz sampling
            
            if len(calibration_samples) > 50:
                # Compute bias offsets
                accels = np.array([s['accel'] for s in calibration_samples])
                gyros = np.array([s['gyro'] for s in calibration_samples])
                
                self.accel_bias = np.mean(accels, axis=0)
                self.gyro_bias = np.mean(gyros, axis=0)
                
                # Adjust for gravity (assuming robot is level)
                self.accel_bias[2] -= 9.81  # Remove gravity from z-axis
                
                self.logger.info("IMU calibration completed")
                self.logger.info(f"Accel bias: {self.accel_bias}")
                self.logger.info(f"Gyro bias: {self.gyro_bias}")
                
                return True
        
        except Exception as e:
            self.logger.error(f"IMU calibration failed: {e}")
        
        return False
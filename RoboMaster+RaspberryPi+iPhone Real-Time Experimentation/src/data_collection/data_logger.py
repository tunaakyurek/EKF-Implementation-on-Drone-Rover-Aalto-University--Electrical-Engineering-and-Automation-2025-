"""
Data logging and collection system for EKF analysis
"""

import csv
import json
import time
import os
from datetime import datetime
from typing import List, Dict, Any, Optional
import numpy as np
from dataclasses import asdict

from ..ekf.ekf_core import SensorData, EKFState

class DataLogger:
    """Logs sensor data and EKF states for analysis"""
    
    def __init__(self, log_directory: str = "logs"):
        self.log_directory = log_directory
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create log directory if it doesn't exist
        os.makedirs(log_directory, exist_ok=True)
        
        # Initialize log files
        self.sensor_log_file = os.path.join(log_directory, f"sensor_data_{self.session_id}.csv")
        self.ekf_log_file = os.path.join(log_directory, f"ekf_states_{self.session_id}.csv")
        self.metadata_file = os.path.join(log_directory, f"metadata_{self.session_id}.json")
        
        # Initialize CSV writers
        self._init_csv_files()
        
        # Session metadata
        self.metadata = {
            "session_id": self.session_id,
            "start_time": time.time(),
            "sensor_count": 0,
            "ekf_count": 0
        }
    
    def _init_csv_files(self):
        """Initialize CSV files with headers"""
        # Sensor data CSV
        sensor_headers = [
            'timestamp', 'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'mag_x', 'mag_y', 'mag_z',
            'pressure', 'altitude',
            'gps_lat', 'gps_lon', 'gps_alt',
            'chassis_x', 'chassis_y', 'chassis_yaw'
        ]
        
        with open(self.sensor_log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(sensor_headers)
        
        # EKF state CSV
        ekf_headers = [
            'timestamp', 'pos_x', 'pos_y', 'pos_z',
            'vel_x', 'vel_y', 'vel_z',
            'roll', 'pitch', 'yaw',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
            'covariance_trace'
        ]
        
        with open(self.ekf_log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(ekf_headers)
    
    def log_sensor_data(self, sensor_data: SensorData):
        """Log sensor data to CSV"""
        try:
            row = [
                sensor_data.timestamp,
                sensor_data.accel[0] if sensor_data.accel is not None else None,
                sensor_data.accel[1] if sensor_data.accel is not None else None,
                sensor_data.accel[2] if sensor_data.accel is not None else None,
                sensor_data.gyro[0] if sensor_data.gyro is not None else None,
                sensor_data.gyro[1] if sensor_data.gyro is not None else None,
                sensor_data.gyro[2] if sensor_data.gyro is not None else None,
                sensor_data.mag[0] if sensor_data.mag is not None else None,
                sensor_data.mag[1] if sensor_data.mag is not None else None,
                sensor_data.mag[2] if sensor_data.mag is not None else None,
                sensor_data.pressure,
                sensor_data.altitude,
                sensor_data.gps_lat,
                sensor_data.gps_lon,
                sensor_data.gps_alt,
                sensor_data.chassis_x,
                sensor_data.chassis_y,
                sensor_data.chassis_yaw
            ]
            
            with open(self.sensor_log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
            
            self.metadata["sensor_count"] += 1
            
        except Exception as e:
            print(f"Error logging sensor data: {e}")
    
    def log_ekf_state(self, timestamp: float, ekf_state: EKFState, covariance_trace: float):
        """Log EKF state to CSV"""
        try:
            row = [
                timestamp,
                ekf_state.position[0],
                ekf_state.position[1],
                ekf_state.position[2],
                ekf_state.velocity[0],
                ekf_state.velocity[1],
                ekf_state.velocity[2],
                ekf_state.orientation[0],
                ekf_state.orientation[1],
                ekf_state.orientation[2],
                ekf_state.angular_velocity[0],
                ekf_state.angular_velocity[1],
                ekf_state.angular_velocity[2],
                covariance_trace
            ]
            
            with open(self.ekf_log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
            
            self.metadata["ekf_count"] += 1
            
        except Exception as e:
            print(f"Error logging EKF state: {e}")
    
    def save_metadata(self, additional_metadata: Optional[Dict] = None):
        """Save session metadata"""
        if additional_metadata:
            self.metadata.update(additional_metadata)
        
        self.metadata["end_time"] = time.time()
        self.metadata["duration"] = self.metadata["end_time"] - self.metadata["start_time"]
        
        with open(self.metadata_file, 'w') as f:
            json.dump(self.metadata, f, indent=2)
    
    def close(self):
        """Close logger and save metadata"""
        self.save_metadata()

class DataAnalyzer:
    """Analyze logged data for performance evaluation"""
    
    def __init__(self, log_directory: str = "logs"):
        self.log_directory = log_directory
    
    def load_session_data(self, session_id: str) -> Dict[str, Any]:
        """Load data from a specific session"""
        sensor_file = os.path.join(self.log_directory, f"sensor_data_{session_id}.csv")
        ekf_file = os.path.join(self.log_directory, f"ekf_states_{session_id}.csv")
        metadata_file = os.path.join(self.log_directory, f"metadata_{session_id}.json")
        
        data = {}
        
        # Load sensor data
        try:
            import pandas as pd
            data['sensors'] = pd.read_csv(sensor_file)
        except ImportError:
            print("Pandas not available, using basic CSV loading")
            data['sensors'] = self._load_csv_basic(sensor_file)
        
        # Load EKF data
        try:
            data['ekf'] = pd.read_csv(ekf_file)
        except:
            data['ekf'] = self._load_csv_basic(ekf_file)
        
        # Load metadata
        if os.path.exists(metadata_file):
            with open(metadata_file, 'r') as f:
                data['metadata'] = json.load(f)
        
        return data
    
    def _load_csv_basic(self, filename: str) -> List[Dict]:
        """Basic CSV loading without pandas"""
        data = []
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Convert numeric values
                for key, value in row.items():
                    if value and value != 'None':
                        try:
                            row[key] = float(value)
                        except ValueError:
                            pass
                data.append(row)
        return data
    
    def compute_rmse(self, session_id: str, reference_trajectory: Optional[np.ndarray] = None) -> Dict[str, float]:
        """Compute RMSE for position estimation"""
        data = self.load_session_data(session_id)
        
        if reference_trajectory is None:
            # Use chassis position as reference (if available)
            sensors = data['sensors']
            ekf = data['ekf']
            
            # Extract positions
            chassis_x = [s.get('chassis_x') for s in sensors if s.get('chassis_x') is not None]
            chassis_y = [s.get('chassis_y') for s in sensors if s.get('chassis_y') is not None]
            
            ekf_x = [e.get('pos_x') for e in ekf if e.get('pos_x') is not None]
            ekf_y = [e.get('pos_y') for e in ekf if e.get('pos_y') is not None]
            
            if len(chassis_x) == 0 or len(ekf_x) == 0:
                return {"error": "Insufficient data for RMSE calculation"}
            
            # Interpolate to common time base (simplified)
            min_len = min(len(chassis_x), len(ekf_x))
            chassis_x = chassis_x[:min_len]
            chassis_y = chassis_y[:min_len]
            ekf_x = ekf_x[:min_len]
            ekf_y = ekf_y[:min_len]
            
            # Compute RMSE
            rmse_x = np.sqrt(np.mean([(cx - ex)**2 for cx, ex in zip(chassis_x, ekf_x)]))
            rmse_y = np.sqrt(np.mean([(cy - ey)**2 for cy, ey in zip(chassis_y, ekf_y)]))
            rmse_total = np.sqrt(rmse_x**2 + rmse_y**2)
            
            return {
                "rmse_x": rmse_x,
                "rmse_y": rmse_y,
                "rmse_total": rmse_total
            }
        
        return {"error": "Reference trajectory comparison not implemented"}
    
    def get_available_sessions(self) -> List[str]:
        """Get list of available session IDs"""
        sessions = []
        for file in os.listdir(self.log_directory):
            if file.startswith("metadata_") and file.endswith(".json"):
                session_id = file.replace("metadata_", "").replace(".json", "")
                sessions.append(session_id)
        return sorted(sessions)
    
    def generate_summary_report(self, session_id: str) -> str:
        """Generate a summary report for a session"""
        data = self.load_session_data(session_id)
        
        report = f"=== Session {session_id} Summary ===\n"
        
        if 'metadata' in data:
            metadata = data['metadata']
            report += f"Duration: {metadata.get('duration', 0):.1f} seconds\n"
            report += f"Sensor samples: {metadata.get('sensor_count', 0)}\n"
            report += f"EKF states: {metadata.get('ekf_count', 0)}\n"
        
        # Add RMSE if available
        rmse = self.compute_rmse(session_id)
        if 'error' not in rmse:
            report += f"RMSE X: {rmse.get('rmse_x', 0):.4f} m\n"
            report += f"RMSE Y: {rmse.get('rmse_y', 0):.4f} m\n"
            report += f"RMSE Total: {rmse.get('rmse_total', 0):.4f} m\n"
        
        return report
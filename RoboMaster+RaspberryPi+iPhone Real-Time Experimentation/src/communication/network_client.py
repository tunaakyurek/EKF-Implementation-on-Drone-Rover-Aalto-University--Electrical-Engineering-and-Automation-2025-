"""
Network communication between Raspberry Pi and Ground PC
"""

import zmq
import json
import time
import threading
from typing import Dict, Any, Callable, Optional
import numpy as np
from dataclasses import asdict
import logging

from ..ekf.ekf_core import SensorData, EKFState

class NetworkClient:
    """Handles network communication from Raspberry Pi to Ground PC"""
    
    def __init__(self, ground_pc_ip: str = "192.168.1.100", port: int = 5555):
        self.ground_pc_ip = ground_pc_ip
        self.port = port
        self.context = zmq.Context()
        self.socket = None
        self.is_connected = False
        self.is_running = False
        
        # Data queues
        self.data_queue = []
        self.max_queue_size = 1000
        
        # Communication thread
        self.comm_thread = None
        self.lock = threading.Lock()
        
        # Configure logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def connect(self) -> bool:
        """Connect to Ground PC"""
        try:
            self.socket = self.context.socket(zmq.PUSH)
            self.socket.connect(f"tcp://{self.ground_pc_ip}:{self.port}")
            
            # Test connection
            test_message = {
                "type": "connection_test",
                "timestamp": time.time(),
                "message": "RaspberryPi EKF System Connected"
            }
            self.socket.send_string(json.dumps(test_message), zmq.NOBLOCK)
            
            self.is_connected = True
            self.logger.info(f"Connected to Ground PC at {self.ground_pc_ip}:{self.port}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect to Ground PC: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Ground PC"""
        self.is_running = False
        
        if self.comm_thread:
            self.comm_thread.join(timeout=2.0)
        
        if self.socket:
            self.socket.close()
        
        self.is_connected = False
        self.logger.info("Disconnected from Ground PC")
    
    def start_streaming(self):
        """Start streaming data to Ground PC"""
        if not self.is_connected:
            self.logger.error("Not connected to Ground PC")
            return False
        
        self.is_running = True
        self.comm_thread = threading.Thread(target=self._communication_loop)
        self.comm_thread.daemon = True
        self.comm_thread.start()
        
        self.logger.info("Started data streaming to Ground PC")
        return True
    
    def send_sensor_data(self, sensor_data: SensorData):
        """Queue sensor data for transmission"""
        data_dict = {
            "type": "sensor_data",
            "timestamp": sensor_data.timestamp,
            "data": self._sensor_data_to_dict(sensor_data)
        }
        
        with self.lock:
            self.data_queue.append(data_dict)
            
            # Maintain queue size
            if len(self.data_queue) > self.max_queue_size:
                self.data_queue.pop(0)
    
    def send_ekf_state(self, timestamp: float, ekf_state: EKFState, covariance: np.ndarray):
        """Queue EKF state for transmission"""
        data_dict = {
            "type": "ekf_state",
            "timestamp": timestamp,
            "data": {
                "position": ekf_state.position.tolist(),
                "velocity": ekf_state.velocity.tolist(),
                "orientation": ekf_state.orientation.tolist(),
                "angular_velocity": ekf_state.angular_velocity.tolist(),
                "covariance_trace": float(np.trace(covariance))
            }
        }
        
        with self.lock:
            self.data_queue.append(data_dict)
            
            if len(self.data_queue) > self.max_queue_size:
                self.data_queue.pop(0)
    
    def send_status_update(self, status: Dict[str, Any]):
        """Send system status update"""
        data_dict = {
            "type": "status_update",
            "timestamp": time.time(),
            "data": status
        }
        
        with self.lock:
            self.data_queue.append(data_dict)
    
    def _sensor_data_to_dict(self, sensor_data: SensorData) -> Dict[str, Any]:
        """Convert SensorData to dictionary"""
        return {
            "accel": sensor_data.accel.tolist() if sensor_data.accel is not None else None,
            "gyro": sensor_data.gyro.tolist() if sensor_data.gyro is not None else None,
            "mag": sensor_data.mag.tolist() if sensor_data.mag is not None else None,
            "pressure": sensor_data.pressure,
            "altitude": sensor_data.altitude,
            "gps_lat": sensor_data.gps_lat,
            "gps_lon": sensor_data.gps_lon,
            "gps_alt": sensor_data.gps_alt,
            "chassis_x": sensor_data.chassis_x,
            "chassis_y": sensor_data.chassis_y,
            "chassis_yaw": sensor_data.chassis_yaw
        }
    
    def _communication_loop(self):
        """Main communication loop"""
        while self.is_running and self.is_connected:
            try:
                # Get data from queue
                with self.lock:
                    if self.data_queue:
                        data_to_send = self.data_queue.copy()
                        self.data_queue.clear()
                    else:
                        data_to_send = []
                
                # Send data
                for data in data_to_send:
                    try:
                        self.socket.send_string(json.dumps(data), zmq.NOBLOCK)
                    except zmq.Again:
                        # Socket would block, skip this message
                        pass
                    except Exception as e:
                        self.logger.error(f"Error sending data: {e}")
                
                time.sleep(0.01)  # 100 Hz communication rate
                
            except Exception as e:
                self.logger.error(f"Error in communication loop: {e}")
                time.sleep(0.1)

class GroundStationServer:
    """Ground PC server for receiving data from Raspberry Pi"""
    
    def __init__(self, port: int = 5555, callback: Optional[Callable[[Dict], None]] = None):
        self.port = port
        self.callback = callback
        self.context = zmq.Context()
        self.socket = None
        self.is_running = False
        self.server_thread = None
        
        # Data storage
        self.received_data = []
        self.max_stored_data = 10000
        
        # Configure logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def start_server(self) -> bool:
        """Start the ground station server"""
        try:
            self.socket = self.context.socket(zmq.PULL)
            self.socket.bind(f"tcp://*:{self.port}")
            
            self.is_running = True
            self.server_thread = threading.Thread(target=self._server_loop)
            self.server_thread.daemon = True
            self.server_thread.start()
            
            self.logger.info(f"Ground station server started on port {self.port}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start server: {e}")
            return False
    
    def stop_server(self):
        """Stop the ground station server"""
        self.is_running = False
        
        if self.server_thread:
            self.server_thread.join(timeout=2.0)
        
        if self.socket:
            self.socket.close()
        
        self.logger.info("Ground station server stopped")
    
    def _server_loop(self):
        """Main server loop for receiving data"""
        while self.is_running:
            try:
                # Receive data with timeout
                if self.socket.poll(timeout=100):  # 100ms timeout
                    message = self.socket.recv_string(zmq.NOBLOCK)
                    data = json.loads(message)
                    
                    # Store data
                    self.received_data.append(data)
                    
                    # Maintain storage limit
                    if len(self.received_data) > self.max_stored_data:
                        self.received_data.pop(0)
                    
                    # Call callback if provided
                    if self.callback:
                        self.callback(data)
                
            except zmq.Again:
                # No message available
                continue
            except Exception as e:
                self.logger.error(f"Error in server loop: {e}")
                time.sleep(0.1)
    
    def get_latest_data(self, data_type: Optional[str] = None, count: int = 100) -> list:
        """Get latest received data"""
        if data_type:
            filtered_data = [d for d in self.received_data if d.get('type') == data_type]
            return filtered_data[-count:]
        else:
            return self.received_data[-count:]
    
    def save_received_data(self, filename: str):
        """Save all received data to file"""
        with open(filename, 'w') as f:
            json.dump(self.received_data, f, indent=2)
        self.logger.info(f"Saved {len(self.received_data)} messages to {filename}")

# Configuration loader
class NetworkConfig:
    """Network configuration management"""
    
    @staticmethod
    def load_config(config_file: str = "config/network_config.json") -> Dict[str, Any]:
        """Load network configuration from file"""
        default_config = {
            "ground_pc_ip": "192.168.1.100",
            "port": 5555,
            "max_queue_size": 1000,
            "communication_rate": 100,  # Hz
            "timeout": 5.0  # seconds
        }
        
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
                default_config.update(config)
        except FileNotFoundError:
            # Create default config file
            os.makedirs(os.path.dirname(config_file), exist_ok=True)
            with open(config_file, 'w') as f:
                json.dump(default_config, f, indent=2)
        
        return default_config
    
    @staticmethod
    def save_config(config: Dict[str, Any], config_file: str = "config/network_config.json"):
        """Save network configuration to file"""
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=2)
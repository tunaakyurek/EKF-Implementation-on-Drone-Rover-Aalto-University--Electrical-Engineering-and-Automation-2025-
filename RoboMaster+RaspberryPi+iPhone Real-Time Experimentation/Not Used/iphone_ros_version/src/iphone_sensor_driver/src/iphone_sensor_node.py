#!/usr/bin/env python3
"""
iPhone Sensor ROS Driver Node
==============================
Receives sensor data from iPhone and publishes to ROS topics
Supports UDP/TCP connections and automatic calibration

Author: RoboMaster EKF ROS Integration
Date: 2025
"""

import rospy
import socket
import json
import threading
import numpy as np
from typing import Optional, Dict, Any
from collections import deque

from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu, NavSatFix, MagneticField, FluidPressure, Temperature
from std_msgs.msg import Header, Float64
from std_srvs.srv import Trigger, TriggerResponse

# Import custom message
from iphone_ekf_fusion.msg import IPhoneSensorData


class iPhoneSensorDriver:
    """
    ROS driver for iPhone sensor data
    Handles network communication and data conversion
    """
    
    def __init__(self):
        """Initialize iPhone sensor driver"""
        rospy.init_node('iphone_sensor_driver', anonymous=False)
        
        # Connection parameters
        self.connection_type = rospy.get_param('~connection_type', 'udp')
        self.port = rospy.get_param('~port', 5555)
        self.buffer_size = rospy.get_param('~buffer_size', 4096)
        
        # Calibration parameters
        self.calibration_samples = rospy.get_param('~calibration_samples', 200)
        self.auto_calibrate = rospy.get_param('~auto_calibrate', True)
        
        # Publishers
        self.iphone_pub = rospy.Publisher(
            '/iphone/sensor_data', IPhoneSensorData, queue_size=1
        )
        self.imu_pub = rospy.Publisher(
            '/iphone/imu', Imu, queue_size=1
        )
        self.gps_pub = rospy.Publisher(
            '/iphone/gps', NavSatFix, queue_size=1
        )
        self.mag_pub = rospy.Publisher(
            '/iphone/mag', MagneticField, queue_size=1
        )
        self.pressure_pub = rospy.Publisher(
            '/iphone/pressure', FluidPressure, queue_size=1
        )
        self.temperature_pub = rospy.Publisher(
            '/iphone/temperature', Temperature, queue_size=1
        )
        
        # Services
        self.calibrate_srv = rospy.Service(
            '~calibrate', Trigger, self.calibrate_service
        )
        self.reset_srv = rospy.Service(
            '~reset', Trigger, self.reset_service
        )
        
        # Calibration data
        self.calibration = {
            'accel_bias': np.zeros(3),
            'gyro_bias': np.zeros(3),
            'mag_bias': np.zeros(3),
            'mag_scale': np.ones(3),
            'is_calibrated': False
        }
        
        # Data buffers for calibration
        self.calibration_buffer = deque(maxlen=self.calibration_samples)
        
        # Connection management
        self.socket = None
        self.is_connected = False
        self.receive_thread = None
        self.shutdown_flag = threading.Event()
        
        # Statistics
        self.packets_received = 0
        self.packets_dropped = 0
        self.last_packet_time = rospy.Time.now()
        
        # Start connection
        self.start_connection()
        
        # Auto-calibrate if enabled
        if self.auto_calibrate:
            rospy.Timer(rospy.Duration(2.0), self.auto_calibrate_callback, oneshot=True)
        
        rospy.loginfo(f"iPhone sensor driver initialized on {self.connection_type}:{self.port}")
    
    def start_connection(self):
        """Start network connection"""
        if self.connection_type == 'udp':
            self.start_udp_connection()
        elif self.connection_type == 'tcp':
            self.start_tcp_connection()
        else:
            rospy.logerr(f"Unknown connection type: {self.connection_type}")
    
    def start_udp_connection(self):
        """Start UDP listener"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(('', self.port))
            self.socket.settimeout(1.0)
            
            self.is_connected = True
            self.receive_thread = threading.Thread(target=self.udp_receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            rospy.loginfo(f"UDP listener started on port {self.port}")
        except Exception as e:
            rospy.logerr(f"Failed to start UDP connection: {e}")
    
    def start_tcp_connection(self):
        """Start TCP server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(('', self.port))
            self.socket.listen(1)
            self.socket.settimeout(1.0)
            
            self.is_connected = True
            self.receive_thread = threading.Thread(target=self.tcp_receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            rospy.loginfo(f"TCP server started on port {self.port}")
        except Exception as e:
            rospy.logerr(f"Failed to start TCP connection: {e}")
    
    def udp_receive_loop(self):
        """UDP receive loop"""
        while not self.shutdown_flag.is_set() and not rospy.is_shutdown():
            try:
                data, addr = self.socket.recvfrom(self.buffer_size)
                self.process_raw_data(data)
            except socket.timeout:
                continue
            except Exception as e:
                if not self.shutdown_flag.is_set():
                    rospy.logerr(f"UDP receive error: {e}")
    
    def tcp_receive_loop(self):
        """TCP receive loop"""
        while not self.shutdown_flag.is_set() and not rospy.is_shutdown():
            try:
                # Accept connection
                client_socket, addr = self.socket.accept()
                rospy.loginfo(f"iPhone connected from {addr}")
                client_socket.settimeout(1.0)
                
                # Receive data from client
                while not self.shutdown_flag.is_set():
                    try:
                        data = client_socket.recv(self.buffer_size)
                        if not data:
                            break
                        self.process_raw_data(data)
                    except socket.timeout:
                        continue
                    except Exception as e:
                        rospy.logerr(f"TCP client error: {e}")
                        break
                
                client_socket.close()
                rospy.loginfo("iPhone disconnected")
                
            except socket.timeout:
                continue
            except Exception as e:
                if not self.shutdown_flag.is_set():
                    rospy.logerr(f"TCP server error: {e}")
    
    def process_raw_data(self, data: bytes):
        """Process raw data from iPhone"""
        try:
            # Decode JSON data
            json_data = json.loads(data.decode('utf-8'))
            
            # Update statistics
            self.packets_received += 1
            self.last_packet_time = rospy.Time.now()
            
            # Store for calibration if needed
            if not self.calibration['is_calibrated']:
                self.calibration_buffer.append(json_data)
            
            # Apply calibration and publish
            calibrated_data = self.apply_calibration(json_data)
            self.publish_sensor_data(calibrated_data)
            
        except json.JSONDecodeError as e:
            rospy.logwarn(f"Invalid JSON data: {e}")
            self.packets_dropped += 1
        except Exception as e:
            rospy.logerr(f"Error processing data: {e}")
            self.packets_dropped += 1
    
    def apply_calibration(self, raw_data: Dict) -> Dict:
        """Apply calibration to raw sensor data"""
        calibrated = raw_data.copy()
        
        if self.calibration['is_calibrated']:
            # Apply accelerometer calibration
            if 'accel_x' in calibrated:
                calibrated['accel_x'] -= self.calibration['accel_bias'][0]
                calibrated['accel_y'] -= self.calibration['accel_bias'][1]
                calibrated['accel_z'] -= self.calibration['accel_bias'][2]
            
            # Apply gyroscope calibration
            if 'gyro_x' in calibrated:
                calibrated['gyro_x'] -= self.calibration['gyro_bias'][0]
                calibrated['gyro_y'] -= self.calibration['gyro_bias'][1]
                calibrated['gyro_z'] -= self.calibration['gyro_bias'][2]
            
            # Apply magnetometer calibration
            if 'mag_x' in calibrated:
                calibrated['mag_x'] = (calibrated['mag_x'] - self.calibration['mag_bias'][0]) * \
                                     self.calibration['mag_scale'][0]
                calibrated['mag_y'] = (calibrated['mag_y'] - self.calibration['mag_bias'][1]) * \
                                     self.calibration['mag_scale'][1]
                calibrated['mag_z'] = (calibrated['mag_z'] - self.calibration['mag_bias'][2]) * \
                                     self.calibration['mag_scale'][2]
        
        return calibrated
    
    def publish_sensor_data(self, data: Dict):
        """Publish sensor data to ROS topics"""
        now = rospy.Time.now()
        
        # Create and publish IPhoneSensorData message
        iphone_msg = self.create_iphone_msg(data, now)
        self.iphone_pub.publish(iphone_msg)
        
        # Publish individual sensor messages
        if 'accel_x' in data and 'gyro_x' in data:
            imu_msg = self.create_imu_msg(data, now)
            self.imu_pub.publish(imu_msg)
        
        if 'gps_lat' in data and data.get('gps_valid', False):
            gps_msg = self.create_gps_msg(data, now)
            self.gps_pub.publish(gps_msg)
        
        if 'mag_x' in data:
            mag_msg = self.create_mag_msg(data, now)
            self.mag_pub.publish(mag_msg)
        
        if 'pressure' in data:
            pressure_msg = self.create_pressure_msg(data, now)
            self.pressure_pub.publish(pressure_msg)
    
    def create_iphone_msg(self, data: Dict, timestamp: rospy.Time) -> IPhoneSensorData:
        """Create IPhoneSensorData message"""
        msg = IPhoneSensorData()
        msg.header.stamp = timestamp
        msg.header.frame_id = "iphone"
        
        # IMU data
        msg.linear_acceleration.x = data.get('accel_x', 0.0)
        msg.linear_acceleration.y = data.get('accel_y', 0.0)
        msg.linear_acceleration.z = data.get('accel_z', 0.0)
        
        msg.angular_velocity.x = data.get('gyro_x', 0.0)
        msg.angular_velocity.y = data.get('gyro_y', 0.0)
        msg.angular_velocity.z = data.get('gyro_z', 0.0)
        
        # Magnetometer
        msg.magnetic_field.x = data.get('mag_x', 0.0)
        msg.magnetic_field.y = data.get('mag_y', 0.0)
        msg.magnetic_field.z = data.get('mag_z', 0.0)
        msg.magnetic_field_accuracy = data.get('mag_accuracy', 0.0)
        
        # GPS
        msg.latitude = data.get('gps_lat', 0.0)
        msg.longitude = data.get('gps_lon', 0.0)
        msg.altitude = data.get('gps_alt', 0.0)
        msg.gps_accuracy = data.get('gps_accuracy', 100.0)
        msg.gps_valid = data.get('gps_valid', False)
        
        # Barometer
        msg.pressure = data.get('pressure', 0.0)
        msg.barometric_altitude = data.get('altitude', 0.0)
        
        # Device motion
        if 'qw' in data:
            msg.orientation.w = data['qw']
            msg.orientation.x = data['qx']
            msg.orientation.y = data['qy']
            msg.orientation.z = data['qz']
        
        msg.gravity.x = data.get('gravity_x', 0.0)
        msg.gravity.y = data.get('gravity_y', 0.0)
        msg.gravity.z = data.get('gravity_z', 0.0)
        
        msg.user_acceleration.x = data.get('user_accel_x', 0.0)
        msg.user_acceleration.y = data.get('user_accel_y', 0.0)
        msg.user_acceleration.z = data.get('user_accel_z', 0.0)
        
        msg.motion_quality = data.get('motion_quality', 0)
        
        return msg
    
    def create_imu_msg(self, data: Dict, timestamp: rospy.Time) -> Imu:
        """Create IMU message"""
        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = "iphone_imu"
        
        # Linear acceleration
        msg.linear_acceleration.x = data['accel_x']
        msg.linear_acceleration.y = data['accel_y']
        msg.linear_acceleration.z = data['accel_z']
        
        # Angular velocity
        msg.angular_velocity.x = data['gyro_x']
        msg.angular_velocity.y = data['gyro_y']
        msg.angular_velocity.z = data['gyro_z']
        
        # Orientation (if available)
        if 'qw' in data:
            msg.orientation.w = data['qw']
            msg.orientation.x = data['qx']
            msg.orientation.y = data['qy']
            msg.orientation.z = data['qz']
            msg.orientation_covariance[0] = 0.01
            msg.orientation_covariance[4] = 0.01
            msg.orientation_covariance[8] = 0.01
        else:
            msg.orientation_covariance[0] = -1  # Orientation not available
        
        # Covariances
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1
        
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        
        return msg
    
    def create_gps_msg(self, data: Dict, timestamp: rospy.Time) -> NavSatFix:
        """Create GPS message"""
        msg = NavSatFix()
        msg.header.stamp = timestamp
        msg.header.frame_id = "gps"
        
        msg.latitude = data['gps_lat']
        msg.longitude = data['gps_lon']
        msg.altitude = data['gps_alt']
        
        # Status
        msg.status.service = 1  # GPS
        msg.status.status = 0 if data.get('gps_valid', False) else -1
        
        # Covariance
        accuracy = data.get('gps_accuracy', 10.0)
        msg.position_covariance[0] = accuracy ** 2
        msg.position_covariance[4] = accuracy ** 2
        msg.position_covariance[8] = (accuracy * 2) ** 2
        msg.position_covariance_type = 2  # Diagonal known
        
        return msg
    
    def create_mag_msg(self, data: Dict, timestamp: rospy.Time) -> MagneticField:
        """Create magnetometer message"""
        msg = MagneticField()
        msg.header.stamp = timestamp
        msg.header.frame_id = "iphone_mag"
        
        # Convert from μT to Tesla
        msg.magnetic_field.x = data['mag_x'] * 1e-6
        msg.magnetic_field.y = data['mag_y'] * 1e-6
        msg.magnetic_field.z = data['mag_z'] * 1e-6
        
        # Covariance
        accuracy = 1.0 - data.get('mag_accuracy', 0.5)
        variance = (accuracy * 50e-6) ** 2  # 50 μT base uncertainty
        msg.magnetic_field_covariance[0] = variance
        msg.magnetic_field_covariance[4] = variance
        msg.magnetic_field_covariance[8] = variance
        
        return msg
    
    def create_pressure_msg(self, data: Dict, timestamp: rospy.Time) -> FluidPressure:
        """Create pressure message"""
        msg = FluidPressure()
        msg.header.stamp = timestamp
        msg.header.frame_id = "iphone_baro"
        
        msg.fluid_pressure = data['pressure']
        msg.variance = 10.0  # Pa^2
        
        return msg
    
    def calibrate_service(self, req):
        """ROS service for calibration"""
        rospy.loginfo("Starting calibration...")
        
        if len(self.calibration_buffer) < self.calibration_samples:
            return TriggerResponse(
                success=False,
                message=f"Not enough samples: {len(self.calibration_buffer)}/{self.calibration_samples}"
            )
        
        success = self.perform_calibration()
        
        if success:
            return TriggerResponse(
                success=True,
                message="Calibration successful"
            )
        else:
            return TriggerResponse(
                success=False,
                message="Calibration failed"
            )
    
    def perform_calibration(self) -> bool:
        """Perform sensor calibration"""
        if not self.calibration_buffer:
            rospy.logwarn("No calibration data available")
            return False
        
        try:
            # Convert buffer to arrays
            accel_data = []
            gyro_data = []
            mag_data = []
            
            for sample in self.calibration_buffer:
                if 'accel_x' in sample:
                    accel_data.append([
                        sample['accel_x'],
                        sample['accel_y'],
                        sample['accel_z']
                    ])
                if 'gyro_x' in sample:
                    gyro_data.append([
                        sample['gyro_x'],
                        sample['gyro_y'],
                        sample['gyro_z']
                    ])
                if 'mag_x' in sample:
                    mag_data.append([
                        sample['mag_x'],
                        sample['mag_y'],
                        sample['mag_z']
                    ])
            
            # Calculate biases
            if accel_data:
                accel_array = np.array(accel_data)
                # Assume device is stationary and level
                self.calibration['accel_bias'][0] = np.mean(accel_array[:, 0])
                self.calibration['accel_bias'][1] = np.mean(accel_array[:, 1])
                self.calibration['accel_bias'][2] = np.mean(accel_array[:, 2]) - 9.81
            
            if gyro_data:
                gyro_array = np.array(gyro_data)
                self.calibration['gyro_bias'] = np.mean(gyro_array, axis=0)
            
            if mag_data:
                mag_array = np.array(mag_data)
                # Simple hard-iron calibration
                self.calibration['mag_bias'] = np.mean(mag_array, axis=0)
                # Could add soft-iron calibration here
            
            self.calibration['is_calibrated'] = True
            
            rospy.loginfo(f"Calibration complete:")
            rospy.loginfo(f"  Accel bias: {self.calibration['accel_bias']}")
            rospy.loginfo(f"  Gyro bias: {self.calibration['gyro_bias']}")
            rospy.loginfo(f"  Mag bias: {self.calibration['mag_bias']}")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Calibration error: {e}")
            return False
    
    def reset_service(self, req):
        """ROS service to reset calibration"""
        self.calibration['accel_bias'] = np.zeros(3)
        self.calibration['gyro_bias'] = np.zeros(3)
        self.calibration['mag_bias'] = np.zeros(3)
        self.calibration['mag_scale'] = np.ones(3)
        self.calibration['is_calibrated'] = False
        self.calibration_buffer.clear()
        
        return TriggerResponse(
            success=True,
            message="Calibration reset"
        )
    
    def auto_calibrate_callback(self, event):
        """Auto-calibrate after startup"""
        rospy.loginfo("Auto-calibration starting in 3 seconds...")
        rospy.sleep(3.0)
        
        # Wait for enough samples
        wait_time = 0
        while len(self.calibration_buffer) < self.calibration_samples and wait_time < 10:
            rospy.sleep(0.5)
            wait_time += 0.5
        
        if len(self.calibration_buffer) >= self.calibration_samples:
            self.perform_calibration()
        else:
            rospy.logwarn("Auto-calibration failed: insufficient samples")
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down iPhone sensor driver...")
        
        self.shutdown_flag.set()
        
        if self.socket:
            self.socket.close()
        
        if self.receive_thread:
            self.receive_thread.join(timeout=2)
        
        rospy.loginfo(f"Statistics: Received {self.packets_received} packets, "
                     f"dropped {self.packets_dropped}")


if __name__ == '__main__':
    try:
        driver = iPhoneSensorDriver()
        rospy.on_shutdown(driver.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

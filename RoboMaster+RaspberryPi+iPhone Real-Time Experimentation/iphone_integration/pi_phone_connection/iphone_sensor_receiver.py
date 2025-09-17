"""
iPhone Sensor Data Receiver for Raspberry Pi

Receives sensor data from iPhone via WebSocket or HTTP API, converting
the data to the format required by the 8-DOF EKF algorithm.
"""

import json
import logging
import time
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, Callable
import socket
import struct
import threading
import queue

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class iPhoneSensorData:
    """Container for iPhone sensor measurements (SI units)"""
    timestamp: float
    
    # Motion sensors (required for 8-DOF EKF)
    accel_x: float      # m/s² - Acceleration X (device frame)
    accel_y: float      # m/s² - Acceleration Y (device frame)
    accel_z: float      # m/s² - Acceleration Z (device frame)
    
    gyro_x: float       # rad/s - Angular velocity X (device frame)
    gyro_y: float       # rad/s - Angular velocity Y (device frame)
    gyro_z: float       # rad/s - Angular velocity Z (device frame)
    
    # Magnetometer (for heading reference)
    mag_x: Optional[float] = None   # μT - Magnetic field X
    mag_y: Optional[float] = None   # μT - Magnetic field Y
    mag_z: Optional[float] = None   # μT - Magnetic field Z
    
    # GPS data (if available outdoors)
    gps_lat: Optional[float] = None    # degrees
    gps_lon: Optional[float] = None    # degrees
    gps_alt: Optional[float] = None    # meters
    gps_accuracy: Optional[float] = None  # meters
    gps_speed: Optional[float] = None     # m/s
    gps_course: Optional[float] = None    # degrees (0-360, 0=N)
    
    # Barometer (altitude)
    pressure: Optional[float] = None    # Pa
    altitude: Optional[float] = None    # meters
    
    # Device orientation (from Core Motion)
    roll: Optional[float] = None     # radians
    pitch: Optional[float] = None    # radians
    yaw: Optional[float] = None      # radians
    
    # Additional iPhone-specific data
    user_accel_x: Optional[float] = None  # m/s² - User acceleration (gravity removed)
    user_accel_y: Optional[float] = None  # m/s²
    user_accel_z: Optional[float] = None  # m/s²
    
    gravity_x: Optional[float] = None     # m/s² - Gravity vector
    gravity_y: Optional[float] = None     # m/s²
    gravity_z: Optional[float] = None     # m/s²
    
    def to_ekf_format(self) -> Dict[str, Any]:
        """Convert to format expected by EKF algorithm"""
        return {
            'timestamp': self.timestamp,
            'imu': {
                'accel': [self.accel_x, self.accel_y, self.accel_z],
                'gyro': [self.gyro_x, self.gyro_y, self.gyro_z]
            },
            'magnetometer': {
                'mag': [self.mag_x, self.mag_y, self.mag_z] if self.mag_x else None
            },
            'gps': {
                'lat': self.gps_lat,
                'lon': self.gps_lon,
                'alt': self.gps_alt,
                'accuracy': self.gps_accuracy
            } if self.gps_lat else None,
            'barometer': {
                'pressure': self.pressure,
                'altitude': self.altitude
            } if self.pressure else None,
            'orientation': {
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw
            } if self.roll else None
        }


class iPhoneDataReceiver:
    """
    Handles connection and data reception from iPhone sensors
    Supports multiple connection methods: WebSocket, UDP, TCP
    """
    
    def __init__(self, connection_type: str = 'udp', port: int = 5555, host: str = ''):
        """
        Initialize iPhone data receiver
        
        Args:
            connection_type: 'udp', 'tcp', or 'websocket'
            port: Port number to listen on
        """
        self.connection_type = connection_type
        self.port = port
        self.host = host  # bind host, '' or '0.0.0.0' to listen on all
        self.is_running = False
        self.data_queue = queue.Queue(maxsize=100)
        self.latest_data = None
        self.data_callback = None
        self.connection_thread = None
        
        # Statistics
        self.packets_received = 0
        self.packets_dropped = 0
        self.last_packet_time = 0
        self.data_rate = 0
        
        # JSON message reconstruction for fragmented packets
        self.json_buffer = ""
        self.buffer_timeout = 1.0  # Clear buffer if no data for 1 second
        self.last_fragment_time = 0
        
        logger.info(f"iPhone receiver initialized: {connection_type} on port {port}")
    
    def start(self, callback: Optional[Callable] = None):
        """
        Start receiving data from iPhone
        
        Args:
            callback: Optional callback function for new data
        """
        if self.is_running:
            logger.warning("Receiver already running")
            return
        
        self.is_running = True
        self.data_callback = callback
        
        if self.connection_type == 'udp':
            self.connection_thread = threading.Thread(target=self._udp_receiver)
        elif self.connection_type == 'tcp':
            self.connection_thread = threading.Thread(target=self._tcp_receiver)
        else:
            raise ValueError(f"Unsupported connection type: {self.connection_type}")
        
        self.connection_thread.daemon = True
        self.connection_thread.start()
        logger.info("iPhone data receiver started")
    
    def stop(self):
        """Stop receiving data"""
        self.is_running = False
        if self.connection_thread:
            self.connection_thread.join(timeout=2)
        logger.info("iPhone data receiver stopped")
    
    def _udp_receiver(self):
        """UDP receiver thread"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Increase buffer size to handle larger packets
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        try:
            sock.bind((self.host or '', self.port))
        except Exception as e:
            logger.error(f"Failed to bind UDP socket on {self.host or '0.0.0.0'}:{self.port}: {e}")
            self.is_running = False
            sock.close()
            return
        sock.settimeout(1.0)  # 1 second timeout for checking stop flag
        
        logger.info(f"UDP receiver listening on {self.host or '0.0.0.0'}:{self.port}")
        
        while self.is_running:
            try:
                data, addr = sock.recvfrom(8192)  # Increased from 4096
                logger.info(f"Received {len(data)} bytes from {addr}")
                self._process_raw_data(data)
            except socket.timeout:
                continue
            except Exception as e:
                logger.error(f"UDP receiver error: {e}")
        
        sock.close()
    
    def _tcp_receiver(self):
        """TCP receiver thread"""
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server_sock.bind((self.host or '', self.port))
        except Exception as e:
            logger.error(f"Failed to bind TCP socket on {self.host or '0.0.0.0'}:{self.port}: {e}")
            self.is_running = False
            server_sock.close()
            return
        server_sock.listen(1)
        server_sock.settimeout(1.0)
        
        logger.info(f"TCP receiver listening on {self.host or '0.0.0.0'}:{self.port}")
        
        while self.is_running:
            try:
                client_sock, addr = server_sock.accept()
                logger.info(f"iPhone connected from {addr}")
                client_sock.settimeout(1.0)
                
                # Handle client connection
                while self.is_running:
                    try:
                        # Read message length (4 bytes)
                        length_data = client_sock.recv(4)
                        if not length_data:
                            break
                        
                        msg_length = struct.unpack('!I', length_data)[0]
                        
                        # Read message data
                        data = b''
                        while len(data) < msg_length:
                            chunk = client_sock.recv(min(msg_length - len(data), 4096))
                            if not chunk:
                                break
                            data += chunk
                        
                        if len(data) == msg_length:
                            self._process_raw_data(data)
                    
                    except socket.timeout:
                        continue
                    except Exception as e:
                        logger.error(f"TCP client error: {e}")
                        break
                
                client_sock.close()
                logger.info("iPhone disconnected")
            
            except socket.timeout:
                continue
            except Exception as e:
                logger.error(f"TCP server error: {e}")
        
        server_sock.close()
    
    def _process_raw_data(self, data: bytes):
        """Process raw data from iPhone with JSON fragment reassembly"""
        try:
            # Decode the incoming data
            raw_text = data.decode('utf-8', errors='ignore').strip()
            current_time = time.time()
            
            # Clear buffer if timeout exceeded (new message starting)
            if current_time - self.last_fragment_time > self.buffer_timeout:
                self.json_buffer = ""
            
            self.last_fragment_time = current_time
            
            # Add this fragment to buffer
            self.json_buffer += raw_text
            
            logger.debug(f"Fragment: {raw_text[:100]}... (buffer size: {len(self.json_buffer)})")
            
            # Check if we have a complete JSON message
            complete_messages = []
            buffer_pos = 0
            brace_count = 0
            start_pos = -1
            
            for i, char in enumerate(self.json_buffer):
                if char == '{':
                    if brace_count == 0:
                        start_pos = i
                    brace_count += 1
                elif char == '}':
                    brace_count -= 1
                    if brace_count == 0 and start_pos >= 0:
                        # Found complete JSON object
                        json_text = self.json_buffer[start_pos:i+1]
                        complete_messages.append(json_text)
                        buffer_pos = i + 1
            
            # Remove processed messages from buffer
            if buffer_pos > 0:
                self.json_buffer = self.json_buffer[buffer_pos:]
            
            # Process each complete JSON message
            for json_text in complete_messages:
                try:
                    json_data = json.loads(json_text)
                    logger.info(f"✅ Complete JSON parsed with {len(json_data)} fields")
                    
                    # Parse into iPhoneSensorData
                    sensor_data = self._parse_json_data(json_data)
                    
                    # Update statistics
                    self.packets_received += 1
                    current_time = time.time()
                    if self.last_packet_time > 0:
                        dt = current_time - self.last_packet_time
                        self.data_rate = 0.9 * self.data_rate + 0.1 * (1.0 / dt)
                    self.last_packet_time = current_time
                    
                    # Store latest data
                    self.latest_data = sensor_data
                    
                    # Add to queue (drop oldest if full)
                    if self.data_queue.full():
                        try:
                            self.data_queue.get_nowait()
                            self.packets_dropped += 1
                        except:
                            pass
                    
                    self.data_queue.put(sensor_data)
                    
                    # Call callback if provided
                    if self.data_callback:
                        self.data_callback(sensor_data)
                        
                except json.JSONDecodeError as e:
                    logger.error(f"JSON decode error: {e}")
                    logger.debug(f"Problematic JSON: {json_text[:200]}...")
            
            # Log buffer status
            if len(complete_messages) == 0:
                logger.debug(f"Incomplete JSON, buffering... (buffer: {len(self.json_buffer)} chars)")
            
        except Exception as e:
            logger.error(f"Error processing data: {e}")
            import traceback
            traceback.print_exc()
    
    def _parse_json_data(self, json_data: Dict) -> iPhoneSensorData:
        """Parse JSON (SensorLog or custom) into iPhoneSensorData object"""
        def get_num(d: Dict, key: str, default: Optional[float] = None) -> Optional[float]:
            v = d.get(key, default)
            if v is None:
                return None
            try:
                return float(v)
            except Exception:
                return default

        g = 9.81

        # Timestamp: prefer provided unix timestamp; otherwise, now
        timestamp = get_num(json_data, 'timestamp', None)
        if timestamp is None:
            # Try common SensorLog timestamps
            timestamp = get_num(json_data, 'motionTimestamp_sinceReboot', None)
            if timestamp is None:
                timestamp = get_num(json_data, 'accelerometerTimestamp_sinceReboot', None)
            if timestamp is None:
                timestamp = get_num(json_data, 'gyroTimestamp_sinceReboot', None)
        if timestamp is None:
            timestamp = time.time()

        # Accelerometer (m/s^2)
        # Preference order:
        # 1) accelerometerAcceleration* (in g) → convert to m/s^2
        # 2) motionGravity* + motionUserAcceleration* (both in g) → sum and convert
        # 3) motionGravity* (g) → convert
        # 4) motionUserAcceleration* (g) → convert
        ax = ay = az = None
        acc_ax = get_num(json_data, 'accelerometerAccelerationX', None)
        acc_ay = get_num(json_data, 'accelerometerAccelerationY', None)
        acc_az = get_num(json_data, 'accelerometerAccelerationZ', None)
        if acc_ax is not None and acc_ay is not None and acc_az is not None:
            ax, ay, az = acc_ax * g, acc_ay * g, acc_az * g
        else:
            grav_x = get_num(json_data, 'motionGravityX', None)
            grav_y = get_num(json_data, 'motionGravityY', None)
            grav_z = get_num(json_data, 'motionGravityZ', None)
            user_x = get_num(json_data, 'motionUserAccelerationX', None)
            user_y = get_num(json_data, 'motionUserAccelerationY', None)
            user_z = get_num(json_data, 'motionUserAccelerationZ', None)
            if None not in (grav_x, grav_y, grav_z, user_x, user_y, user_z):
                ax, ay, az = (grav_x + user_x) * g, (grav_y + user_y) * g, (grav_z + user_z) * g
            elif None not in (grav_x, grav_y, grav_z):
                ax, ay, az = grav_x * g, grav_y * g, grav_z * g
            elif None not in (user_x, user_y, user_z):
                ax, ay, az = user_x * g, user_y * g, user_z * g

        # Fallback to custom keys if present
        if ax is None:
            ax = get_num(json_data, 'accel_x', 0.0)
            ay = get_num(json_data, 'accel_y', 0.0)
            az = get_num(json_data, 'accel_z', 0.0)

        # Gyroscope (rad/s)
        gx = get_num(json_data, 'gyroRotationX', None)
        gy = get_num(json_data, 'gyroRotationY', None)
        gz = get_num(json_data, 'gyroRotationZ', None)
        if None in (gx, gy, gz):
            gx = get_num(json_data, 'motionRotationRateX', gx)
            gy = get_num(json_data, 'motionRotationRateY', gy)
            gz = get_num(json_data, 'motionRotationRateZ', gz)
        if None in (gx, gy, gz):
            gx = get_num(json_data, 'gyro_x', gx or 0.0)
            gy = get_num(json_data, 'gyro_y', gy or 0.0)
            gz = get_num(json_data, 'gyro_z', gz or 0.0)

        # Magnetometer (μT)
        mx = get_num(json_data, 'magnetometerX', None)
        my = get_num(json_data, 'magnetometerY', None)
        mz = get_num(json_data, 'magnetometerZ', None)
        if None in (mx, my, mz):
            mx = get_num(json_data, 'mag_x', mx)
            my = get_num(json_data, 'mag_y', my)
            mz = get_num(json_data, 'mag_z', mz)

        # GPS
        lat = get_num(json_data, 'locationLatitude', None)
        lon = get_num(json_data, 'locationLongitude', None)
        alt = get_num(json_data, 'locationAltitude', None)
        acc = get_num(json_data, 'locationHorizontalAccuracy', None)
        spd = get_num(json_data, 'locationSpeed', None)
        crs = get_num(json_data, 'locationCourse', None)
        if lat is None:
            lat = get_num(json_data, 'gps_lat', None)
            lon = get_num(json_data, 'gps_lon', None)
            alt = get_num(json_data, 'gps_alt', alt)
            acc = get_num(json_data, 'gps_accuracy', acc)
            spd = get_num(json_data, 'gps_speed', spd)
            crs = get_num(json_data, 'gps_course', crs)
        # Common generic keys used by various apps
        if lat is None:
            lat = get_num(json_data, 'lat', None)
        if lon is None:
            lon = get_num(json_data, 'lon', None)
        if lon is None:
            lon = get_num(json_data, 'lng', None)
        if alt is None:
            alt = get_num(json_data, 'alt', None)
        # Generic fallbacks
        if spd is None:
            spd = get_num(json_data, 'speed', None)
        if crs is None:
            crs = get_num(json_data, 'course', None)

        # Barometer
        pressure = get_num(json_data, 'altimeterPressure', None)
        altitude_baro = get_num(json_data, 'altimeterRelativeAltitude', None)
        if pressure is None:
            pressure = get_num(json_data, 'pressure', None)
        if altitude_baro is None:
            altitude_baro = get_num(json_data, 'altitude', None)

        # Orientation (radians, if provided)
        roll = get_num(json_data, 'motionRoll', None)
        pitch = get_num(json_data, 'motionPitch', None)
        yaw = get_num(json_data, 'motionYaw', None)
        if roll is None:
            roll = get_num(json_data, 'roll', None)
        if pitch is None:
            pitch = get_num(json_data, 'pitch', None)
        if yaw is None:
            yaw = get_num(json_data, 'yaw', None)
        
        # Handle missing heading information
        if yaw is None:
            heading = get_num(json_data, 'motionHeading', None)
            if heading is None:
                heading = get_num(json_data, 'locationTrueHeading', None)
                if heading is None:
                    heading = get_num(json_data, 'locationMagneticHeading', None)
            if heading is not None:
                # Convert heading (0-360°) to yaw (-π to π)
                import math
                yaw = math.radians(heading - 180.0)  # Convert to NED yaw convention

        # Build dataclass
        return iPhoneSensorData(
            timestamp=timestamp,
            accel_x=ax or 0.0,
            accel_y=ay or 0.0,
            accel_z=az or 0.0,
            gyro_x=gx or 0.0,
            gyro_y=gy or 0.0,
            gyro_z=gz or 0.0,
            mag_x=mx,
            mag_y=my,
            mag_z=mz,
            gps_lat=lat,
            gps_lon=lon,
            gps_alt=alt,
            gps_accuracy=acc,
            gps_speed=spd,
            gps_course=crs,
            pressure=pressure,
            altitude=altitude_baro,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            user_accel_x=get_num(json_data, 'motionUserAccelerationX', None),
            user_accel_y=get_num(json_data, 'motionUserAccelerationY', None),
            user_accel_z=get_num(json_data, 'motionUserAccelerationZ', None),
            gravity_x=get_num(json_data, 'motionGravityX', None),
            gravity_y=get_num(json_data, 'motionGravityY', None),
            gravity_z=get_num(json_data, 'motionGravityZ', None),
        )
    
    def get_latest_data(self) -> Optional[iPhoneSensorData]:
        """Get the most recent sensor data"""
        return self.latest_data
    
    def get_data_queue(self) -> queue.Queue:
        """Get the data queue for batch processing"""
        return self.data_queue
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get receiver statistics"""
        return {
            'packets_received': self.packets_received,
            'packets_dropped': self.packets_dropped,
            'data_rate': self.data_rate,
            'queue_size': self.data_queue.qsize(),
            'is_running': self.is_running
        }


class iPhoneDataProcessor:
    """
    Processes iPhone sensor data for EKF integration
    Handles coordinate transformations and calibration
    """
    
    def __init__(self):
        """Initialize data processor"""
        self.calibration = {
            'accel_bias': [0.0, 0.0, 0.0],
            'gyro_bias': [0.0, 0.0, 0.0],
            'mag_bias': [0.0, 0.0, 0.0],
            'mag_scale': [1.0, 1.0, 1.0]
        }
        
        # Coordinate transformation from iPhone to drone body frame
        # This depends on how the iPhone is mounted
        self.mounting_rotation = None  # Will be set during calibration
        
        logger.info("iPhone data processor initialized")
    
    def calibrate(self, static_data: list, duration: float = 10.0):
        """
        Calibrate sensors using static data
        
        Args:
            static_data: List of iPhoneSensorData collected while stationary
            duration: Duration of calibration in seconds
        """
        if not static_data:
            logger.error("No calibration data provided")
            return
        
        # Calculate biases from static data
        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        
        for data in static_data:
            accel_sum[0] += data.accel_x
            accel_sum[1] += data.accel_y
            accel_sum[2] += data.accel_z
            
            gyro_sum[0] += data.gyro_x
            gyro_sum[1] += data.gyro_y
            gyro_sum[2] += data.gyro_z
        
        n = len(static_data)
        
        # Gyroscope bias (should be zero when stationary)
        self.calibration['gyro_bias'] = [
            gyro_sum[0] / n,
            gyro_sum[1] / n,
            gyro_sum[2] / n
        ]
        
        # Accelerometer bias (remove gravity)
        # Assuming phone is level, gravity is mainly in Z
        gravity = 9.81
        self.calibration['accel_bias'] = [
            accel_sum[0] / n,
            accel_sum[1] / n,
            accel_sum[2] / n - gravity
        ]
        
        logger.info(f"Calibration complete: {self.calibration}")
    
    def process(self, raw_data: iPhoneSensorData) -> Dict[str, Any]:
        """
        Process raw iPhone data for EKF
        
        Args:
            raw_data: Raw sensor data from iPhone
            
        Returns:
            Processed data ready for EKF update
        """
        # Apply calibration
        accel = [
            raw_data.accel_x - self.calibration['accel_bias'][0],
            raw_data.accel_y - self.calibration['accel_bias'][1],
            raw_data.accel_z - self.calibration['accel_bias'][2]
        ]
        
        gyro = [
            raw_data.gyro_x - self.calibration['gyro_bias'][0],
            raw_data.gyro_y - self.calibration['gyro_bias'][1],
            raw_data.gyro_z - self.calibration['gyro_bias'][2]
        ]
        
        # Transform to drone body frame if needed
        # (This would involve rotation matrix multiplication)
        
        processed = {
            'timestamp': raw_data.timestamp,
            'accel': accel,
            'gyro': gyro
        }
        
        # Add optional sensors if available
        if raw_data.mag_x is not None:
            processed['mag'] = [
                (raw_data.mag_x - self.calibration['mag_bias'][0]) * self.calibration['mag_scale'][0],
                (raw_data.mag_y - self.calibration['mag_bias'][1]) * self.calibration['mag_scale'][1],
                (raw_data.mag_z - self.calibration['mag_bias'][2]) * self.calibration['mag_scale'][2]
            ]
        
        if raw_data.gps_lat is not None:
            gps_dict = {
                'lat': raw_data.gps_lat,
                'lon': raw_data.gps_lon,
                'alt': raw_data.gps_alt,
                'accuracy': raw_data.gps_accuracy,
                'speed': raw_data.gps_speed,
                'course': raw_data.gps_course
            }
            # Provide velocity vector if speed & course available
            if raw_data.gps_speed is not None and raw_data.gps_course is not None:
                import numpy as _np
                course_rad = _np.radians(raw_data.gps_course)
                gps_dict['velocity'] = [raw_data.gps_speed * _np.cos(course_rad),
                                        raw_data.gps_speed * _np.sin(course_rad)]
            processed['gps'] = gps_dict
        
        if raw_data.pressure is not None:
            processed['baro'] = {
                'pressure': raw_data.pressure,
                'altitude': raw_data.altitude
            }
        
        return processed


# Example usage and testing
if __name__ == "__main__":
    import random
    
    def data_callback(data: iPhoneSensorData):
        """Example callback function"""
        logger.info(f"Received data: accel=[{data.accel_x:.2f}, {data.accel_y:.2f}, {data.accel_z:.2f}]")
    
    # Create receiver
    receiver = iPhoneDataReceiver(connection_type='udp', port=5555)
    processor = iPhoneDataProcessor()
    
    # Start receiver
    receiver.start(callback=data_callback)
    
    try:
        # Simulate running for some time
        logger.info("Receiver running. Press Ctrl+C to stop...")
        while True:
            time.sleep(1)
            stats = receiver.get_statistics()
            logger.info(f"Stats: {stats}")
            
            # Get latest data
            latest = receiver.get_latest_data()
            if latest:
                processed = processor.process(latest)
                logger.info(f"Processed: {processed}")
    
    except KeyboardInterrupt:
        logger.info("Stopping receiver...")
        receiver.stop()

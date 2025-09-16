"""
Complete System Simulation
==========================
End-to-end simulation of iPhone-EKF-RoboMaster integration
Following RoboMaster EKF Formulary specifications

This script simulates the complete data flow:
1. iPhone sensor data generation
2. EKF processing
3. Autonomous control
4. Data logging and analysis

Author: RoboMaster EKF Integration System
Date: 2025
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'pi_phone_connection'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'robomaster_control'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'offline_analysis', 'python'))

import numpy as np
import matplotlib.pyplot as plt
import time
import json
import threading
import queue
from typing import Dict, List, Optional, Any

from iphone_sensor_receiver import iPhoneSensorData
from ekf_8dof_formulary import EKF8DOF, EKF8State
from autonomous_controller import AutonomousController, ControlMode, Waypoint, MissionPlanner


class iPhoneSimulator:
    """Simulates iPhone sensor data for testing"""
    
    def __init__(self, noise_level: float = 0.1):
        """
        Initialize iPhone simulator
        
        Args:
            noise_level: Sensor noise level (0-1)
        """
        self.noise_level = noise_level
        self.position = np.array([0.0, 0.0, 1.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        
        self.is_running = False
        self.data_queue = queue.Queue(maxsize=100)
        self.simulation_thread = None
        
    def start(self, update_rate: float = 50.0):
        """Start sensor simulation"""
        self.is_running = True
        self.simulation_thread = threading.Thread(
            target=self._simulation_loop,
            args=(update_rate,)
        )
        self.simulation_thread.daemon = True
        self.simulation_thread.start()
    
    def stop(self):
        """Stop sensor simulation"""
        self.is_running = False
        if self.simulation_thread:
            self.simulation_thread.join(timeout=2)
    
    def _simulation_loop(self, update_rate: float):
        """Main simulation loop"""
        dt = 1.0 / update_rate
        
        while self.is_running:
            # Generate sensor data
            sensor_data = self._generate_sensor_data()
            
            # Add to queue
            if not self.data_queue.full():
                self.data_queue.put(sensor_data)
            
            # Update simulated state (simple physics)
            self.position += self.velocity * dt
            self.orientation += self.angular_velocity * dt
            
            time.sleep(dt)
    
    def _generate_sensor_data(self) -> iPhoneSensorData:
        """Generate simulated sensor data"""
        # Simulated accelerometer (gravity + motion)
        gravity = np.array([0, 0, 9.81])
        
        # Rotate gravity to body frame
        roll, pitch, yaw = self.orientation
        R = self._rotation_matrix(roll, pitch, yaw)
        gravity_body = R.T @ gravity
        
        accel = gravity_body + np.random.randn(3) * self.noise_level
        
        # Simulated gyroscope
        gyro = self.angular_velocity + np.random.randn(3) * self.noise_level * 0.1
        
        # Create sensor data object
        return iPhoneSensorData(
            timestamp=time.time(),
            accel_x=accel[0],
            accel_y=accel[1],
            accel_z=accel[2],
            gyro_x=gyro[0],
            gyro_y=gyro[1],
            gyro_z=gyro[2],
            roll=self.orientation[0],
            pitch=self.orientation[1],
            yaw=self.orientation[2]
        )
    
    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Compute rotation matrix"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
    
    def set_motion(self, velocity: np.ndarray, angular_velocity: np.ndarray):
        """Set simulated motion"""
        self.velocity = velocity
        self.angular_velocity = angular_velocity
    
    def get_sensor_data(self) -> Optional[iPhoneSensorData]:
        """Get latest sensor data"""
        try:
            return self.data_queue.get(timeout=0.1)
        except queue.Empty:
            return None


class SystemSimulation:
    """Complete system simulation"""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize system simulation
        
        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        
        # Components
        self.iphone_sim = iPhoneSimulator(noise_level=0.1)
        self.ekf = EKF8DOF(self.config.get('ekf_config', {}))
        self.controller = AutonomousController(self.config.get('control_config', {}))
        
        # Data logging
        self.log_data = {
            'timestamp': [],
            'true_position': [],
            'ekf_position': [],
            'true_orientation': [],
            'ekf_orientation': [],
            'control_commands': [],
            'sensor_data': []
        }
        
        # Simulation parameters
        self.simulation_time = 0.0
        self.dt = 0.02  # 50 Hz
        self.is_running = False
        
    def run_mission(self, mission_type: str = 'square', duration: float = 30.0):
        """
        Run a simulated mission
        
        Args:
            mission_type: Type of mission ('square', 'circle', 'waypoints')
            duration: Mission duration in seconds
        """
        print(f"Starting {mission_type} mission for {duration} seconds...")
        
        # Create mission
        planner = MissionPlanner()
        
        if mission_type == 'square':
            waypoints = planner.create_square_path(
                center=(0, 0), size=3.0, altitude=1.0, points_per_side=5
            )
        elif mission_type == 'circle':
            waypoints = planner.create_circle_path(
                center=(0, 0), radius=2.0, altitude=1.0, num_points=20
            )
        else:
            # Custom waypoints
            waypoints = [
                Waypoint(x=0, y=0, z=1, speed=0.5),
                Waypoint(x=2, y=0, z=1, speed=0.5),
                Waypoint(x=2, y=2, z=1.5, speed=0.5),
                Waypoint(x=0, y=2, z=1, speed=0.5),
                Waypoint(x=0, y=0, z=1, speed=0.5)
            ]
        
        # Set mission
        self.controller.set_waypoints(waypoints)
        self.controller.set_mode(ControlMode.WAYPOINT)
        
        # Start components
        self.iphone_sim.start(update_rate=50)
        self.controller.start()
        
        # Run simulation
        self.is_running = True
        start_time = time.time()
        
        while time.time() - start_time < duration and self.is_running:
            self._simulation_step()
            time.sleep(self.dt)
            
            # Print progress
            if int(self.simulation_time) % 5 == 0 and self.simulation_time % 1 < self.dt:
                self._print_status()
        
        # Stop components
        self.iphone_sim.stop()
        self.controller.stop()
        
        print(f"\nMission complete. Logged {len(self.log_data['timestamp'])} samples.")
    
    def _simulation_step(self):
        """Single simulation step"""
        self.simulation_time += self.dt
        
        # Get sensor data
        sensor_data = self.iphone_sim.get_sensor_data()
        if not sensor_data:
            return
        
        # EKF prediction
        self.ekf.predict(self.dt)
        
        # EKF update
        accel = np.array([sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z])
        gyro = np.array([sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z])
        self.ekf.update_imu(accel, gyro)
        
        # Get EKF state
        ekf_state = self.ekf.get_state()
        
        # Update controller with EKF state
        state_dict = {
            'x': ekf_state.x,
            'y': ekf_state.y,
            'z': ekf_state.z,
            'yaw': ekf_state.yaw
        }
        self.controller.update_state(state_dict)
        
        # Get control command
        command = self.controller.get_command()
        
        # Apply control to simulator (simplified dynamics)
        if command:
            # Convert command to simulator motion
            velocity = np.array([command.vx, command.vy, command.vz])
            angular_velocity = np.array([0, 0, command.yaw_rate])
            self.iphone_sim.set_motion(velocity, angular_velocity)
        
        # Log data
        self.log_data['timestamp'].append(self.simulation_time)
        self.log_data['true_position'].append(self.iphone_sim.position.copy())
        self.log_data['ekf_position'].append(np.array([ekf_state.x, ekf_state.y, ekf_state.z]))
        self.log_data['true_orientation'].append(self.iphone_sim.orientation.copy())
        self.log_data['ekf_orientation'].append(np.array([ekf_state.roll, ekf_state.pitch, ekf_state.yaw]))
        
        if command:
            self.log_data['control_commands'].append({
                'vx': command.vx, 'vy': command.vy,
                'vz': command.vz, 'yaw_rate': command.yaw_rate
            })
        else:
            self.log_data['control_commands'].append({
                'vx': 0, 'vy': 0, 'vz': 0, 'yaw_rate': 0
            })
        
        self.log_data['sensor_data'].append({
            'accel': accel.tolist(),
            'gyro': gyro.tolist()
        })
    
    def _print_status(self):
        """Print current status"""
        ekf_state = self.ekf.get_state()
        controller_stats = self.controller.get_statistics()
        
        print(f"\n[t={self.simulation_time:.1f}s] "
              f"Position: ({ekf_state.x:.2f}, {ekf_state.y:.2f}, {ekf_state.z:.2f}) "
              f"Yaw: {np.degrees(ekf_state.yaw):.1f}° "
              f"Mode: {controller_stats['mode']} "
              f"Progress: {controller_stats['waypoint_progress']}")
    
    def analyze_results(self):
        """Analyze simulation results"""
        if not self.log_data['timestamp']:
            print("No data to analyze")
            return
        
        # Convert to numpy arrays
        timestamps = np.array(self.log_data['timestamp'])
        true_pos = np.array(self.log_data['true_position'])
        ekf_pos = np.array(self.log_data['ekf_position'])
        true_orient = np.array(self.log_data['true_orientation'])
        ekf_orient = np.array(self.log_data['ekf_orientation'])
        
        # Calculate errors
        pos_error = np.linalg.norm(ekf_pos - true_pos, axis=1)
        orient_error = np.linalg.norm(ekf_orient - true_orient, axis=1)
        
        # Statistics
        print("\n" + "=" * 60)
        print("Simulation Results")
        print("=" * 60)
        print(f"Duration: {timestamps[-1]:.1f} seconds")
        print(f"Samples: {len(timestamps)}")
        print(f"\nPosition Error:")
        print(f"  RMSE: {np.sqrt(np.mean(pos_error**2)):.4f} m")
        print(f"  Max: {np.max(pos_error):.4f} m")
        print(f"  Mean: {np.mean(pos_error):.4f} m")
        print(f"  Std: {np.std(pos_error):.4f} m")
        print(f"\nOrientation Error:")
        print(f"  RMSE: {np.degrees(np.sqrt(np.mean(orient_error**2))):.2f}°")
        print(f"  Max: {np.degrees(np.max(orient_error)):.2f}°")
        print(f"  Mean: {np.degrees(np.mean(orient_error)):.2f}°")
        
        controller_stats = self.controller.get_statistics()
        print(f"\nController Performance:")
        print(f"  Waypoints Reached: {controller_stats['waypoints_reached']}")
        print(f"  Total Distance: {controller_stats['total_distance']:.2f} m")
        print(f"  Control Updates: {controller_stats['control_updates']}")
    
    def plot_results(self, save_path: Optional[str] = None):
        """Plot simulation results"""
        if not self.log_data['timestamp']:
            print("No data to plot")
            return
        
        # Convert to numpy arrays
        timestamps = np.array(self.log_data['timestamp'])
        true_pos = np.array(self.log_data['true_position'])
        ekf_pos = np.array(self.log_data['ekf_position'])
        
        # Create figure
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Trajectory comparison
        ax = axes[0, 0]
        ax.plot(true_pos[:, 0], true_pos[:, 1], 'g--', 
               linewidth=2, alpha=0.7, label='True')
        ax.plot(ekf_pos[:, 0], ekf_pos[:, 1], 'b-', 
               linewidth=1.5, label='EKF')
        
        # Plot waypoints
        if self.controller.waypoints:
            wp_x = [w.x for w in self.controller.waypoints]
            wp_y = [w.y for w in self.controller.waypoints]
            ax.scatter(wp_x, wp_y, c='r', s=50, marker='o', 
                      zorder=5, label='Waypoints')
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Trajectory Comparison')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Position error over time
        ax = axes[0, 1]
        pos_error = np.linalg.norm(ekf_pos - true_pos, axis=1)
        ax.plot(timestamps, pos_error, 'r-', linewidth=1.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Error (m)')
        ax.set_title('Position Error')
        ax.grid(True, alpha=0.3)
        
        # Control commands
        ax = axes[1, 0]
        commands = self.log_data['control_commands']
        vx = [c['vx'] for c in commands]
        vy = [c['vy'] for c in commands]
        ax.plot(timestamps, vx, 'r-', alpha=0.7, label='vx')
        ax.plot(timestamps, vy, 'g-', alpha=0.7, label='vy')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity Command (m/s)')
        ax.set_title('Control Commands')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Altitude profile
        ax = axes[1, 1]
        ax.plot(timestamps, true_pos[:, 2], 'g--', 
               linewidth=1.5, alpha=0.7, label='True')
        ax.plot(timestamps, ekf_pos[:, 2], 'b-', 
               linewidth=1.5, label='EKF')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Altitude (m)')
        ax.set_title('Altitude Profile')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.invert_yaxis()  # NED convention
        
        plt.suptitle('System Simulation Results', fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"Plot saved to {save_path}")
        
        plt.show()


def main():
    """Main simulation runner"""
    
    print("=" * 60)
    print("iPhone-EKF-RoboMaster System Simulation")
    print("=" * 60)
    
    # Configuration
    config = {
        'ekf_config': {
            'q_position': 0.01,
            'q_velocity': 0.1,
            'q_orientation': 0.05,
            'q_angular_velocity': 0.1
        },
        'control_config': {
            'max_velocity': 0.5,
            'control_rate': 20,
            'position_kp': 1.0,
            'position_ki': 0.1,
            'position_kd': 0.2
        }
    }
    
    # Create simulation
    sim = SystemSimulation(config)
    
    # Run missions
    missions = [
        ('square', 30.0),
        ('circle', 30.0),
        ('waypoints', 20.0)
    ]
    
    for mission_type, duration in missions:
        print(f"\n{'='*60}")
        print(f"Running {mission_type} mission")
        print('='*60)
        
        # Reset simulation
        sim = SystemSimulation(config)
        
        # Run mission
        sim.run_mission(mission_type, duration)
        
        # Analyze results
        sim.analyze_results()
        
        # Plot results
        sim.plot_results()
        
        # Ask to continue
        response = input("\nContinue to next mission? (y/n): ")
        if response.lower() != 'y':
            break
    
    print("\n" + "=" * 60)
    print("Simulation Complete")
    print("=" * 60)


if __name__ == "__main__":
    main()

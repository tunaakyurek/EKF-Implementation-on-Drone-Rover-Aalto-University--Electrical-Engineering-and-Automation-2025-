"""
Real-time visualization and analysis tools for EKF data
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Arrow
import numpy as np
import time
from collections import deque
from typing import Dict, List, Any, Optional
import threading
import json

class RealTimePlotter:
    """Real-time plotting for EKF visualization"""
    
    def __init__(self, max_points: int = 1000):
        self.max_points = max_points
        
        # Data storage
        self.timestamps = deque(maxlen=max_points)
        self.positions = {'x': deque(maxlen=max_points), 'y': deque(maxlen=max_points), 'z': deque(maxlen=max_points)}
        self.orientations = {'roll': deque(maxlen=max_points), 'pitch': deque(maxlen=max_points), 'yaw': deque(maxlen=max_points)}
        self.velocities = {'vx': deque(maxlen=max_points), 'vy': deque(maxlen=max_points), 'vz': deque(maxlen=max_points)}
        self.sensor_data = {'accel': deque(maxlen=max_points), 'gyro': deque(maxlen=max_points)}
        self.chassis_data = {'x': deque(maxlen=max_points), 'y': deque(maxlen=max_points)}
        self.covariance_trace = deque(maxlen=max_points)
        
        # Setup matplotlib
        plt.style.use('dark_background')
        self.fig = None
        self.axes = {}
        self.lines = {}
        self.animation = None
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Setup plots
        self._setup_plots()
    
    def _setup_plots(self):
        """Setup matplotlib subplots"""
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4), (self.ax5, self.ax6)) = plt.subplots(3, 2, figsize=(15, 12))
        
        # 2D trajectory plot
        self.ax1.set_title('2D Trajectory (XY Plane)', color='white')
        self.ax1.set_xlabel('X Position (m)')
        self.ax1.set_ylabel('Y Position (m)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal')
        
        # Position vs time
        self.ax2.set_title('Position vs Time', color='white')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Position (m)')
        self.ax2.grid(True, alpha=0.3)
        
        # Orientation vs time
        self.ax3.set_title('Orientation vs Time', color='white')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Angle (rad)')
        self.ax3.grid(True, alpha=0.3)
        
        # Velocity vs time
        self.ax4.set_title('Velocity vs Time', color='white')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Velocity (m/s)')
        self.ax4.grid(True, alpha=0.3)
        
        # IMU data
        self.ax5.set_title('IMU Acceleration', color='white')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Acceleration (m/s²)')
        self.ax5.grid(True, alpha=0.3)
        
        # Covariance trace
        self.ax6.set_title('EKF Uncertainty (Covariance Trace)', color='white')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Trace(P)')
        self.ax6.grid(True, alpha=0.3)
        
        # Initialize lines
        self._initialize_lines()
        
        plt.tight_layout()
    
    def _initialize_lines(self):
        """Initialize plot lines"""
        # 2D trajectory
        self.lines['traj_ekf'], = self.ax1.plot([], [], 'g-', linewidth=2, label='EKF Estimate', alpha=0.8)
        self.lines['traj_chassis'], = self.ax1.plot([], [], 'r--', linewidth=1, label='Chassis Odometry', alpha=0.6)
        self.lines['current_pos'] = self.ax1.scatter([], [], s=100, c='yellow', marker='o', label='Current Position')
        self.ax1.legend()
        
        # Position
        self.lines['pos_x'], = self.ax2.plot([], [], 'r-', label='X', alpha=0.8)
        self.lines['pos_y'], = self.ax2.plot([], [], 'g-', label='Y', alpha=0.8)
        self.lines['pos_z'], = self.ax2.plot([], [], 'b-', label='Z', alpha=0.8)
        self.ax2.legend()
        
        # Orientation
        self.lines['roll'], = self.ax3.plot([], [], 'r-', label='Roll', alpha=0.8)
        self.lines['pitch'], = self.ax3.plot([], [], 'g-', label='Pitch', alpha=0.8)
        self.lines['yaw'], = self.ax3.plot([], [], 'b-', label='Yaw', alpha=0.8)
        self.ax3.legend()
        
        # Velocity
        self.lines['vel_x'], = self.ax4.plot([], [], 'r-', label='Vx', alpha=0.8)
        self.lines['vel_y'], = self.ax4.plot([], [], 'g-', label='Vy', alpha=0.8)
        self.lines['vel_z'], = self.ax4.plot([], [], 'b-', label='Vz', alpha=0.8)
        self.ax4.legend()
        
        # IMU acceleration
        self.lines['accel_x'], = self.ax5.plot([], [], 'r-', label='Ax', alpha=0.8)
        self.lines['accel_y'], = self.ax5.plot([], [], 'g-', label='Ay', alpha=0.8)
        self.lines['accel_z'], = self.ax5.plot([], [], 'b-', label='Az', alpha=0.8)
        self.ax5.legend()
        
        # Covariance
        self.lines['covariance'], = self.ax6.plot([], [], 'orange', linewidth=2, alpha=0.8)
    
    def add_ekf_data(self, timestamp: float, ekf_state: Dict[str, Any], covariance_trace: float):
        """Add EKF state data for plotting"""
        with self.data_lock:
            self.timestamps.append(timestamp)
            
            # Extract position, velocity, orientation
            position = ekf_state.get('position', [0, 0, 0])
            velocity = ekf_state.get('velocity', [0, 0, 0])
            orientation = ekf_state.get('orientation', [0, 0, 0])
            
            self.positions['x'].append(position[0])
            self.positions['y'].append(position[1])
            self.positions['z'].append(position[2])
            
            self.velocities['vx'].append(velocity[0])
            self.velocities['vy'].append(velocity[1])
            self.velocities['vz'].append(velocity[2])
            
            self.orientations['roll'].append(orientation[0])
            self.orientations['pitch'].append(orientation[1])
            self.orientations['yaw'].append(orientation[2])
            
            self.covariance_trace.append(covariance_trace)
    
    def add_sensor_data(self, timestamp: float, sensor_data: Dict[str, Any]):
        """Add sensor data for plotting"""
        with self.data_lock:
            accel = sensor_data.get('accel')
            chassis_x = sensor_data.get('chassis_x')
            chassis_y = sensor_data.get('chassis_y')
            
            if accel:
                self.sensor_data['accel'].append(accel)
            
            if chassis_x is not None and chassis_y is not None:
                self.chassis_data['x'].append(chassis_x)
                self.chassis_data['y'].append(chassis_y)
    
    def update_plots(self, frame):
        """Update all plots (called by animation)"""
        with self.data_lock:
            if len(self.timestamps) < 2:
                return list(self.lines.values())
            
            # Convert to numpy arrays for plotting
            times = np.array(self.timestamps)
            times = times - times[0]  # Relative time
            
            # 2D trajectory
            if len(self.positions['x']) > 0:
                x_pos = np.array(self.positions['x'])
                y_pos = np.array(self.positions['y'])
                
                self.lines['traj_ekf'].set_data(x_pos, y_pos)
                
                # Current position
                if len(x_pos) > 0:
                    self.lines['current_pos'].set_offsets([[x_pos[-1], y_pos[-1]]])
                
                # Chassis trajectory
                if len(self.chassis_data['x']) > 0:
                    chassis_x = np.array(self.chassis_data['x'])
                    chassis_y = np.array(self.chassis_data['y'])
                    self.lines['traj_chassis'].set_data(chassis_x, chassis_y)
                
                # Auto-scale 2D plot
                self.ax1.relim()
                self.ax1.autoscale_view()
            
            # Position vs time
            self.lines['pos_x'].set_data(times, self.positions['x'])
            self.lines['pos_y'].set_data(times, self.positions['y'])
            self.lines['pos_z'].set_data(times, self.positions['z'])
            self.ax2.relim()
            self.ax2.autoscale_view()
            
            # Orientation vs time
            self.lines['roll'].set_data(times, self.orientations['roll'])
            self.lines['pitch'].set_data(times, self.orientations['pitch'])
            self.lines['yaw'].set_data(times, self.orientations['yaw'])
            self.ax3.relim()
            self.ax3.autoscale_view()
            
            # Velocity vs time
            self.lines['vel_x'].set_data(times, self.velocities['vx'])
            self.lines['vel_y'].set_data(times, self.velocities['vy'])
            self.lines['vel_z'].set_data(times, self.velocities['vz'])
            self.ax4.relim()
            self.ax4.autoscale_view()
            
            # IMU acceleration
            if len(self.sensor_data['accel']) > 0:
                accel_data = np.array(list(self.sensor_data['accel']))
                accel_times = times[-len(accel_data):]
                
                if len(accel_data) > 0 and accel_data.shape[1] >= 3:
                    self.lines['accel_x'].set_data(accel_times, accel_data[:, 0])
                    self.lines['accel_y'].set_data(accel_times, accel_data[:, 1])
                    self.lines['accel_z'].set_data(accel_times, accel_data[:, 2])
                    self.ax5.relim()
                    self.ax5.autoscale_view()
            
            # Covariance trace
            self.lines['covariance'].set_data(times, self.covariance_trace)
            self.ax6.relim()
            self.ax6.autoscale_view()
        
        return list(self.lines.values())
    
    def start_animation(self, interval: int = 100):
        """Start real-time animation"""
        self.animation = animation.FuncAnimation(
            self.fig, self.update_plots, interval=interval, blit=False, cache_frame_data=False
        )
        plt.show()
    
    def save_plot(self, filename: str):
        """Save current plot to file"""
        self.fig.savefig(filename, dpi=300, bbox_inches='tight', facecolor='black')
        print(f"Plot saved to {filename}")

class AnalysisPlotter:
    """Static analysis and comparison plots"""
    
    @staticmethod
    def plot_trajectory_comparison(ekf_data: List[Dict], chassis_data: List[Dict], save_path: str = None):
        """Plot trajectory comparison between EKF and chassis odometry"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # Extract data
        ekf_x = [d['data']['position'][0] for d in ekf_data if 'position' in d['data']]
        ekf_y = [d['data']['position'][1] for d in ekf_data if 'position' in d['data']]
        
        chassis_x = [d['data']['chassis_x'] for d in chassis_data if d['data'].get('chassis_x') is not None]
        chassis_y = [d['data']['chassis_y'] for d in chassis_data if d['data'].get('chassis_y') is not None]
        
        # 2D trajectory comparison
        ax1.plot(ekf_x, ekf_y, 'g-', linewidth=2, label='EKF Estimate', alpha=0.8)
        ax1.plot(chassis_x, chassis_y, 'r--', linewidth=1, label='Chassis Odometry', alpha=0.6)
        ax1.scatter([ekf_x[0]], [ekf_y[0]], s=100, c='blue', marker='o', label='Start')
        ax1.scatter([ekf_x[-1]], [ekf_y[-1]], s=100, c='red', marker='s', label='End')
        
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_title('Trajectory Comparison')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')
        
        # Error analysis
        if len(ekf_x) == len(chassis_x):
            errors = np.sqrt((np.array(ekf_x) - np.array(chassis_x))**2 + 
                           (np.array(ekf_y) - np.array(chassis_y))**2)
            
            ax2.plot(errors, 'purple', linewidth=2)
            ax2.set_xlabel('Sample Number')
            ax2.set_ylabel('Position Error (m)')
            ax2.set_title(f'Position Error (RMSE: {np.sqrt(np.mean(errors**2)):.4f} m)')
            ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show()
    
    @staticmethod
    def plot_rmse_analysis(session_ids: List[str], log_directory: str = "logs"):
        """Plot RMSE analysis across multiple sessions"""
        from ..data_collection.data_logger import DataAnalyzer
        
        analyzer = DataAnalyzer(log_directory)
        
        rmse_data = {}
        for session_id in session_ids:
            rmse = analyzer.compute_rmse(session_id)
            if 'error' not in rmse:
                rmse_data[session_id] = rmse
        
        if not rmse_data:
            print("No valid RMSE data found")
            return
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        sessions = list(rmse_data.keys())
        rmse_x = [rmse_data[s]['rmse_x'] for s in sessions]
        rmse_y = [rmse_data[s]['rmse_y'] for s in sessions]
        rmse_total = [rmse_data[s]['rmse_total'] for s in sessions]
        
        # RMSE comparison
        x_pos = np.arange(len(sessions))
        width = 0.25
        
        ax1.bar(x_pos - width, rmse_x, width, label='RMSE X', alpha=0.8)
        ax1.bar(x_pos, rmse_y, width, label='RMSE Y', alpha=0.8)
        ax1.bar(x_pos + width, rmse_total, width, label='RMSE Total', alpha=0.8)
        
        ax1.set_xlabel('Session')
        ax1.set_ylabel('RMSE (m)')
        ax1.set_title('RMSE Comparison Across Sessions')
        ax1.set_xticks(x_pos)
        ax1.set_xticklabels([s[:8] for s in sessions], rotation=45)
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # RMSE distribution
        ax2.hist(rmse_total, bins=10, alpha=0.7, edgecolor='black')
        ax2.set_xlabel('RMSE Total (m)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('RMSE Distribution')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    @staticmethod
    def create_summary_dashboard(session_id: str, log_directory: str = "logs"):
        """Create comprehensive summary dashboard"""
        from ..data_collection.data_logger import DataAnalyzer
        
        analyzer = DataAnalyzer(log_directory)
        data = analyzer.load_session_data(session_id)
        
        if not data:
            print(f"No data found for session {session_id}")
            return
        
        fig = plt.figure(figsize=(20, 15))
        
        # Create grid layout
        gs = fig.add_gridspec(4, 3, height_ratios=[1, 1, 1, 0.3])
        
        # Add plots using the grid
        # This would include all the visualization components
        # created above in a single comprehensive dashboard
        
        plt.suptitle(f'EKF Analysis Dashboard - Session {session_id}', fontsize=16)
        plt.tight_layout()
        plt.show()
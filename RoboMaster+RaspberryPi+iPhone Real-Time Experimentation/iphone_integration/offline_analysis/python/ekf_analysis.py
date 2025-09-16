"""
Offline Analysis Tools for iPhone-EKF Data
==========================================
Python tools for analyzing logged EKF data
Follows RoboMaster EKF Formulary specifications

This module provides comprehensive analysis tools for:
- Trajectory visualization
- Error analysis
- Sensor data analysis
- Performance metrics
- Comparison with ground truth

Author: RoboMaster EKF Integration System
Date: 2025
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from scipy import signal, stats
from typing import Optional, Dict, List, Tuple, Any
import os
import glob
import json


class EKFDataAnalyzer:
    """
    Comprehensive analysis tool for EKF log data
    """
    
    def __init__(self, log_file: str):
        """
        Initialize analyzer with log file
        
        Args:
            log_file: Path to CSV log file
        """
        self.log_file = log_file
        self.data = None
        self.load_data()
        
        # Analysis results
        self.results = {}
        
        print(f"Loaded {len(self.data)} samples from {log_file}")
    
    def load_data(self):
        """Load data from CSV log file"""
        try:
            self.data = pd.read_csv(self.log_file)
            
            # Convert angles to degrees for visualization
            angle_cols = ['roll', 'pitch', 'yaw']
            for col in angle_cols:
                if col in self.data.columns:
                    self.data[f'{col}_deg'] = np.degrees(self.data[col])
            
            # Calculate additional metrics
            if 'x' in self.data.columns and 'y' in self.data.columns:
                # Horizontal distance traveled
                self.data['distance'] = np.sqrt(
                    self.data['x'].diff()**2 + 
                    self.data['y'].diff()**2
                ).cumsum().fillna(0)
                
                # Horizontal velocity
                dt = self.data['timestamp'].diff().fillna(0.02)
                self.data['vx'] = self.data['x'].diff() / dt
                self.data['vy'] = self.data['y'].diff() / dt
                self.data['v_horizontal'] = np.sqrt(
                    self.data['vx']**2 + self.data['vy']**2
                )
        
        except Exception as e:
            print(f"Error loading data: {e}")
            self.data = pd.DataFrame()
    
    def plot_trajectory_2d(self, save_path: Optional[str] = None):
        """
        Plot 2D trajectory (top-down view)
        
        Args:
            save_path: Optional path to save figure
        """
        if self.data.empty:
            print("No data to plot")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # X-Y trajectory
        ax = axes[0, 0]
        ax.plot(self.data['x'], self.data['y'], 'b-', linewidth=1.5, label='EKF Trajectory')
        ax.scatter(self.data['x'].iloc[0], self.data['y'].iloc[0], 
                  c='g', s=100, marker='o', label='Start')
        ax.scatter(self.data['x'].iloc[-1], self.data['y'].iloc[-1], 
                  c='r', s=100, marker='s', label='End')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('2D Trajectory (Top View)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')
        
        # X-Z trajectory (side view)
        ax = axes[0, 1]
        ax.plot(self.data['x'], self.data['z'], 'b-', linewidth=1.5)
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Z Position (m)')
        ax.set_title('Side View (X-Z)')
        ax.grid(True, alpha=0.3)
        ax.invert_yaxis()  # Invert Z for NED convention
        
        # Time series - Position
        ax = axes[1, 0]
        time = self.data['timestamp'] - self.data['timestamp'].iloc[0]
        ax.plot(time, self.data['x'], 'r-', label='X')
        ax.plot(time, self.data['y'], 'g-', label='Y')
        ax.plot(time, self.data['z'], 'b-', label='Z')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        ax.set_title('Position vs Time')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Time series - Orientation
        ax = axes[1, 1]
        if 'roll_deg' in self.data.columns:
            ax.plot(time, self.data['roll_deg'], 'r-', label='Roll')
            ax.plot(time, self.data['pitch_deg'], 'g-', label='Pitch')
            ax.plot(time, self.data['yaw_deg'], 'b-', label='Yaw')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angle (degrees)')
            ax.set_title('Orientation vs Time')
            ax.grid(True, alpha=0.3)
            ax.legend()
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"Figure saved to {save_path}")
        
        plt.show()
    
    def plot_trajectory_3d(self, save_path: Optional[str] = None):
        """
        Plot 3D trajectory
        
        Args:
            save_path: Optional path to save figure
        """
        if self.data.empty:
            print("No data to plot")
            return
        
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectory
        ax.plot(self.data['x'], self.data['y'], -self.data['z'], 
               'b-', linewidth=1.5, label='Trajectory')
        
        # Mark start and end
        ax.scatter(self.data['x'].iloc[0], self.data['y'].iloc[0], 
                  -self.data['z'].iloc[0], c='g', s=100, marker='o', label='Start')
        ax.scatter(self.data['x'].iloc[-1], self.data['y'].iloc[-1], 
                  -self.data['z'].iloc[-1], c='r', s=100, marker='s', label='End')
        
        # Color by time
        time_colors = self.data['timestamp'] - self.data['timestamp'].iloc[0]
        scatter = ax.scatter(self.data['x'][::10], self.data['y'][::10], 
                            -self.data['z'][::10], c=time_colors[::10], 
                            cmap='viridis', s=1, alpha=0.5)
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Altitude (m)')
        ax.set_title('3D Trajectory')
        ax.legend()
        
        # Add colorbar for time
        cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
        cbar.set_label('Time (s)')
        
        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"Figure saved to {save_path}")
        
        plt.show()
    
    def plot_sensor_data(self, save_path: Optional[str] = None):
        """
        Plot raw sensor data
        
        Args:
            save_path: Optional path to save figure
        """
        if self.data.empty:
            print("No data to plot")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        time = self.data['timestamp'] - self.data['timestamp'].iloc[0]
        
        # Accelerometer data
        ax = axes[0, 0]
        if 'accel_x' in self.data.columns:
            ax.plot(time, self.data['accel_x'], 'r-', alpha=0.7, label='X')
            ax.plot(time, self.data['accel_y'], 'g-', alpha=0.7, label='Y')
            ax.plot(time, self.data['accel_z'], 'b-', alpha=0.7, label='Z')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Acceleration (m/s²)')
            ax.set_title('Accelerometer Data')
            ax.grid(True, alpha=0.3)
            ax.legend()
        
        # Gyroscope data
        ax = axes[0, 1]
        if 'gyro_x' in self.data.columns:
            ax.plot(time, np.degrees(self.data['gyro_x']), 'r-', alpha=0.7, label='X')
            ax.plot(time, np.degrees(self.data['gyro_y']), 'g-', alpha=0.7, label='Y')
            ax.plot(time, np.degrees(self.data['gyro_z']), 'b-', alpha=0.7, label='Z')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angular Velocity (deg/s)')
            ax.set_title('Gyroscope Data')
            ax.grid(True, alpha=0.3)
            ax.legend()
        
        # Velocity estimates
        ax = axes[1, 0]
        if 'vz' in self.data.columns:
            ax.plot(time, self.data['vz'], 'b-', label='Vertical (vz)')
        if 'v_horizontal' in self.data.columns:
            ax.plot(time, self.data['v_horizontal'], 'r-', label='Horizontal')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Velocity Estimates')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Covariance trace
        ax = axes[1, 1]
        if 'cov_trace' in self.data.columns:
            ax.plot(time, self.data['cov_trace'], 'purple', linewidth=1.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Covariance Trace')
            ax.set_title('Filter Uncertainty')
            ax.grid(True, alpha=0.3)
            ax.set_yscale('log')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"Figure saved to {save_path}")
        
        plt.show()
    
    def analyze_performance(self) -> Dict[str, Any]:
        """
        Analyze EKF performance metrics
        
        Returns:
            Dictionary of performance metrics
        """
        metrics = {}
        
        if self.data.empty:
            return metrics
        
        # Basic statistics
        metrics['duration'] = self.data['timestamp'].iloc[-1] - self.data['timestamp'].iloc[0]
        metrics['samples'] = len(self.data)
        metrics['sample_rate'] = metrics['samples'] / metrics['duration']
        
        # Position metrics
        if 'x' in self.data.columns:
            metrics['total_distance'] = self.data['distance'].iloc[-1] if 'distance' in self.data.columns else 0
            metrics['position_range'] = {
                'x': (self.data['x'].min(), self.data['x'].max()),
                'y': (self.data['y'].min(), self.data['y'].max()),
                'z': (self.data['z'].min(), self.data['z'].max())
            }
        
        # Orientation metrics
        if 'roll_deg' in self.data.columns:
            metrics['orientation_stats'] = {
                'roll': {
                    'mean': self.data['roll_deg'].mean(),
                    'std': self.data['roll_deg'].std(),
                    'range': (self.data['roll_deg'].min(), self.data['roll_deg'].max())
                },
                'pitch': {
                    'mean': self.data['pitch_deg'].mean(),
                    'std': self.data['pitch_deg'].std(),
                    'range': (self.data['pitch_deg'].min(), self.data['pitch_deg'].max())
                },
                'yaw': {
                    'mean': self.data['yaw_deg'].mean(),
                    'std': self.data['yaw_deg'].std(),
                    'range': (self.data['yaw_deg'].min(), self.data['yaw_deg'].max())
                }
            }
        
        # Sensor noise analysis
        if 'accel_x' in self.data.columns:
            # Estimate noise during stationary periods
            velocity_threshold = 0.1  # m/s
            if 'v_horizontal' in self.data.columns:
                stationary_mask = self.data['v_horizontal'] < velocity_threshold
                if stationary_mask.any():
                    metrics['sensor_noise'] = {
                        'accel_x_std': self.data.loc[stationary_mask, 'accel_x'].std(),
                        'accel_y_std': self.data.loc[stationary_mask, 'accel_y'].std(),
                        'accel_z_std': self.data.loc[stationary_mask, 'accel_z'].std(),
                        'gyro_x_std': self.data.loc[stationary_mask, 'gyro_x'].std(),
                        'gyro_y_std': self.data.loc[stationary_mask, 'gyro_y'].std(),
                        'gyro_z_std': self.data.loc[stationary_mask, 'gyro_z'].std()
                    }
        
        # Covariance analysis
        if 'cov_trace' in self.data.columns:
            metrics['covariance'] = {
                'initial': self.data['cov_trace'].iloc[0],
                'final': self.data['cov_trace'].iloc[-1],
                'mean': self.data['cov_trace'].mean(),
                'converged': self.data['cov_trace'].iloc[-1] < self.data['cov_trace'].iloc[0]
            }
        
        self.results = metrics
        return metrics
    
    def compare_with_ground_truth(self, ground_truth_file: str) -> Dict[str, Any]:
        """
        Compare EKF estimates with ground truth data
        
        Args:
            ground_truth_file: Path to ground truth CSV file
            
        Returns:
            Dictionary of comparison metrics
        """
        try:
            gt_data = pd.read_csv(ground_truth_file)
            
            # Align timestamps
            # This is simplified - in practice you'd need proper time alignment
            min_len = min(len(self.data), len(gt_data))
            
            comparison = {}
            
            # Position RMSE
            if 'x' in self.data.columns and 'x' in gt_data.columns:
                pos_error = np.sqrt(
                    (self.data['x'][:min_len] - gt_data['x'][:min_len])**2 +
                    (self.data['y'][:min_len] - gt_data['y'][:min_len])**2 +
                    (self.data['z'][:min_len] - gt_data['z'][:min_len])**2
                )
                
                comparison['position_rmse'] = pos_error.mean()
                comparison['position_max_error'] = pos_error.max()
                comparison['position_std'] = pos_error.std()
            
            # Orientation RMSE
            if 'roll' in self.data.columns and 'roll' in gt_data.columns:
                orient_error = np.sqrt(
                    (self.data['roll'][:min_len] - gt_data['roll'][:min_len])**2 +
                    (self.data['pitch'][:min_len] - gt_data['pitch'][:min_len])**2 +
                    (self.data['yaw'][:min_len] - gt_data['yaw'][:min_len])**2
                )
                
                comparison['orientation_rmse'] = np.degrees(orient_error.mean())
                comparison['orientation_max_error'] = np.degrees(orient_error.max())
                comparison['orientation_std'] = np.degrees(orient_error.std())
            
            return comparison
        
        except Exception as e:
            print(f"Error comparing with ground truth: {e}")
            return {}
    
    def generate_report(self, output_dir: str = '.'):
        """
        Generate comprehensive analysis report
        
        Args:
            output_dir: Directory to save report
        """
        # Analyze performance
        metrics = self.analyze_performance()
        
        # Create report
        report = []
        report.append("=" * 60)
        report.append("EKF Data Analysis Report")
        report.append("=" * 60)
        report.append(f"\nLog File: {self.log_file}")
        report.append(f"Analysis Date: {pd.Timestamp.now()}")
        
        report.append("\n" + "-" * 40)
        report.append("Basic Statistics")
        report.append("-" * 40)
        report.append(f"Duration: {metrics.get('duration', 0):.2f} seconds")
        report.append(f"Samples: {metrics.get('samples', 0)}")
        report.append(f"Sample Rate: {metrics.get('sample_rate', 0):.2f} Hz")
        
        if 'total_distance' in metrics:
            report.append(f"Total Distance: {metrics['total_distance']:.2f} meters")
        
        if 'position_range' in metrics:
            report.append("\nPosition Range:")
            for axis, (min_val, max_val) in metrics['position_range'].items():
                report.append(f"  {axis.upper()}: [{min_val:.2f}, {max_val:.2f}] meters")
        
        if 'orientation_stats' in metrics:
            report.append("\nOrientation Statistics:")
            for angle, stats in metrics['orientation_stats'].items():
                report.append(f"  {angle.capitalize()}:")
                report.append(f"    Mean: {stats['mean']:.2f}°")
                report.append(f"    Std: {stats['std']:.2f}°")
                report.append(f"    Range: [{stats['range'][0]:.2f}°, {stats['range'][1]:.2f}°]")
        
        if 'sensor_noise' in metrics:
            report.append("\nSensor Noise (Stationary):")
            for sensor, noise in metrics['sensor_noise'].items():
                report.append(f"  {sensor}: {noise:.6f}")
        
        if 'covariance' in metrics:
            report.append("\nCovariance Analysis:")
            report.append(f"  Initial: {metrics['covariance']['initial']:.6f}")
            report.append(f"  Final: {metrics['covariance']['final']:.6f}")
            report.append(f"  Mean: {metrics['covariance']['mean']:.6f}")
            report.append(f"  Converged: {metrics['covariance']['converged']}")
        
        # Save report
        report_text = '\n'.join(report)
        report_file = os.path.join(output_dir, 'analysis_report.txt')
        
        with open(report_file, 'w') as f:
            f.write(report_text)
        
        print(report_text)
        print(f"\nReport saved to {report_file}")
        
        # Save metrics as JSON
        metrics_file = os.path.join(output_dir, 'analysis_metrics.json')
        with open(metrics_file, 'w') as f:
            json.dump(metrics, f, indent=2, default=str)
        
        print(f"Metrics saved to {metrics_file}")


def batch_analysis(log_directory: str, output_dir: str = 'analysis_results'):
    """
    Perform batch analysis on multiple log files
    
    Args:
        log_directory: Directory containing log files
        output_dir: Directory to save analysis results
    """
    # Find all log files
    log_files = glob.glob(os.path.join(log_directory, 'iphone_ekf_log_*.csv'))
    
    if not log_files:
        print(f"No log files found in {log_directory}")
        return
    
    print(f"Found {len(log_files)} log files")
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Analyze each file
    all_metrics = []
    
    for i, log_file in enumerate(log_files):
        print(f"\nAnalyzing {i+1}/{len(log_files)}: {os.path.basename(log_file)}")
        
        # Create analyzer
        analyzer = EKFDataAnalyzer(log_file)
        
        # Create output subdirectory
        log_name = os.path.splitext(os.path.basename(log_file))[0]
        log_output_dir = os.path.join(output_dir, log_name)
        os.makedirs(log_output_dir, exist_ok=True)
        
        # Generate plots
        analyzer.plot_trajectory_2d(os.path.join(log_output_dir, 'trajectory_2d.png'))
        analyzer.plot_trajectory_3d(os.path.join(log_output_dir, 'trajectory_3d.png'))
        analyzer.plot_sensor_data(os.path.join(log_output_dir, 'sensor_data.png'))
        
        # Generate report
        analyzer.generate_report(log_output_dir)
        
        # Collect metrics
        metrics = analyzer.analyze_performance()
        metrics['log_file'] = log_file
        all_metrics.append(metrics)
    
    # Save summary
    summary_df = pd.DataFrame(all_metrics)
    summary_file = os.path.join(output_dir, 'batch_summary.csv')
    summary_df.to_csv(summary_file, index=False)
    
    print(f"\nBatch analysis complete. Summary saved to {summary_file}")


# Example usage
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='EKF Data Analysis Tool')
    parser.add_argument('log_file', type=str, help='Path to log file or directory for batch analysis')
    parser.add_argument('--batch', action='store_true', help='Perform batch analysis on directory')
    parser.add_argument('--output', type=str, default='analysis_results', help='Output directory')
    parser.add_argument('--ground-truth', type=str, help='Ground truth file for comparison')
    
    args = parser.parse_args()
    
    if args.batch:
        batch_analysis(args.log_file, args.output)
    else:
        # Single file analysis
        analyzer = EKFDataAnalyzer(args.log_file)
        
        # Generate plots
        analyzer.plot_trajectory_2d()
        analyzer.plot_trajectory_3d()
        analyzer.plot_sensor_data()
        
        # Analyze performance
        metrics = analyzer.analyze_performance()
        print("\nPerformance Metrics:")
        print(json.dumps(metrics, indent=2, default=str))
        
        # Compare with ground truth if provided
        if args.ground_truth:
            comparison = analyzer.compare_with_ground_truth(args.ground_truth)
            print("\nGround Truth Comparison:")
            print(json.dumps(comparison, indent=2))
        
        # Generate report
        analyzer.generate_report(args.output)

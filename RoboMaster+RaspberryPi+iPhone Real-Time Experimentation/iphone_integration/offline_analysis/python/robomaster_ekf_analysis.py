#!/usr/bin/env python3
"""
RoboMaster EKF Offline Analysis
===============================
Analyze RoboMaster formulary EKF log files with state vector:
[x, y, theta, vx, vy, bias_accel_x, bias_accel_y, bias_angular_velocity]

Usage:
    python robomaster_ekf_analysis.py --file path/to/robomaster_ekf_log.csv
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from datetime import datetime
import sys

def load_robomaster_data(csv_file):
    """Load RoboMaster EKF CSV log file"""
    print(f"Loading RoboMaster EKF data from: {csv_file}")
    
    # Expected columns for RoboMaster formulary
    expected_cols = [
        'timestamp', 'x', 'y', 'theta', 'vx', 'vy', 
        'bias_accel_x', 'bias_accel_y', 'bias_angular_velocity',
        'accel_x', 'accel_y', 'accel_z', 
        'gyro_x', 'gyro_y', 'gyro_z', 'cov_trace'
    ]
    
    # Optional raw sensor columns that may appear in newer logs
    optional_cols = [
        # GPS in local ENU (preferred if present)
        'gps_x', 'gps_y', 'gps_vx', 'gps_vy', 'gps_speed',
        # GPS in geodetic
        'lat', 'lon', 'altitude',
        # Magnetometer
        'mag_x', 'mag_y', 'mag_z',
        # Barometer / altitude
        'baro'
    ]
    
    try:
        df = pd.read_csv(csv_file)
        print(f"Loaded {len(df)} samples")
        print(f"Columns: {list(df.columns)}")
        
        # Validate columns
        missing_cols = [col for col in expected_cols if col not in df.columns]
        if missing_cols:
            print(f"Missing columns: {missing_cols}")
        
        # Convert timestamp to relative time
        df['time_rel'] = df['timestamp'] - df['timestamp'].iloc[0]
        
        # Convert theta from radians to degrees for plotting
        df['theta_deg'] = np.degrees(df['theta'])

        # If GPS velocity components exist, compute speed if missing
        if {'gps_vx', 'gps_vy'}.issubset(df.columns) and 'gps_speed' not in df.columns:
            df['gps_speed'] = np.sqrt(df['gps_vx']**2 + df['gps_vy']**2)

        # If geodetic GPS (lat/lon) exists but local 'gps_x','gps_y' don't, create approximate local meters
        if {'lat', 'lon'}.issubset(df.columns) and not {'gps_x', 'gps_y'}.issubset(df.columns):
            try:
                lat0 = float(df['lat'].iloc[0])
                lon0 = float(df['lon'].iloc[0])
                lat_to_m = 111320.0
                lon_to_m = 111320.0 * np.cos(np.radians(lat0))
                df['gps_x'] = (df['lat'] - lat0) * lat_to_m
                df['gps_y'] = (df['lon'] - lon0) * lon_to_m
            except Exception:
                pass
        
        print(f"Data duration: {df['time_rel'].iloc[-1]:.1f} seconds")
        print(f"Sampling rate: {len(df) / df['time_rel'].iloc[-1]:.1f} Hz")
        
        return df
        
    except Exception as e:
        print(f"Error loading file: {e}")
        return None

def analyze_robomaster_trajectory(df):
    """Analyze RoboMaster trajectory data"""
    print("\nRoboMaster Trajectory Analysis")
    print("=" * 50)
    
    # Position statistics
    pos_stats = {
        'x_range': df['x'].max() - df['x'].min(),
        'y_range': df['y'].max() - df['y'].min(),
        'total_distance': np.sum(np.sqrt(np.diff(df['x'])**2 + np.diff(df['y'])**2)),
        'max_distance_from_origin': np.sqrt(df['x']**2 + df['y']**2).max()
    }
    
    # Velocity statistics
    vel_stats = {
        'max_vx': df['vx'].max(),
        'max_vy': df['vy'].max(),
        'max_speed': np.sqrt(df['vx']**2 + df['vy']**2).max(),
        'avg_speed': np.sqrt(df['vx']**2 + df['vy']**2).mean()
    }
    
    # Orientation statistics
    theta_stats = {
        'theta_range_deg': df['theta_deg'].max() - df['theta_deg'].min(),
        'max_angular_change': np.abs(np.diff(df['theta_deg'])).max()
    }
    
    # Bias statistics
    bias_stats = {
        'final_bias_accel_x': df['bias_accel_x'].iloc[-1],
        'final_bias_accel_y': df['bias_accel_y'].iloc[-1],
        'final_bias_angular': df['bias_angular_velocity'].iloc[-1],
        'bias_convergence_x': np.abs(df['bias_accel_x'].iloc[-1] - df['bias_accel_x'].iloc[len(df)//2]),
        'bias_convergence_y': np.abs(df['bias_accel_y'].iloc[-1] - df['bias_accel_y'].iloc[len(df)//2])
    }
    
    print("Position Analysis:")
    print(f"   X range: {pos_stats['x_range']:.3f} m")
    print(f"   Y range: {pos_stats['y_range']:.3f} m")
    print(f"   Total distance: {pos_stats['total_distance']:.3f} m")
    print(f"   Max distance from origin: {pos_stats['max_distance_from_origin']:.3f} m")
    
    print("\nVelocity Analysis:")
    print(f"   Max vx: {vel_stats['max_vx']:.3f} m/s")
    print(f"   Max vy: {vel_stats['max_vy']:.3f} m/s")
    print(f"   Max speed: {vel_stats['max_speed']:.3f} m/s")
    print(f"   Average speed: {vel_stats['avg_speed']:.3f} m/s")
    
    print("\nOrientation Analysis:")
    print(f"   Theta range: {theta_stats['theta_range_deg']:.1f}°")
    print(f"   Max angular change: {theta_stats['max_angular_change']:.1f}°")
    
    print("\nSensor Bias Analysis:")
    print(f"   Final accel bias X: {bias_stats['final_bias_accel_x']:.4f} m/s²")
    print(f"   Final accel bias Y: {bias_stats['final_bias_accel_y']:.4f} m/s²")
    print(f"   Final angular bias: {bias_stats['final_bias_angular']:.4f} rad/s")
    print(f"   Bias convergence X: {bias_stats['bias_convergence_x']:.4f} m/s²")
    print(f"   Bias convergence Y: {bias_stats['bias_convergence_y']:.4f} m/s²")
    
    return pos_stats, vel_stats, theta_stats, bias_stats

def plot_robomaster_analysis(df, output_dir):
    """Create comprehensive RoboMaster analysis plots"""
    print(f"\nCreating RoboMaster analysis plots...")
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))
    
    # 1. 2D Trajectory
    ax1 = plt.subplot(3, 3, 1)
    plt.plot(df['x'], df['y'], 'b-', linewidth=2, alpha=0.7)
    plt.plot(df['x'].iloc[0], df['y'].iloc[0], 'go', markersize=10, label='Start')
    plt.plot(df['x'].iloc[-1], df['y'].iloc[-1], 'ro', markersize=10, label='End')
    # Overlay raw GPS trajectory if available (either local gps_x/gps_y or derived from lat/lon)
    if {'gps_x', 'gps_y'}.issubset(df.columns):
        plt.scatter(df['gps_x'], df['gps_y'], s=8, c='k', alpha=0.4, label='GPS')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('RoboMaster 2D Trajectory')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.axis('equal')
    
    # 2. Position vs Time
    ax2 = plt.subplot(3, 3, 2)
    plt.plot(df['time_rel'], df['x'], 'r-', label='X', linewidth=2)
    plt.plot(df['time_rel'], df['y'], 'g-', label='Y', linewidth=2)
    # Overlay GPS position if available
    if {'gps_x', 'gps_y'}.issubset(df.columns):
        plt.plot(df['time_rel'], df['gps_x'], 'r:', label='GPS X', linewidth=1.5, alpha=0.8)
        plt.plot(df['time_rel'], df['gps_y'], 'g:', label='GPS Y', linewidth=1.5, alpha=0.8)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Position vs Time')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 3. Orientation vs Time
    ax3 = plt.subplot(3, 3, 3)
    plt.plot(df['time_rel'], df['theta_deg'], 'm-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation (degrees)')
    plt.title('Orientation vs Time')
    plt.grid(True, alpha=0.3)
    
    # 4. Velocity vs Time
    ax4 = plt.subplot(3, 3, 4)
    plt.plot(df['time_rel'], df['vx'], 'r-', label='vx', linewidth=2)
    plt.plot(df['time_rel'], df['vy'], 'g-', label='vy', linewidth=2)
    speed = np.sqrt(df['vx']**2 + df['vy']**2)
    plt.plot(df['time_rel'], speed, 'b--', label='speed', linewidth=2)
    # Overlay GPS velocity if available
    if {'gps_vx', 'gps_vy'}.issubset(df.columns):
        plt.plot(df['time_rel'], df['gps_vx'], 'r-.', label='GPS vx', linewidth=1.5, alpha=0.9)
        plt.plot(df['time_rel'], df['gps_vy'], 'g-.', label='GPS vy', linewidth=1.5, alpha=0.9)
        if 'gps_speed' in df.columns:
            plt.plot(df['time_rel'], df['gps_speed'], 'b:', label='GPS speed', linewidth=1.5, alpha=0.9)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity vs Time')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 5. Accelerometer Biases
    ax5 = plt.subplot(3, 3, 5)
    plt.plot(df['time_rel'], df['bias_accel_x'], 'r-', label='Bias X', linewidth=2)
    plt.plot(df['time_rel'], df['bias_accel_y'], 'g-', label='Bias Y', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Accel Bias (m/s²)')
    plt.title('Accelerometer Bias Estimation')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 6. Angular Velocity Bias
    ax6 = plt.subplot(3, 3, 6)
    plt.plot(df['time_rel'], df['bias_angular_velocity'], 'c-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Bias (rad/s)')
    plt.title('Gyroscope Bias Estimation')
    plt.grid(True, alpha=0.3)
    
    # 7. Sensor Data - Accelerometer
    ax7 = plt.subplot(3, 3, 7)
    plt.plot(df['time_rel'], df['accel_x'], 'r-', alpha=0.7, label='ax')
    plt.plot(df['time_rel'], df['accel_y'], 'g-', alpha=0.7, label='ay')
    plt.plot(df['time_rel'], df['accel_z'], 'b-', alpha=0.7, label='az')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.title('Raw Accelerometer Data')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 8. Sensor Data - Gyroscope
    ax8 = plt.subplot(3, 3, 8)
    plt.plot(df['time_rel'], df['gyro_x'], 'r-', alpha=0.7, label='ωx')
    plt.plot(df['time_rel'], df['gyro_y'], 'g-', alpha=0.7, label='ωy')
    plt.plot(df['time_rel'], df['gyro_z'], 'b-', alpha=0.7, label='ωz')
    # If magnetometer data exists, overlay with a secondary axis for clarity
    if {'mag_x', 'mag_y', 'mag_z'}.issubset(df.columns):
        ax8b = ax8.twinx()
        ax8b.plot(df['time_rel'], df['mag_x'], color='tab:orange', alpha=0.4, label='mx')
        ax8b.plot(df['time_rel'], df['mag_y'], color='tab:purple', alpha=0.4, label='my')
        ax8b.plot(df['time_rel'], df['mag_z'], color='tab:brown', alpha=0.4, label='mz')
        ax8b.set_ylabel('Magnetic Field (a.u.)')
        # Build a combined legend
        lines1, labels1 = ax8.get_legend_handles_labels()
        lines2, labels2 = ax8b.get_legend_handles_labels()
        ax8.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Raw Gyroscope Data')
    plt.grid(True, alpha=0.3)
    if not {'mag_x', 'mag_y', 'mag_z'}.issubset(df.columns):
        plt.legend()
    
    # 9. Covariance Trace (overlay barometer altitude if available)
    ax9 = plt.subplot(3, 3, 9)
    plt.semilogy(df['time_rel'], df['cov_trace'], 'k-', linewidth=2)
    if 'baro' in df.columns:
        ax9b = ax9.twinx()
        ax9b.plot(df['time_rel'], df['baro'], 'c-', alpha=0.5, label='Baro Alt')
        ax9b.set_ylabel('Altitude (baro, a.u.)')
        lines1, labels1 = ax9.get_legend_handles_labels()
        lines2, labels2 = ax9b.get_legend_handles_labels()
        ax9.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    plt.xlabel('Time (s)')
    plt.ylabel('Covariance Trace')
    plt.title('EKF Uncertainty (Log Scale)')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plots
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_file = os.path.join(output_dir, f'robomaster_analysis_{timestamp}.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Plots saved to: {plot_file}")
    
    plt.show()

def plot_raw_sensor_overview(df, output_dir):
    """Create a dedicated figure for raw sensor data (GPS/IMU/Mag/Baro)"""
    print("\nCreating Raw Sensor overview plots...")

    # Ensure relative time exists
    if 'time_rel' not in df.columns and 'timestamp' in df.columns and len(df) > 0:
        df['time_rel'] = df['timestamp'] - df['timestamp'].iloc[0]

    # If GPS local coords missing, derive from any lat/lon columns
    have_local = {'gps_x', 'gps_y'}.issubset(df.columns)
    have_ll = {'lat','lon'}.issubset(df.columns) or {'gps_lat','gps_lon'}.issubset(df.columns)
    if not have_local and have_ll:
        try:
            if {'lat','lon'}.issubset(df.columns):
                lat_series = df['lat']
                lon_series = df['lon']
            else:
                lat_series = df['gps_lat']
                lon_series = df['gps_lon']
            lat0 = float(lat_series.iloc[0])
            lon0 = float(lon_series.iloc[0])
            lat_to_m = 111320.0
            lon_to_m = 111320.0 * np.cos(np.radians(lat0))
            df['gps_x'] = (lat_series - lat0) * lat_to_m
            df['gps_y'] = (lon_series - lon0) * lon_to_m
        except Exception:
            pass

    fig = plt.figure(figsize=(16, 12))

    # 1. GPS 2D trajectory (if available)
    ax1 = plt.subplot(3, 3, 1)
    if {'gps_x', 'gps_y'}.issubset(df.columns):
        plt.plot(df['gps_x'], df['gps_y'], 'k.', markersize=2, alpha=0.6, label='GPS')
        plt.plot(df['gps_x'].iloc[0], df['gps_y'].iloc[0], 'go', label='Start')
        plt.plot(df['gps_x'].iloc[-1], df['gps_y'].iloc[-1], 'ro', label='End')
        plt.axis('equal')
        plt.legend()
    else:
        plt.text(0.5, 0.5, 'No GPS (gps_x/gps_y)', ha='center', va='center', transform=ax1.transAxes)
    plt.xlabel('GPS X (m)')
    plt.ylabel('GPS Y (m)')
    plt.title('Raw GPS 2D Trajectory')
    plt.grid(True, alpha=0.3)

    # 2. GPS position vs time
    ax2 = plt.subplot(3, 3, 2)
    if {'gps_x', 'gps_y'}.issubset(df.columns):
        plt.plot(df['time_rel'], df['gps_x'], 'r:', label='GPS X')
        plt.plot(df['time_rel'], df['gps_y'], 'g:', label='GPS Y')
        plt.legend()
    else:
        plt.text(0.5, 0.5, 'No GPS position columns', ha='center', va='center', transform=ax2.transAxes)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('GPS Position vs Time')
    plt.grid(True, alpha=0.3)

    # 3. GPS velocity/speed vs time
    ax3 = plt.subplot(3, 3, 3)
    any_gps_vel = False
    if {'gps_vx', 'gps_vy'}.issubset(df.columns):
        plt.plot(df['time_rel'], df['gps_vx'], 'r-.', label='GPS vx')
        plt.plot(df['time_rel'], df['gps_vy'], 'g-.', label='GPS vy')
        any_gps_vel = True
    if 'gps_speed' in df.columns:
        plt.plot(df['time_rel'], df['gps_speed'], 'b:', label='GPS speed')
        any_gps_vel = True
    if not any_gps_vel:
        plt.text(0.5, 0.5, 'No GPS velocity/speed', ha='center', va='center', transform=ax3.transAxes)
    else:
        plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('GPS Velocity/Speed vs Time')
    plt.grid(True, alpha=0.3)

    # 4. Accelerometer
    ax4 = plt.subplot(3, 3, 4)
    if {'accel_x', 'accel_y', 'accel_z'}.issubset(df.columns):
        plt.plot(df['time_rel'], df['accel_x'], 'r-', alpha=0.7, label='ax')
        plt.plot(df['time_rel'], df['accel_y'], 'g-', alpha=0.7, label='ay')
        plt.plot(df['time_rel'], df['accel_z'], 'b-', alpha=0.7, label='az')
        plt.legend()
    else:
        plt.text(0.5, 0.5, 'No accelerometer data', ha='center', va='center', transform=ax4.transAxes)
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.title('Raw Accelerometer Data')
    plt.grid(True, alpha=0.3)

    # 5. Gyroscope
    ax5 = plt.subplot(3, 3, 5)
    if {'gyro_x', 'gyro_y', 'gyro_z'}.issubset(df.columns):
        plt.plot(df['time_rel'], df['gyro_x'], 'r-', alpha=0.7, label='ωx')
        plt.plot(df['time_rel'], df['gyro_y'], 'g-', alpha=0.7, label='ωy')
        plt.plot(df['time_rel'], df['gyro_z'], 'b-', alpha=0.7, label='ωz')
        plt.legend()
    else:
        plt.text(0.5, 0.5, 'No gyroscope data', ha='center', va='center', transform=ax5.transAxes)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Raw Gyroscope Data')
    plt.grid(True, alpha=0.3)

    # 6. Magnetometer
    ax6 = plt.subplot(3, 3, 6)
    if {'mag_x', 'mag_y', 'mag_z'}.issubset(df.columns):
        plt.plot(df['time_rel'], df['mag_x'], color='tab:orange', alpha=0.8, label='mx')
        plt.plot(df['time_rel'], df['mag_y'], color='tab:purple', alpha=0.8, label='my')
        plt.plot(df['time_rel'], df['mag_z'], color='tab:brown', alpha=0.8, label='mz')
        plt.legend()
    else:
        plt.text(0.5, 0.5, 'No magnetometer data', ha='center', va='center', transform=ax6.transAxes)
    plt.xlabel('Time (s)')
    plt.ylabel('Magnetic Field (a.u.)')
    plt.title('Raw Magnetometer Data')
    plt.grid(True, alpha=0.3)

    # 7. Barometer / altitude
    ax7 = plt.subplot(3, 3, 7)
    if 'baro' in df.columns:
        plt.plot(df['time_rel'], df['baro'], 'c-', alpha=0.9, label='baro')
        plt.legend()
    elif 'altitude' in df.columns:
        plt.plot(df['time_rel'], df['altitude'], 'c-', alpha=0.9, label='altitude')
        plt.legend()
    else:
        plt.text(0.5, 0.5, 'No barometer/altitude data', ha='center', va='center', transform=ax7.transAxes)
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (a.u.)')
    plt.title('Barometer / Altitude')
    plt.grid(True, alpha=0.3)

    plt.tight_layout()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_file = os.path.join(output_dir, f'robomaster_raw_sensors_{timestamp}.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Raw sensor plots saved to: {plot_file}")

    plt.show()

def generate_report(df, pos_stats, vel_stats, theta_stats, bias_stats, output_dir):
    """Generate analysis report"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = os.path.join(output_dir, f'robomaster_analysis_report_{timestamp}.txt')
    
    with open(report_file, 'w') as f:
        f.write("RoboMaster EKF Analysis Report\n")
        f.write("=" * 50 + "\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Data file: {len(df)} samples\n")
        f.write(f"Duration: {df['time_rel'].iloc[-1]:.1f} seconds\n")
        f.write(f"Sample rate: {len(df) / df['time_rel'].iloc[-1]:.1f} Hz\n\n")
        
        f.write("State Vector: [x, y, theta, vx, vy, bias_accel_x, bias_accel_y, bias_angular_velocity]\n\n")
        
        f.write("Position Analysis:\n")
        for key, value in pos_stats.items():
            f.write(f"  {key}: {value:.4f}\n")
        
        f.write("\nVelocity Analysis:\n")
        for key, value in vel_stats.items():
            f.write(f"  {key}: {value:.4f}\n")
        
        f.write("\nOrientation Analysis:\n")
        for key, value in theta_stats.items():
            f.write(f"  {key}: {value:.4f}\n")
        
        f.write("\nBias Analysis:\n")
        for key, value in bias_stats.items():
            f.write(f"  {key}: {value:.6f}\n")
    
    print(f"Report saved to: {report_file}")

def detect_and_filter_phases(df, pre_delay_seconds=5.0, calibration_duration=5.0):
    """
    Detect and filter data phases to exclude pre-delay and show only calibration + EKF phases
    
    Args:
        df: DataFrame with RoboMaster data
        pre_delay_seconds: Duration of pre-delay phase (default 5.0s)
        calibration_duration: Duration of calibration phase (default 5.0s)
    
    Returns:
        filtered_df: DataFrame with only calibration and EKF phases
        phase_info: Dictionary with phase timing information
    """
    print(f"\nDetecting data phases (pre-delay: {pre_delay_seconds}s, calibration: {calibration_duration}s)")
    
    # Detect phases based on EKF state values
    # Pre-delay: EKF states are all zeros (not yet calibrated)
    # Calibration: EKF states still zeros but collecting data
    # Running: EKF states have non-zero values (calibration complete)
    
    # Find when EKF starts producing non-zero states (calibration complete)
    ekf_running_mask = (df['x'] != 0.0) | (df['y'] != 0.0) | (df['vx'] != 0.0) | (df['vy'] != 0.0)
    
    if ekf_running_mask.any():
        # Find first non-zero EKF state
        first_ekf_idx = ekf_running_mask.idxmax()
        first_ekf_time = df.loc[first_ekf_idx, 'time_rel']
        
        # Calibration phase starts after pre-delay
        calibration_start_time = pre_delay_seconds
        calibration_end_time = calibration_start_time + calibration_duration
        
        # Filter data: exclude pre-delay, include calibration + EKF phases
        phase_mask = df['time_rel'] >= calibration_start_time
        
        filtered_df = df[phase_mask].copy()
        
        # Adjust time to start from calibration phase
        filtered_df['time_rel'] = filtered_df['time_rel'] - calibration_start_time
        
        phase_info = {
            'pre_delay_duration': pre_delay_seconds,
            'calibration_start': calibration_start_time,
            'calibration_end': calibration_end_time,
            'ekf_start': first_ekf_time,
            'total_filtered_duration': filtered_df['time_rel'].iloc[-1],
            'samples_excluded': len(df) - len(filtered_df),
            'samples_included': len(filtered_df)
        }
        
        print(f"Phase detection complete:")
        print(f"   Pre-delay: 0.0s - {pre_delay_seconds:.1f}s (excluded)")
        print(f"   Calibration: {calibration_start_time:.1f}s - {calibration_end_time:.1f}s (included)")
        print(f"   EKF running: {first_ekf_time:.1f}s - end (included)")
        print(f"   Data filtered: {phase_info['samples_excluded']} samples excluded, {phase_info['samples_included']} samples included")
        
        return filtered_df, phase_info
        
    else:
        print("No EKF state changes detected - using full dataset")
        return df, {'pre_delay_duration': 0, 'calibration_start': 0, 'calibration_end': 0, 'ekf_start': 0}

def main():
    parser = argparse.ArgumentParser(description='Analyze RoboMaster EKF log files')
    parser.add_argument('--file', required=True, help='Path to RoboMaster EKF CSV log file')
    parser.add_argument('--output', default='analysis_results', help='Output directory for plots and reports')
    parser.add_argument('--raw-file', default=None, help='Optional path to raw sensor CSV file (robomaster_raw_log_*.csv)')
    parser.add_argument('--pre-delay', type=float, default=5.0, help='Pre-delay duration in seconds (default: 5.0)')
    parser.add_argument('--calibration-duration', type=float, default=5.0, help='Calibration duration in seconds (default: 5.0)')
    
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output, exist_ok=True)
    
    # Load data
    df = load_robomaster_data(args.file)
    if df is None:
        return
    
    # Detect and filter phases
    df_filtered, phase_info = detect_and_filter_phases(df, args.pre_delay, args.calibration_duration)
    
    # Analyze trajectory using filtered data
    pos_stats, vel_stats, theta_stats, bias_stats = analyze_robomaster_trajectory(df_filtered)
    
    # Create plots using filtered data
    plot_robomaster_analysis(df_filtered, args.output)
    # Create dedicated raw sensor figure
    if args.raw_file and os.path.exists(args.raw_file):
        try:
            df_raw = pd.read_csv(args.raw_file)
            # Build time_rel if possible
            if 'timestamp' in df_raw.columns and len(df_raw) > 0:
                df_raw['time_rel'] = df_raw['timestamp'] - df_raw['timestamp'].iloc[0]
            # Map geodetic to local if EKF file had reference
            if {'lat','lon'}.issubset(df_raw.columns) and not {'gps_x','gps_y'}.issubset(df_raw.columns):
                # try to derive using EKF file's origin if available
                if {'gps_x','gps_y'}.issubset(df.columns):
                    # Already in local in EKF; skip
                    pass
            plot_raw_sensor_overview(df_raw, args.output)
        except Exception as e:
            print(f"Failed to read raw file: {e}. Falling back to EKF CSV for raw plots.")
            plot_raw_sensor_overview(df, args.output)
    else:
        plot_raw_sensor_overview(df, args.output)
    
    # Generate report using filtered data and phase info
    generate_report(df_filtered, pos_stats, vel_stats, theta_stats, bias_stats, args.output)
    
    print(f"\nRoboMaster EKF analysis complete!")
    print(f"Results saved in: {args.output}/")

if __name__ == "__main__":
    main()

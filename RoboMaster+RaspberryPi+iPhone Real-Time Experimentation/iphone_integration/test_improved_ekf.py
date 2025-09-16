#!/usr/bin/env python3
"""
Test script for improved RoboMaster EKF with yaw observability fixes
===============================================================
This script demonstrates the improved EKF that solves the yaw drift issue
by adding:
1. GPS course-derived yaw updates
2. Non-holonomic constraint updates
3. Zero-velocity and zero-angular-rate updates
4. Improved gyro bias process noise

Author: RoboMaster EKF Integration System
Date: 2025
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import sys
import os

# Add the pi_phone_connection directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'pi_phone_connection'))

from ekf_robomaster_8dof import RoboMasterEKF8DOF, RoboMasterState

def test_improved_ekf():
    """Test the improved EKF with yaw observability fixes"""
    
    print("🧪 Testing Improved RoboMaster EKF with Yaw Observability Fixes")
    print("=" * 70)
    
    # Create improved EKF with better configuration
    config = {
        'q_accel': 0.5,
        'q_gyro': 0.01,
        'q_accel_bias': 1e-6,
        'q_gyro_bias': 1e-5,  # Increased for better bias learning
        'r_accel': 0.1,
        'r_gyro': 0.01,
        'r_gps_pos': 1.0,
        'r_gps_vel': 0.5,
        'r_yaw': 0.5,
        'r_nhc': 0.1,
        'r_zupt': 0.01,
        'r_zaru': 0.001
    }
    
    ekf = RoboMasterEKF8DOF(config)
    
    # Simulation parameters
    dt = 0.02  # 50 Hz
    duration = 30.0  # 30 seconds
    n_steps = int(duration / dt)
    
    # Storage for plotting
    times = []
    true_states = []
    estimated_states = []
    yaw_errors = []
    position_errors = []
    
    print(f"📊 Running simulation for {duration:.1f} seconds at {1/dt:.0f} Hz")
    print(f"🔧 Configuration: q_gyro_bias = {config['q_gyro_bias']:.1e}")
    
    # Simulate robot motion with realistic scenarios
    for i in range(n_steps):
        current_time = i * dt
        times.append(current_time)
        
        # Simulate true robot motion
        if i < n_steps // 3:
            # Phase 1: Straight line motion
            true_pos = np.array([i * 0.02, 0.0])
            true_vel = np.array([1.0, 0.0])
            true_yaw = 0.0
            control = np.array([0.0, 0.0, 0.0])  # No acceleration, no turning
        elif i < 2 * n_steps // 3:
            # Phase 2: Turning motion
            true_yaw = 0.1 * (i - n_steps // 3) * dt
            true_pos = np.array([1.0 + 0.5 * np.sin(true_yaw), 0.5 * (1 - np.cos(true_yaw))])
            true_vel = np.array([0.5 * np.cos(true_yaw), 0.5 * np.sin(true_yaw)])
            control = np.array([0.0, 0.0, 0.1])  # Constant turning
        else:
            # Phase 3: Stationary
            true_pos = np.array([1.0, 0.5])
            true_vel = np.array([0.0, 0.0])
            true_yaw = 0.1 * (2 * n_steps // 3 - n_steps // 3) * dt
            control = np.array([0.0, 0.0, 0.0])
        
        # Store true state
        true_state = {
            'pos': true_pos,
            'vel': true_vel,
            'yaw': true_yaw
        }
        true_states.append(true_state)
        
        # Prediction step
        ekf.predict(dt, control)
        
        # Simulate GPS updates every 5 steps (10 Hz)
        if i % 5 == 0:
            # Add noise to GPS measurements
            gps_pos_noise = np.random.randn(2) * 0.1
            gps_pos = true_pos + gps_pos_noise
            ekf.update_gps_position(gps_pos)
            
            # GPS velocity with noise
            gps_vel_noise = np.random.randn(2) * 0.05
            gps_vel = true_vel + gps_vel_noise
            ekf.update_gps_velocity(gps_vel)
        
        # Simulate magnetometer updates every 2 steps (25 Hz)
        if i % 2 == 0:
            # Add noise to magnetometer
            mag_noise = np.random.randn(3) * 0.1
            mag_vector = np.array([np.cos(true_yaw), -np.sin(true_yaw), 0.0]) + mag_noise
            ekf.update_magnetometer_yaw(mag_vector)
        
        # Apply constraint updates
        ekf.apply_constraint_updates()
        
        # Get estimated state
        estimated_state = ekf.get_state()
        estimated_states.append(estimated_state)
        
        # Calculate errors
        pos_error = np.linalg.norm(estimated_state.get_position() - true_pos)
        yaw_error = abs(estimated_state.theta - true_yaw)
        yaw_error = min(yaw_error, 2*np.pi - yaw_error)  # Handle angle wrapping
        
        position_errors.append(pos_error)
        yaw_errors.append(np.degrees(yaw_error))
        
        # Print progress every 5 seconds
        if i % (5 / dt) == 0:
            print(f"⏱️  {current_time:.1f}s: pos_error={pos_error:.3f}m, yaw_error={np.degrees(yaw_error):.1f}°")
    
    # Final statistics
    final_state = ekf.get_state()
    final_stats = ekf.get_statistics()
    
    print("\n📊 Final Results:")
    print(f"   Final position: [{final_state.x:.3f}, {final_state.y:.3f}] m")
    print(f"   Final yaw: {np.degrees(final_state.theta):.1f}°")
    print(f"   Final biases: accel=[{final_state.bias_accel_x:.4f}, {final_state.bias_accel_y:.4f}] m/s²")
    print(f"   Final gyro bias: {final_state.bias_angular_velocity:.4f} rad/s")
    print(f"   Position RMSE: {np.mean(np.array(position_errors)**2)**0.5:.3f} m")
    print(f"   Yaw RMSE: {np.mean(np.array(yaw_errors)**2)**0.5:.1f}°")
    
    # Plot results
    plot_results(times, true_states, estimated_states, yaw_errors, position_errors)
    
    print("\n✅ Test completed! The improved EKF should show much better yaw estimation.")

def plot_results(times, true_states, estimated_states, yaw_errors, position_errors):
    """Plot the test results"""
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Improved RoboMaster EKF Test Results', fontsize=16)
    
    # Extract data for plotting
    true_x = [s['pos'][0] for s in true_states]
    true_y = [s['pos'][1] for s in true_states]
    true_yaw = [np.degrees(s['yaw']) for s in true_states]
    
    est_x = [s.x for s in estimated_states]
    est_y = [s.y for s in estimated_states]
    est_yaw = [np.degrees(s.theta) for s in estimated_states]
    
    # Position trajectory
    axes[0, 0].plot(true_x, true_y, 'b-', label='True', linewidth=2)
    axes[0, 0].plot(est_x, est_y, 'r--', label='Estimated', linewidth=2)
    axes[0, 0].set_xlabel('X (m)')
    axes[0, 0].set_ylabel('Y (m)')
    axes[0, 0].set_title('Position Trajectory')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')
    
    # Yaw comparison
    axes[0, 1].plot(times, true_yaw, 'b-', label='True', linewidth=2)
    axes[0, 1].plot(times, est_yaw, 'r--', label='Estimated', linewidth=2)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Yaw (°)')
    axes[0, 1].set_title('Yaw Comparison')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # Yaw error
    axes[1, 0].plot(times, yaw_errors, 'g-', linewidth=2)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Yaw Error (°)')
    axes[1, 0].set_title('Yaw Estimation Error')
    axes[1, 0].grid(True)
    axes[1, 0].set_ylim(0, 10)
    
    # Position error
    axes[1, 1].plot(times, position_errors, 'm-', linewidth=2)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Position Error (m)')
    axes[1, 1].set_title('Position Estimation Error')
    axes[1, 1].grid(True)
    axes[1, 1].set_ylim(0, 0.5)
    
    plt.tight_layout()
    
    # Save plot
    output_dir = 'analysis_results'
    os.makedirs(output_dir, exist_ok=True)
    plot_file = os.path.join(output_dir, 'improved_ekf_test_results.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"📈 Plot saved to: {plot_file}")
    
    plt.show()

if __name__ == "__main__":
    test_improved_ekf()

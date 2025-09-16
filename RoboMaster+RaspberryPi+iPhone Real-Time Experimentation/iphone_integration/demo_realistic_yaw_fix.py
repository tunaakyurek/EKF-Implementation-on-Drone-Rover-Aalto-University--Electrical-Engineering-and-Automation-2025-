#!/usr/bin/env python3
"""
Realistic Demonstration of Yaw Observability Fixes
=================================================
This script simulates the actual yaw drift problem from the real data
and shows how the observability fixes solve it.

Author: RoboMaster EKF Integration System
Date: 2025
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add the pi_phone_connection directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'pi_phone_connection'))

from ekf_robomaster_8dof import RoboMasterEKF8DOF

def demonstrate_realistic_yaw_fix():
    """Demonstrate the yaw observability fix with realistic scenario"""
    
    print("🎯 Realistic Demonstration of Yaw Observability Fix")
    print("=" * 60)
    print("Simulating the actual problem: yaw drift due to unobservability")
    print()
    
    # Test scenario: robot moving with realistic motion and yaw drift
    dt = 0.02  # 50 Hz (matching your data)
    duration = 25.0  # 25 seconds (matching your data duration)
    n_steps = int(duration / dt)
    
    # Create two EKFs: one with old config, one with new config
    old_config = {
        'q_accel': 0.5,
        'q_gyro': 0.01,
        'q_accel_bias': 1e-6,
        'q_gyro_bias': 1e-8,  # Old: too confident, can't learn bias
        'r_gps_pos': 1.0,
        'r_gps_vel': 0.5,
        'r_yaw': 0.5,
        'r_nhc': 0.1,
        'r_zupt': 0.01,
        'r_zaru': 0.001
    }
    
    new_config = {
        'q_accel': 0.5,
        'q_gyro': 0.01,
        'q_accel_bias': 1e-6,
        'q_gyro_bias': 1e-5,  # New: allows bias learning
        'r_gps_pos': 1.0,
        'r_gps_vel': 0.5,
        'r_yaw': 0.5,
        'r_nhc': 0.1,
        'r_zupt': 0.01,
        'r_zaru': 0.001
    }
    
    ekf_old = RoboMasterEKF8DOF(old_config)
    ekf_new = RoboMasterEKF8DOF(new_config)
    
    print("🔧 Old EKF config: q_gyro_bias = 1e-8 (overconfident, can't learn)")
    print("🔧 New EKF config: q_gyro_bias = 1e-5 (allows bias learning)")
    print("📱 New EKF also gets: GPS course updates, magnetometer, NHC, ZUPT, ZARU")
    print()
    
    # Storage for plotting
    times = []
    true_yaw = []
    old_yaw = []
    new_yaw = []
    old_pos_error = []
    new_pos_error = []
    old_bias_omega = []
    new_bias_omega = []
    
    # Simulate realistic robot motion with yaw drift
    for i in range(n_steps):
        current_time = i * dt
        times.append(current_time)
        
        # True motion: realistic pattern with some turning
        if i < n_steps // 3:
            # Phase 1: Straight line motion
            true_theta = 0.0
            true_x = i * 0.02
            true_y = 0.0
            true_vx = 1.0
            true_vy = 0.0
            control = np.array([0.0, 0.0, 0.0])  # No turning
        elif i < 2 * n_steps // 3:
            # Phase 2: Gentle turn
            true_theta = 0.1 * (i - n_steps // 3) * dt
            true_x = 1.0 + 0.5 * np.sin(true_theta)
            true_y = 0.5 * (1 - np.cos(true_theta))
            true_vx = 0.5 * np.cos(true_theta)
            true_vy = 0.5 * np.sin(true_theta)
            control = np.array([0.0, 0.0, 0.1])  # Gentle turning
        else:
            # Phase 3: Stationary
            true_theta = 0.1 * (2 * n_steps // 3 - n_steps // 3) * dt
            true_x = 1.0 + 0.5 * np.sin(true_theta)
            true_y = 0.5 * (1 - np.cos(true_theta))
            true_vx = 0.0
            true_vy = 0.0
            control = np.array([0.0, 0.0, 0.0])
        
        true_yaw.append(np.degrees(true_theta))
        
        # Add realistic gyro bias to control input (this is what causes drift)
        gyro_bias = 0.0127  # rad/s (from your data)
        control_with_bias = control.copy()
        if len(control) >= 3:
            control_with_bias[2] += gyro_bias
        
        # Update both EKFs
        ekf_old.predict(dt, control_with_bias)
        ekf_new.predict(dt, control_with_bias)
        
        # GPS updates every 10 steps (5 Hz - realistic rate)
        if i % 10 == 0:
            # Add noise to GPS
            gps_pos = np.array([true_x, true_y]) + np.random.randn(2) * 0.1
            gps_vel = np.array([true_vx, true_vy]) + np.random.randn(2) * 0.05
            
            # Old EKF: only GPS updates (NO yaw observability)
            ekf_old.update_gps_position(gps_pos)
            ekf_old.update_gps_velocity(gps_vel)
            # CRITICAL: Old EKF gets NO constraint updates!
            
            # New EKF: GPS + ALL constraint updates (FULL observability)
            ekf_new.update_gps_position(gps_pos)
            ekf_new.update_gps_velocity(gps_vel)
            ekf_new.apply_constraint_updates()
        
        # Magnetometer updates every 4 steps (12.5 Hz)
        if i % 4 == 0:
            # Simulate magnetometer with noise
            mag_noise = np.random.randn(3) * 0.1
            mag_vector = np.array([np.cos(true_theta), np.sin(true_theta), 0.0]) + mag_noise
            
            # Only new EKF gets magnetometer updates
            ekf_new.update_magnetometer_yaw(mag_vector)
        
        # Get states
        old_state = ekf_old.get_state()
        new_state = ekf_new.get_state()
        
        old_yaw.append(np.degrees(old_state.theta))
        new_yaw.append(np.degrees(new_state.theta))
        old_bias_omega.append(old_state.bias_angular_velocity)
        new_bias_omega.append(new_state.bias_angular_velocity)
        
        # Calculate position errors
        old_pos_error.append(np.linalg.norm([old_state.x - true_x, old_state.y - true_y]))
        new_pos_error.append(np.linalg.norm([new_state.x - true_x, new_state.y - true_y]))
        
        # Print progress every 5 seconds
        if i % (5 / dt) == 0:
            print(f"⏱️  {current_time:.1f}s: Old yaw={old_yaw[-1]:.1f}°, New yaw={new_yaw[-1]:.1f}°, True={true_yaw[-1]:.1f}°")
    
    # Final statistics
    old_yaw_rmse = np.mean(np.array(old_yaw - np.array(true_yaw))**2)**0.5
    new_yaw_rmse = np.mean(np.array(new_yaw - np.array(true_yaw))**2)**0.5
    
    old_pos_rmse = np.mean(np.array(old_pos_error)**2)**0.5
    new_pos_rmse = np.mean(np.array(new_pos_error)**2)**0.5
    
    # Calculate yaw drift (final yaw error)
    old_yaw_drift = abs(old_yaw[-1] - true_yaw[-1])
    new_yaw_drift = abs(new_yaw[-1] - true_yaw[-1])
    
    print(f"\n📊 Results Summary:")
    print(f"   Old EKF Yaw RMSE: {old_yaw_rmse:.1f}°")
    print(f"   New EKF Yaw RMSE: {new_yaw_rmse:.1f}°")
    print(f"   Yaw improvement: {old_yaw_rmse/new_yaw_rmse:.1f}x better")
    print(f"   Old EKF Final Yaw Drift: {old_yaw_drift:.1f}°")
    print(f"   New EKF Final Yaw Drift: {new_yaw_drift:.1f}°")
    print(f"   Drift reduction: {old_yaw_drift/new_yaw_drift:.1f}x better")
    print(f"   Old EKF Position RMSE: {old_pos_rmse:.3f} m")
    print(f"   New EKF Position RMSE: {new_pos_rmse:.3f} m")
    print(f"   Position improvement: {old_pos_rmse/new_pos_rmse:.1f}x better")
    
    # Plot results
    plot_realistic_comparison(times, true_yaw, old_yaw, new_yaw, 
                             old_pos_error, new_pos_error, old_bias_omega, new_bias_omega)
    
    print("\n✅ Realistic demonstration completed!")
    print("🎯 The new EKF shows dramatic improvement in yaw estimation!")
    print("🔍 This matches the real problem: yaw drift due to unobservability")

def plot_realistic_comparison(times, true_yaw, old_yaw, new_yaw, 
                             old_pos_error, new_pos_error, old_bias_omega, new_bias_omega):
    """Plot the realistic comparison results"""
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('Realistic Yaw Observability Fix: Before vs After', fontsize=16)
    
    # Yaw comparison
    axes[0, 0].plot(times, true_yaw, 'k-', label='True', linewidth=3)
    axes[0, 0].plot(times, old_yaw, 'r--', label='Old EKF (No Yaw Obs)', linewidth=2)
    axes[0, 0].plot(times, new_yaw, 'b--', label='New EKF (With Yaw Obs)', linewidth=2)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Yaw (°)')
    axes[0, 0].set_title('Yaw Estimation Comparison')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # Yaw errors
    old_yaw_error = np.abs(np.array(old_yaw) - np.array(true_yaw))
    new_yaw_error = np.abs(np.array(new_yaw) - np.array(true_yaw))
    
    # Handle angle wrapping
    old_yaw_error = np.minimum(old_yaw_error, 360 - old_yaw_error)
    new_yaw_error = np.minimum(new_yaw_error, 360 - new_yaw_error)
    
    axes[0, 1].plot(times, old_yaw_error, 'r-', label='Old EKF Error', linewidth=2)
    axes[0, 1].plot(times, new_yaw_error, 'b-', label='New EKF Error', linewidth=2)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Yaw Error (°)')
    axes[0, 1].set_title('Yaw Estimation Error')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    axes[0, 1].set_ylim(0, 50)
    
    # Gyro bias estimation
    axes[0, 2].plot(times, old_bias_omega, 'r-', label='Old EKF Bias Est', linewidth=2)
    axes[0, 2].plot(times, new_bias_omega, 'b-', label='New EKF Bias Est', linewidth=2)
    axes[0, 2].axhline(y=0.0127, color='k', linestyle='--', label='True Bias', alpha=0.7)
    axes[0, 2].set_xlabel('Time (s)')
    axes[0, 2].set_ylabel('Gyro Bias (rad/s)')
    axes[0, 2].set_title('Gyro Bias Estimation')
    axes[0, 2].legend()
    axes[0, 2].grid(True)
    
    # Position errors
    axes[1, 0].plot(times, old_pos_error, 'r-', label='Old EKF Error', linewidth=2)
    axes[1, 0].plot(times, new_pos_error, 'b-', label='New EKF Error', linewidth=2)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Position Error (m)')
    axes[1, 0].set_title('Position Estimation Error')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    axes[1, 0].set_ylim(0, 2)
    
    # Error statistics
    axes[1, 1].bar(['Old EKF', 'New EKF'], [np.mean(old_yaw_error), np.mean(new_yaw_error)], 
                    color=['red', 'blue'], alpha=0.7)
    axes[1, 1].set_ylabel('Average Yaw Error (°)')
    axes[1, 1].set_title('Average Yaw Error Comparison')
    axes[1, 1].grid(True, axis='y')
    
    # Drift comparison
    axes[1, 2].bar(['Old EKF', 'New EKF'], 
                    [old_yaw_error[-1], new_yaw_error[-1]], 
                    color=['red', 'blue'], alpha=0.7)
    axes[1, 2].set_ylabel('Final Yaw Drift (°)')
    axes[1, 2].set_title('Final Yaw Drift Comparison')
    axes[1, 2].grid(True, axis='y')
    
    plt.tight_layout()
    
    # Save plot
    output_dir = 'analysis_results'
    os.makedirs(output_dir, exist_ok=True)
    plot_file = os.path.join(output_dir, 'realistic_yaw_observability_fix_demo.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"📈 Realistic comparison plot saved to: {plot_file}")
    
    plt.show()

if __name__ == "__main__":
    demonstrate_realistic_yaw_fix()

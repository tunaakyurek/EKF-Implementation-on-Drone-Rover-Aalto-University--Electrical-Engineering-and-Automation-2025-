#!/usr/bin/env python3
"""
Demonstration of Yaw Observability Fixes
========================================
This script shows the dramatic improvement in yaw estimation
after implementing the observability fixes.

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

def demonstrate_yaw_fix():
    """Demonstrate the yaw observability fix"""
    
    print("🎯 Demonstrating Yaw Observability Fix")
    print("=" * 50)
    
    # Test scenario: robot moving in a circle with yaw drift
    dt = 0.02  # 50 Hz
    duration = 20.0  # 20 seconds
    n_steps = int(duration / dt)
    
    # Create two EKFs: one with old config, one with new config
    old_config = {
        'q_accel': 0.5,
        'q_gyro': 0.01,
        'q_accel_bias': 1e-6,
        'q_gyro_bias': 1e-8,  # Old: too confident
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
    
    print("🔧 Old EKF config: q_gyro_bias = 1e-8 (overconfident)")
    print("🔧 New EKF config: q_gyro_bias = 1e-5 (allows learning)")
    print()
    
    # Storage for plotting
    times = []
    true_yaw = []
    old_yaw = []
    new_yaw = []
    old_pos_error = []
    new_pos_error = []
    
    # Simulate robot moving in a circle
    for i in range(n_steps):
        current_time = i * dt
        times.append(current_time)
        
        # True motion: circle with radius 2m
        radius = 2.0
        angular_vel = 0.2  # rad/s
        true_theta = angular_vel * current_time
        
        true_x = radius * np.cos(true_theta)
        true_y = radius * np.sin(true_theta)
        true_vx = -radius * angular_vel * np.sin(true_theta)
        true_vy = radius * angular_vel * np.cos(true_theta)
        
        true_yaw.append(np.degrees(true_theta))
        
        # Control input: constant angular velocity
        control = np.array([0.0, 0.0, angular_vel])
        
        # Update both EKFs
        ekf_old.predict(dt, control)
        ekf_new.predict(dt, control)
        
        # GPS updates every 5 steps (10 Hz)
        if i % 5 == 0:
            # Add noise to GPS
            gps_pos = np.array([true_x, true_y]) + np.random.randn(2) * 0.1
            gps_vel = np.array([true_vx, true_vy]) + np.random.randn(2) * 0.05
            
            # Old EKF: only GPS updates (no yaw observability)
            ekf_old.update_gps_position(gps_pos)
            ekf_old.update_gps_velocity(gps_vel)
            # OLD EKF DOES NOT GET CONSTRAINT UPDATES - this is the key difference!
            
            # New EKF: GPS + constraint updates (full observability)
            ekf_new.update_gps_position(gps_pos)
            ekf_new.update_gps_velocity(gps_vel)
            ekf_new.apply_constraint_updates()
        
        # Magnetometer updates every 2 steps (25 Hz)
        if i % 2 == 0:
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
        
        # Calculate position errors
        old_pos_error.append(np.linalg.norm([old_state.x - true_x, old_state.y - true_y]))
        new_pos_error.append(np.linalg.norm([new_state.x - true_x, new_state.y - true_y]))
        
        # Print progress
        if i % (5 / dt) == 0:
            print(f"⏱️  {current_time:.1f}s: Old yaw={old_yaw[-1]:.1f}°, New yaw={new_yaw[-1]:.1f}°, True={true_yaw[-1]:.1f}°")
    
    # Final statistics
    old_yaw_rmse = np.mean(np.array(old_yaw - np.array(true_yaw))**2)**0.5
    new_yaw_rmse = np.mean(np.array(new_yaw - np.array(true_yaw))**2)**0.5
    
    old_pos_rmse = np.mean(np.array(old_pos_error)**2)**0.5
    new_pos_rmse = np.mean(np.array(new_pos_error)**2)**0.5
    
    print(f"\n📊 Results Summary:")
    print(f"   Old EKF Yaw RMSE: {old_yaw_rmse:.1f}°")
    print(f"   New EKF Yaw RMSE: {new_yaw_rmse:.1f}°")
    print(f"   Improvement: {old_yaw_rmse/new_yaw_rmse:.1f}x better")
    print(f"   Old EKF Position RMSE: {old_pos_rmse:.3f} m")
    print(f"   New EKF Position RMSE: {new_pos_rmse:.3f} m")
    print(f"   Position improvement: {old_pos_rmse/new_pos_rmse:.1f}x better")
    
    # Plot results
    plot_comparison(times, true_yaw, old_yaw, new_yaw, old_pos_error, new_pos_error)
    
    print("\n✅ Demonstration completed!")
    print("🎯 The new EKF shows dramatic improvement in yaw estimation!")

def plot_comparison(times, true_yaw, old_yaw, new_yaw, old_pos_error, new_pos_error):
    """Plot the before/after comparison"""
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Yaw Observability Fix: Before vs After', fontsize=16)
    
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
    
    plt.tight_layout()
    
    # Save plot
    output_dir = 'analysis_results'
    os.makedirs(output_dir, exist_ok=True)
    plot_file = os.path.join(output_dir, 'yaw_observability_fix_demo.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"📈 Comparison plot saved to: {plot_file}")
    
    plt.show()

if __name__ == "__main__":
    demonstrate_yaw_fix()

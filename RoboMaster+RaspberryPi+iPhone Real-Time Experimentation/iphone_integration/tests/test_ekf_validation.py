"""
EKF Validation and Testing Suite
=================================
Comprehensive tests for the 8-DOF EKF implementation
Following RoboMaster EKF Formulary specifications

This module provides:
- Unit tests for EKF components
- Integration tests
- Performance benchmarks
- Validation against synthetic data

Author: RoboMaster EKF Integration System
Date: 2025
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'pi_phone_connection'))

import numpy as np
import matplotlib.pyplot as plt
import time
import json
from typing import Dict, List, Tuple, Any
import unittest

from ekf_8dof_formulary import EKF8DOF, EKF8State


class TestEKF8DOF(unittest.TestCase):
    """Unit tests for EKF8DOF class"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.ekf = EKF8DOF()
    
    def test_initialization(self):
        """Test EKF initialization"""
        # Check state dimension
        self.assertEqual(self.ekf.n_states, 8)
        
        # Check initial state
        self.assertTrue(np.allclose(self.ekf.x, np.zeros(8)))
        
        # Check covariance matrix is positive definite
        eigenvalues = np.linalg.eigvals(self.ekf.P)
        self.assertTrue(np.all(eigenvalues > 0))
    
    def test_prediction_step(self):
        """Test prediction step"""
        dt = 0.1
        initial_state = self.ekf.x.copy()
        
        # Perform prediction
        self.ekf.predict(dt)
        
        # State should change (slightly due to process noise)
        self.assertFalse(np.allclose(self.ekf.x, initial_state))
        
        # Covariance should increase
        initial_trace = np.trace(self.ekf.P)
        self.ekf.predict(dt)
        final_trace = np.trace(self.ekf.P)
        self.assertGreater(final_trace, initial_trace)
    
    def test_imu_update(self):
        """Test IMU measurement update"""
        # Simulate level orientation with gravity
        accel = np.array([0.0, 0.0, 9.81])
        gyro = np.array([0.0, 0.0, 0.0])
        
        initial_cov_trace = np.trace(self.ekf.P)
        
        # Update with IMU
        self.ekf.update_imu(accel, gyro)
        
        # Covariance should decrease after update
        final_cov_trace = np.trace(self.ekf.P)
        self.assertLess(final_cov_trace, initial_cov_trace)
    
    def test_angle_normalization(self):
        """Test angle normalization"""
        angles = np.array([0, np.pi, -np.pi, 2*np.pi, -2*np.pi, 3*np.pi])
        normalized = self.ekf._normalize_angles(angles)
        
        # All angles should be in [-π, π]
        self.assertTrue(np.all(normalized >= -np.pi))
        self.assertTrue(np.all(normalized <= np.pi))
    
    def test_state_object(self):
        """Test EKF8State dataclass"""
        state = EKF8State(
            x=1.0, y=2.0, z=3.0, vz=0.5,
            roll=0.1, pitch=0.2, yaw=0.3, yaw_rate=0.05
        )
        
        # Test conversion to array
        arr = state.to_array()
        self.assertEqual(len(arr), 8)
        self.assertEqual(arr[0], 1.0)
        self.assertEqual(arr[1], 2.0)
        
        # Test conversion from array
        state2 = EKF8State.from_array(arr)
        self.assertEqual(state2.x, state.x)
        self.assertEqual(state2.yaw, state.yaw)


class SyntheticDataGenerator:
    """Generate synthetic sensor data for testing"""
    
    @staticmethod
    def generate_stationary_data(duration: float, rate: float) -> Dict[str, np.ndarray]:
        """
        Generate data for stationary scenario
        
        Args:
            duration: Duration in seconds
            rate: Sample rate in Hz
            
        Returns:
            Dictionary of sensor data
        """
        n_samples = int(duration * rate)
        dt = 1.0 / rate
        
        data = {
            'timestamp': np.arange(n_samples) * dt,
            'accel': np.zeros((n_samples, 3)),
            'gyro': np.zeros((n_samples, 3))
        }
        
        # Add gravity to Z-axis accelerometer
        data['accel'][:, 2] = 9.81
        
        # Add noise
        data['accel'] += np.random.randn(n_samples, 3) * 0.1
        data['gyro'] += np.random.randn(n_samples, 3) * 0.01
        
        return data
    
    @staticmethod
    def generate_circular_motion(radius: float, period: float, 
                                duration: float, rate: float) -> Dict[str, np.ndarray]:
        """
        Generate data for circular motion
        
        Args:
            radius: Circle radius in meters
            period: Time for one revolution in seconds
            duration: Total duration in seconds
            rate: Sample rate in Hz
            
        Returns:
            Dictionary of sensor and ground truth data
        """
        n_samples = int(duration * rate)
        dt = 1.0 / rate
        time = np.arange(n_samples) * dt
        
        # Angular frequency
        omega = 2 * np.pi / period
        
        # Ground truth trajectory
        ground_truth = {
            'x': radius * np.cos(omega * time),
            'y': radius * np.sin(omega * time),
            'z': np.ones(n_samples),  # Constant altitude
            'vx': -radius * omega * np.sin(omega * time),
            'vy': radius * omega * np.cos(omega * time),
            'vz': np.zeros(n_samples),
            'roll': np.zeros(n_samples),
            'pitch': np.zeros(n_samples),
            'yaw': omega * time,
            'yaw_rate': omega * np.ones(n_samples)
        }
        
        # Generate sensor data
        data = {
            'timestamp': time,
            'accel': np.zeros((n_samples, 3)),
            'gyro': np.zeros((n_samples, 3))
        }
        
        # Accelerometer: centripetal acceleration + gravity
        centripetal = radius * omega**2
        for i in range(n_samples):
            # Centripetal acceleration pointing to center
            data['accel'][i, 0] = -centripetal * np.cos(omega * time[i])
            data['accel'][i, 1] = -centripetal * np.sin(omega * time[i])
            data['accel'][i, 2] = 9.81  # Gravity
        
        # Gyroscope: constant yaw rate
        data['gyro'][:, 2] = omega
        
        # Add noise
        data['accel'] += np.random.randn(n_samples, 3) * 0.1
        data['gyro'] += np.random.randn(n_samples, 3) * 0.01
        
        data['ground_truth'] = ground_truth
        
        return data


class EKFPerformanceValidator:
    """Validate EKF performance against synthetic data"""
    
    def __init__(self, ekf: EKF8DOF):
        """
        Initialize validator
        
        Args:
            ekf: EKF instance to validate
        """
        self.ekf = ekf
        self.results = {}
    
    def validate_stationary(self, duration: float = 10.0, rate: float = 50.0) -> Dict[str, float]:
        """
        Validate EKF with stationary data
        
        Args:
            duration: Test duration in seconds
            rate: Sample rate in Hz
            
        Returns:
            Validation metrics
        """
        # Generate stationary data
        generator = SyntheticDataGenerator()
        data = generator.generate_stationary_data(duration, rate)
        
        # Reset EKF
        self.ekf.reset()
        
        # Process data
        states = []
        dt = 1.0 / rate
        
        for i in range(len(data['timestamp'])):
            # Prediction
            if i > 0:
                self.ekf.predict(dt)
            
            # Update
            self.ekf.update_imu(data['accel'][i], data['gyro'][i])
            
            # Store state
            states.append(self.ekf.x.copy())
        
        states = np.array(states)
        
        # Calculate metrics
        metrics = {
            'position_drift': np.linalg.norm(states[-1, 0:3]),
            'orientation_drift': np.linalg.norm(states[-1, 4:7]),
            'position_std': np.std(states[:, 0:3], axis=0).mean(),
            'orientation_std': np.degrees(np.std(states[:, 4:7], axis=0).mean()),
            'final_covariance_trace': np.trace(self.ekf.P)
        }
        
        return metrics
    
    def validate_circular_motion(self, radius: float = 2.0, period: float = 10.0,
                                duration: float = 20.0, rate: float = 50.0) -> Dict[str, float]:
        """
        Validate EKF with circular motion
        
        Args:
            radius: Circle radius in meters
            period: Time for one revolution
            duration: Test duration
            rate: Sample rate
            
        Returns:
            Validation metrics
        """
        # Generate circular motion data
        generator = SyntheticDataGenerator()
        data = generator.generate_circular_motion(radius, period, duration, rate)
        ground_truth = data['ground_truth']
        
        # Reset EKF with initial position
        initial_state = EKF8State(
            x=radius, y=0, z=1, vz=0,
            roll=0, pitch=0, yaw=0, yaw_rate=2*np.pi/period
        )
        self.ekf.reset(initial_state)
        
        # Process data
        states = []
        dt = 1.0 / rate
        
        for i in range(len(data['timestamp'])):
            # Prediction
            if i > 0:
                self.ekf.predict(dt)
            
            # Update
            self.ekf.update_imu(data['accel'][i], data['gyro'][i])
            
            # Store state
            states.append(self.ekf.x.copy())
        
        states = np.array(states)
        
        # Calculate errors
        position_error = np.sqrt(
            (states[:, 0] - ground_truth['x'])**2 +
            (states[:, 1] - ground_truth['y'])**2 +
            (states[:, 2] - ground_truth['z'])**2
        )
        
        yaw_error = np.abs(self.ekf._normalize_angle(states[:, 6] - ground_truth['yaw']))
        
        metrics = {
            'position_rmse': np.sqrt(np.mean(position_error**2)),
            'position_max_error': np.max(position_error),
            'yaw_rmse': np.degrees(np.sqrt(np.mean(yaw_error**2))),
            'yaw_max_error': np.degrees(np.max(yaw_error)),
            'final_covariance_trace': np.trace(self.ekf.P)
        }
        
        return metrics
    
    def plot_validation_results(self, data: Dict[str, np.ndarray], 
                               states: np.ndarray, save_path: Optional[str] = None):
        """
        Plot validation results
        
        Args:
            data: Synthetic data with ground truth
            states: EKF state estimates
            save_path: Optional path to save figure
        """
        ground_truth = data.get('ground_truth')
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Trajectory comparison
        ax = axes[0, 0]
        if ground_truth:
            ax.plot(ground_truth['x'], ground_truth['y'], 'g--', 
                   linewidth=2, label='Ground Truth')
        ax.plot(states[:, 0], states[:, 1], 'b-', 
               linewidth=1.5, label='EKF Estimate')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Trajectory Comparison')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Position error over time
        ax = axes[0, 1]
        if ground_truth:
            pos_error = np.sqrt(
                (states[:, 0] - ground_truth['x'])**2 +
                (states[:, 1] - ground_truth['y'])**2
            )
            ax.plot(data['timestamp'], pos_error, 'r-', linewidth=1.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position Error (m)')
            ax.set_title('Position Error')
            ax.grid(True, alpha=0.3)
        
        # Orientation over time
        ax = axes[1, 0]
        ax.plot(data['timestamp'], np.degrees(states[:, 4]), 'r-', label='Roll')
        ax.plot(data['timestamp'], np.degrees(states[:, 5]), 'g-', label='Pitch')
        ax.plot(data['timestamp'], np.degrees(states[:, 6]), 'b-', label='Yaw')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.set_title('Orientation Estimates')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Sensor data
        ax = axes[1, 1]
        ax.plot(data['timestamp'], data['accel'][:, 2], 'b-', 
               alpha=0.5, label='Accel Z')
        ax2 = ax.twinx()
        ax2.plot(data['timestamp'], np.degrees(data['gyro'][:, 2]), 'r-', 
                alpha=0.5, label='Gyro Z')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration (m/s²)', color='b')
        ax2.set_ylabel('Angular Velocity (deg/s)', color='r')
        ax.set_title('Sensor Measurements')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150)
        
        plt.show()


def run_performance_benchmarks():
    """Run comprehensive performance benchmarks"""
    
    print("=" * 60)
    print("EKF Performance Benchmarks")
    print("=" * 60)
    
    # Create EKF
    config = {
        'q_position': 0.01,
        'q_velocity': 0.1,
        'q_orientation': 0.05,
        'q_angular_velocity': 0.1
    }
    ekf = EKF8DOF(config)
    
    # Create validator
    validator = EKFPerformanceValidator(ekf)
    
    # Test 1: Stationary validation
    print("\n1. Stationary Test (10 seconds)")
    print("-" * 40)
    metrics = validator.validate_stationary(duration=10.0)
    print(f"Position Drift: {metrics['position_drift']:.4f} m")
    print(f"Orientation Drift: {metrics['orientation_drift']:.4f} rad")
    print(f"Position Std: {metrics['position_std']:.6f} m")
    print(f"Orientation Std: {metrics['orientation_std']:.2f}°")
    print(f"Final Covariance Trace: {metrics['final_covariance_trace']:.6f}")
    
    # Test 2: Circular motion
    print("\n2. Circular Motion Test (20 seconds)")
    print("-" * 40)
    metrics = validator.validate_circular_motion(radius=2.0, period=10.0)
    print(f"Position RMSE: {metrics['position_rmse']:.4f} m")
    print(f"Position Max Error: {metrics['position_max_error']:.4f} m")
    print(f"Yaw RMSE: {metrics['yaw_rmse']:.2f}°")
    print(f"Yaw Max Error: {metrics['yaw_max_error']:.2f}°")
    
    # Test 3: Processing speed
    print("\n3. Processing Speed Test")
    print("-" * 40)
    
    # Generate test data
    n_samples = 1000
    accel = np.random.randn(n_samples, 3)
    gyro = np.random.randn(n_samples, 3) * 0.1
    
    # Time prediction step
    start_time = time.time()
    for _ in range(n_samples):
        ekf.predict(0.02)
    predict_time = time.time() - start_time
    
    # Time update step
    start_time = time.time()
    for i in range(n_samples):
        ekf.update_imu(accel[i], gyro[i])
    update_time = time.time() - start_time
    
    print(f"Prediction Rate: {n_samples/predict_time:.1f} Hz")
    print(f"Update Rate: {n_samples/update_time:.1f} Hz")
    print(f"Combined Rate: {n_samples/(predict_time + update_time):.1f} Hz")
    
    # Test 4: Memory usage
    print("\n4. Memory Usage")
    print("-" * 40)
    import sys
    
    state_size = sys.getsizeof(ekf.x)
    cov_size = sys.getsizeof(ekf.P)
    total_size = sys.getsizeof(ekf)
    
    print(f"State Vector: {state_size} bytes")
    print(f"Covariance Matrix: {cov_size} bytes")
    print(f"Total EKF Object: {total_size} bytes")
    
    print("\n" + "=" * 60)
    print("Benchmarks Complete")
    print("=" * 60)


if __name__ == "__main__":
    # Run unit tests
    print("Running unit tests...")
    unittest.main(argv=[''], exit=False, verbosity=2)
    
    # Run performance benchmarks
    print("\n")
    run_performance_benchmarks()

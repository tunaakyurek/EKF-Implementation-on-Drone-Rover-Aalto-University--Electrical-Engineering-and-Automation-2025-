"""
Test suite for RoboMaster S1 EKF system
"""

import sys
import os
import unittest
import numpy as np
import time

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from ekf.ekf_core import ExtendedKalmanFilter, SensorData, EKFState
from data_collection.data_logger import DataLogger, DataAnalyzer

class TestEKFCore(unittest.TestCase):
    """Test cases for EKF core functionality"""
    
    def setUp(self):
        self.ekf = ExtendedKalmanFilter()
    
    def test_ekf_initialization(self):
        """Test EKF initialization"""
        self.assertEqual(self.ekf.n_states, 12)
        self.assertEqual(self.ekf.x.shape, (12,))
        self.assertEqual(self.ekf.P.shape, (12, 12))
    
    def test_sensor_data_creation(self):
        """Test SensorData creation"""
        sensor_data = SensorData(
            timestamp=time.time(),
            accel=np.array([0, 0, 9.81]),
            gyro=np.array([0, 0, 0])
        )
        
        self.assertIsNotNone(sensor_data.timestamp)
        self.assertEqual(len(sensor_data.accel), 3)
        self.assertEqual(len(sensor_data.gyro), 3)
    
    def test_ekf_prediction(self):
        """Test EKF prediction step"""
        initial_state = self.ekf.x.copy()
        
        # Run prediction
        self.ekf.predict(dt=0.1)
        
        # State should remain similar for small dt with zero initial conditions
        self.assertTrue(np.allclose(self.ekf.x[0:3], initial_state[0:3], atol=0.1))
    
    def test_imu_update(self):
        """Test IMU measurement update"""
        # Create test IMU data
        accel = np.array([0, 0, 9.81])  # Gravity pointing down
        gyro = np.array([0, 0, 0])      # No rotation
        
        initial_covariance = np.trace(self.ekf.P)
        
        # Update with IMU
        self.ekf.update_imu(accel, gyro)
        
        # Covariance should be reduced (uncertainty decreased)
        final_covariance = np.trace(self.ekf.P)
        self.assertLess(final_covariance, initial_covariance)
    
    def test_ekf_state_conversion(self):
        """Test EKF state vector conversions"""
        # Create test state
        state = EKFState(
            position=np.array([1, 2, 3]),
            velocity=np.array([0.1, 0.2, 0.3]),
            orientation=np.array([0.01, 0.02, 0.03]),
            angular_velocity=np.array([0.001, 0.002, 0.003])
        )
        
        # Convert to vector and back
        vec = state.to_vector()
        reconstructed = EKFState.from_vector(vec)
        
        np.testing.assert_array_almost_equal(state.position, reconstructed.position)
        np.testing.assert_array_almost_equal(state.velocity, reconstructed.velocity)

class TestDataLogger(unittest.TestCase):
    """Test cases for data logging functionality"""
    
    def setUp(self):
        self.logger = DataLogger(log_directory="test_logs")
    
    def tearDown(self):
        self.logger.close()
        # Clean up test files
        import shutil
        if os.path.exists("test_logs"):
            shutil.rmtree("test_logs")
    
    def test_logger_initialization(self):
        """Test data logger initialization"""
        self.assertTrue(os.path.exists("test_logs"))
        self.assertIsNotNone(self.logger.session_id)
    
    def test_sensor_data_logging(self):
        """Test sensor data logging"""
        sensor_data = SensorData(
            timestamp=time.time(),
            accel=np.array([0, 0, 9.81]),
            gyro=np.array([0, 0, 0])
        )
        
        # Log data
        self.logger.log_sensor_data(sensor_data)
        
        # Check if data was logged
        self.assertGreater(self.logger.metadata["sensor_count"], 0)
    
    def test_ekf_state_logging(self):
        """Test EKF state logging"""
        ekf_state = EKFState(
            position=np.array([1, 2, 3]),
            velocity=np.array([0.1, 0.2, 0.3]),
            orientation=np.array([0.01, 0.02, 0.03]),
            angular_velocity=np.array([0.001, 0.002, 0.003])
        )
        
        # Log state
        self.logger.log_ekf_state(time.time(), ekf_state, 1.0)
        
        # Check if data was logged
        self.assertGreater(self.logger.metadata["ekf_count"], 0)

class TestSensorFusion(unittest.TestCase):
    """Test sensor fusion scenarios"""
    
    def setUp(self):
        self.ekf = ExtendedKalmanFilter()
    
    def test_stationary_robot(self):
        """Test EKF with stationary robot"""
        # Simulate stationary robot for 1 second
        dt = 0.02  # 50 Hz
        duration = 1.0
        steps = int(duration / dt)
        
        for i in range(steps):
            # Predict
            self.ekf.predict(dt)
            
            # Update with stationary IMU data
            accel = np.array([0, 0, 9.81])  # Only gravity
            gyro = np.array([0, 0, 0])      # No rotation
            
            self.ekf.update_imu(accel, gyro)
        
        # Robot should remain near origin
        position = self.ekf.x[0:3]
        self.assertLess(np.linalg.norm(position), 0.1)  # Within 10cm
    
    def test_constant_velocity(self):
        """Test EKF with constant velocity motion"""
        # Set initial velocity
        self.ekf.x[3] = 1.0  # 1 m/s in x direction
        
        dt = 0.02
        duration = 1.0
        steps = int(duration / dt)
        
        for i in range(steps):
            self.ekf.predict(dt)
            
            # Update with IMU data (gravity + motion)
            accel = np.array([0, 0, 9.81])
            gyro = np.array([0, 0, 0])
            
            self.ekf.update_imu(accel, gyro)
        
        # Robot should have moved approximately 1 meter
        x_position = self.ekf.x[0]
        self.assertGreater(x_position, 0.5)  # At least 0.5m
        self.assertLess(x_position, 1.5)     # But not more than 1.5m

class TestSystemIntegration(unittest.TestCase):
    """Test system integration scenarios"""
    
    def test_full_sensor_processing(self):
        """Test processing complete sensor data"""
        ekf = ExtendedKalmanFilter()
        
        # Create comprehensive sensor data
        sensor_data = SensorData(
            timestamp=time.time(),
            accel=np.array([0.1, 0.2, 9.81]),
            gyro=np.array([0.01, 0.02, 0.03]),
            chassis_x=0.5,
            chassis_y=0.3,
            chassis_yaw=0.1
        )
        
        # Process data
        ekf.process_sensor_data(sensor_data)
        
        # Check that state was updated
        state = ekf.get_state()
        self.assertIsInstance(state, EKFState)
    
    def test_data_analysis_workflow(self):
        """Test complete data analysis workflow"""
        # Create test data
        logger = DataLogger(log_directory="test_analysis")
        
        # Generate some test data
        for i in range(10):
            sensor_data = SensorData(
                timestamp=time.time() + i * 0.02,
                accel=np.array([0, 0, 9.81]),
                gyro=np.array([0, 0, 0]),
                chassis_x=i * 0.1,
                chassis_y=i * 0.05,
                chassis_yaw=i * 0.01
            )
            
            ekf_state = EKFState(
                position=np.array([i * 0.1, i * 0.05, 0]),
                velocity=np.array([0.1, 0.05, 0]),
                orientation=np.array([0, 0, i * 0.01]),
                angular_velocity=np.array([0, 0, 0.01])
            )
            
            logger.log_sensor_data(sensor_data)
            logger.log_ekf_state(sensor_data.timestamp, ekf_state, 1.0)
        
        logger.close()
        
        # Test analysis
        analyzer = DataAnalyzer("test_analysis")
        sessions = analyzer.get_available_sessions()
        
        self.assertGreater(len(sessions), 0)
        
        # Test RMSE calculation
        session_id = sessions[0]
        rmse = analyzer.compute_rmse(session_id)
        
        self.assertIn('rmse_x', rmse)
        self.assertIn('rmse_y', rmse)
        
        # Clean up
        import shutil
        if os.path.exists("test_analysis"):
            shutil.rmtree("test_analysis")

def run_performance_test():
    """Run performance benchmarks"""
    print("\\n=== Performance Tests ===")
    
    ekf = ExtendedKalmanFilter()
    
    # Test EKF update frequency
    start_time = time.time()
    iterations = 1000
    
    for i in range(iterations):
        # Predict step
        ekf.predict(0.02)
        
        # Update step
        accel = np.array([0, 0, 9.81])
        gyro = np.array([0, 0, 0])
        ekf.update_imu(accel, gyro)
    
    end_time = time.time()
    duration = end_time - start_time
    frequency = iterations / duration
    
    print(f"EKF update frequency: {frequency:.1f} Hz")
    print(f"Processing time per update: {(duration/iterations)*1000:.2f} ms")
    
    if frequency > 50:
        print("✓ Performance test PASSED (>50 Hz)")
    else:
        print("✗ Performance test FAILED (<50 Hz)")

def main():
    """Run all tests"""
    print("=== RoboMaster S1 EKF Test Suite ===")
    
    # Run unit tests
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestEKFCore))
    suite.addTests(loader.loadTestsFromTestCase(TestDataLogger))
    suite.addTests(loader.loadTestsFromTestCase(TestSensorFusion))
    suite.addTests(loader.loadTestsFromTestCase(TestSystemIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Run performance tests
    run_performance_test()
    
    # Summary
    print(f"\\n=== Test Summary ===")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.failures:
        print("\\nFailures:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print("\\nErrors:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    success = len(result.failures) == 0 and len(result.errors) == 0
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
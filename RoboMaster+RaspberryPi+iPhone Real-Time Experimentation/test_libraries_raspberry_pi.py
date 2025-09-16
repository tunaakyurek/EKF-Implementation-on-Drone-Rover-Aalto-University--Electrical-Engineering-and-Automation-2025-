#!/usr/bin/env python3
"""
RoboMaster S1 EKF Project - Library Test Script for Raspberry Pi
Tests all necessary libraries and dependencies without requiring hardware connections.

This script verifies that all required libraries for the EKF implementation
are properly installed and functional on the Raspberry Pi.

Usage:
    python3 test_libraries_raspberry_pi.py

Author: RoboMaster S1 EKF Project
Version: 1.0
"""

import sys
import time
import platform
import subprocess
import importlib.util

# ANSI color codes for better output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'

def print_header():
    """Print the test header"""
    print(f"{Colors.BOLD}{Colors.BLUE}")
    print("=" * 60)
    print("RoboMaster S1 EKF Project - Library Test Script")
    print("Raspberry Pi Compatibility Check")
    print("=" * 60)
    print(f"{Colors.END}")

def print_section(title):
    """Print a section header"""
    print(f"\n{Colors.BOLD}{Colors.YELLOW}{title}{Colors.END}")
    print("-" * len(title))

def test_system_info():
    """Test and display system information"""
    print_section("System Information")
    
    try:
        print(f"Python Version: {sys.version}")
        print(f"Platform: {platform.platform()}")
        print(f"Architecture: {platform.machine()}")
        print(f"Processor: {platform.processor()}")
        
        # Check if we're on Raspberry Pi
        if platform.machine().startswith('arm'):
            print(f"{Colors.GREEN}✓ Detected ARM architecture (likely Raspberry Pi){Colors.END}")
        else:
            print(f"{Colors.YELLOW}⚠ Warning: Not on ARM architecture{Colors.END}")
            
    except Exception as e:
        print(f"{Colors.RED}✗ Error getting system info: {e}{Colors.END}")

def test_basic_libraries():
    """Test basic Python libraries"""
    print_section("Basic Python Libraries")
    
    basic_libs = [
        'os', 'sys', 'time', 'math', 'json', 'datetime',
        'threading', 'queue', 'socket', 'struct'
    ]
    
    for lib in basic_libs:
        try:
            importlib.import_module(lib)
            print(f"{Colors.GREEN}✓ {lib}{Colors.END}")
        except ImportError as e:
            print(f"{Colors.RED}✗ {lib}: {e}{Colors.END}")

def test_numpy():
    """Test NumPy installation and functionality"""
    print_section("NumPy Library Test")
    
    try:
        import numpy as np
        print(f"{Colors.GREEN}✓ NumPy imported successfully{Colors.END}")
        print(f"  Version: {np.__version__}")
        
        # Test basic NumPy operations
        print("  Testing basic operations...")
        
        # Create test arrays
        a = np.array([[1, 2], [3, 4]])
        b = np.array([[5, 6], [7, 8]])
        
        # Test matrix operations
        c = np.dot(a, b)
        d = np.linalg.inv(a)
        e = np.linalg.det(a)
        
        print(f"  Matrix multiplication: ✓")
        print(f"  Matrix inversion: ✓")
        print(f"  Determinant calculation: ✓")
        
        # Test EKF-specific operations
        print("  Testing EKF-specific operations...")
        
        # 12D state vector as per formulary
        x = np.zeros(12)
        P = np.eye(12)
        F = np.eye(12)
        Q = np.eye(12) * 0.1
        
        # Test prediction step
        x_pred = F @ x
        P_pred = F @ P @ F.T + Q
        
        print(f"  State prediction: ✓")
        print(f"  Covariance prediction: ✓")
        
        # Test Kalman update
        H = np.eye(6, 12)
        R = np.eye(6) * 0.5
        z = np.zeros(6)
        
        y = z - H @ x_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        print(f"  Kalman gain calculation: ✓")
        print(f"  Innovation calculation: ✓")
        
        print(f"{Colors.GREEN}✓ All NumPy EKF operations successful{Colors.END}")
        
    except ImportError as e:
        print(f"{Colors.RED}✗ NumPy not installed: {e}{Colors.END}")
        print("  Install with: sudo apt-get install python3-numpy")
        return False
    except Exception as e:
        print(f"{Colors.RED}✗ NumPy test failed: {e}{Colors.END}")
        return False
    
    return True

def test_matplotlib():
    """Test Matplotlib for visualization"""
    print_section("Matplotlib Library Test")
    
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        print(f"{Colors.GREEN}✓ Matplotlib imported successfully{Colors.END}")
        print(f"  Version: {matplotlib.__version__}")
        
        # Test basic plotting (without display)
        matplotlib.use('Agg')  # Use non-interactive backend
        
        # Create a simple plot
        x = [1, 2, 3, 4, 5]
        y = [1, 4, 9, 16, 25]
        
        plt.figure()
        plt.plot(x, y)
        plt.title('Test Plot')
        plt.xlabel('X')
        plt.ylabel('Y')
        
        # Save to file instead of showing
        plt.savefig('/tmp/test_plot.png')
        plt.close()
        
        print(f"  Plot creation: ✓")
        print(f"  File saving: ✓")
        print(f"{Colors.GREEN}✓ Matplotlib test successful{Colors.END}")
        
    except ImportError as e:
        print(f"{Colors.RED}✗ Matplotlib not installed: {e}{Colors.END}")
        print("  Install with: sudo apt-get install python3-matplotlib")
        return False
    except Exception as e:
        print(f"{Colors.RED}✗ Matplotlib test failed: {e}{Colors.END}")
        return False
    
    return True

def test_zmq():
    """Test ZeroMQ for communication"""
    print_section("ZeroMQ Library Test")
    
    try:
        import zmq
        print(f"{Colors.GREEN}✓ ZeroMQ imported successfully{Colors.END}")
        print(f"  Version: {zmq.__version__}")
        
        # Test basic ZMQ operations
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        
        print(f"  Context creation: ✓")
        print(f"  Socket creation: ✓")
        
        # Clean up
        socket.close()
        context.term()
        
        print(f"  Socket cleanup: ✓")
        print(f"{Colors.GREEN}✓ ZeroMQ test successful{Colors.END}")
        
    except ImportError as e:
        print(f"{Colors.RED}✗ ZeroMQ not installed: {e}{Colors.END}")
        print("  Install with: pip3 install pyzmq")
        return False
    except Exception as e:
        print(f"{Colors.RED}✗ ZeroMQ test failed: {e}{Colors.END}")
        return False
    
    return True

def test_serial():
    """Test PySerial for serial communication"""
    print_section("PySerial Library Test")
    
    try:
        import serial
        print(f"{Colors.GREEN}✓ PySerial imported successfully{Colors.END}")
        print(f"  Version: {serial.__version__}")
        
        # List available ports
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        
        print(f"  Available serial ports: {len(ports)}")
        for port in ports:
            print(f"    - {port.device}: {port.description}")
        
        print(f"{Colors.GREEN}✓ PySerial test successful{Colors.END}")
        
    except ImportError as e:
        print(f"{Colors.RED}✗ PySerial not installed: {e}{Colors.END}")
        print("  Install with: pip3 install pyserial")
        return False
    except Exception as e:
        print(f"{Colors.RED}✗ PySerial test failed: {e}{Colors.END}")
        return False
    
    return True

def test_psutil():
    """Test psutil for system monitoring"""
    print_section("psutil Library Test")
    
    try:
        import psutil
        print(f"{Colors.GREEN}✓ psutil imported successfully{Colors.END}")
        print(f"  Version: {psutil.__version__}")
        
        # Test system monitoring
        cpu_percent = psutil.cpu_percent(interval=1)
        memory = psutil.virtual_memory()
        
        print(f"  CPU Usage: {cpu_percent}%")
        print(f"  Memory Usage: {memory.percent}%")
        print(f"  Available Memory: {memory.available / (1024**3):.2f} GB")
        
        print(f"{Colors.GREEN}✓ psutil test successful{Colors.END}")
        
    except ImportError as e:
        print(f"{Colors.RED}✗ psutil not installed: {e}{Colors.END}")
        print("  Install with: pip3 install psutil")
        return False
    except Exception as e:
        print(f"{Colors.RED}✗ psutil test failed: {e}{Colors.END}")
        return False
    
    return True

def test_ekf_simulation():
    """Test EKF simulation without hardware"""
    print_section("EKF Simulation Test")
    
    try:
        import numpy as np
        
        # Simulate EKF state
        print("  Simulating 12D EKF state...")
        
        # State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        x = np.zeros(12)
        P = np.eye(12)
        
        # Simulate IMU data
        accel = np.array([0.1, 0.2, 9.81])  # m/s²
        gyro = np.array([0.01, 0.02, 0.03])  # rad/s
        
        # Simulate chassis data
        chassis_pos = np.array([1.0, 2.0, 0.1])  # m, m, rad
        
        print(f"  State vector: {x.shape}")
        print(f"  Covariance matrix: {P.shape}")
        print(f"  IMU data: accel={accel}, gyro={gyro}")
        print(f"  Chassis data: pos={chassis_pos}")
        
        # Simulate prediction step
        dt = 0.02  # 50 Hz
        F = np.eye(12)
        F[0:3, 3:6] = np.eye(3) * dt  # Position from velocity
        F[6:9, 9:12] = np.eye(3) * dt  # Orientation from angular velocity
        
        x_pred = F @ x
        P_pred = F @ P @ F.T + np.eye(12) * 0.1 * dt
        
        print(f"  Prediction step: ✓")
        
        # Simulate update step
        H = np.zeros((6, 12))
        H[0:3, 0:3] = np.eye(3)  # Position measurement
        H[3:6, 6:9] = np.eye(3)  # Orientation measurement
        
        z = np.concatenate([chassis_pos[0:2], [chassis_pos[2]], gyro])
        R = np.eye(6) * 0.5
        
        y = z - H @ x_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        x_update = x_pred + K @ y
        P_update = (np.eye(12) - K @ H) @ P_pred @ (np.eye(12) - K @ H).T + K @ R @ K.T
        
        print(f"  Update step: ✓")
        print(f"  Final state: {x_update.shape}")
        print(f"  Final covariance: {P_update.shape}")
        
        print(f"{Colors.GREEN}✓ EKF simulation successful{Colors.END}")
        
    except Exception as e:
        print(f"{Colors.RED}✗ EKF simulation failed: {e}{Colors.END}")
        return False
    
    return True

def test_performance():
    """Test computational performance"""
    print_section("Performance Test")
    
    try:
        import numpy as np
        import time
        
        print("  Testing matrix operations performance...")
        
        # Test matrix multiplication performance
        start_time = time.time()
        
        for _ in range(1000):
            A = np.random.rand(12, 12)
            B = np.random.rand(12, 12)
            C = A @ B
        
        elapsed = time.time() - start_time
        print(f"  1000 12x12 matrix multiplications: {elapsed:.3f}s")
        
        if elapsed < 1.0:
            print(f"{Colors.GREEN}✓ Performance: Excellent{Colors.END}")
        elif elapsed < 5.0:
            print(f"{Colors.YELLOW}⚠ Performance: Good{Colors.END}")
        else:
            print(f"{Colors.RED}✗ Performance: Poor{Colors.END}")
        
        # Test EKF update rate
        print("  Testing EKF update rate...")
        
        start_time = time.time()
        iterations = 100
        
        for _ in range(iterations):
            # Simulate one EKF update
            x = np.zeros(12)
            P = np.eye(12)
            F = np.eye(12)
            Q = np.eye(12) * 0.1
            H = np.eye(6, 12)
            R = np.eye(6) * 0.5
            z = np.zeros(6)
            
            # Prediction
            x_pred = F @ x
            P_pred = F @ P @ F.T + Q
            
            # Update
            y = z - H @ x_pred
            S = H @ P_pred @ H.T + R
            K = P_pred @ H.T @ np.linalg.inv(S)
            x_update = x_pred + K @ y
        
        elapsed = time.time() - start_time
        rate = iterations / elapsed
        
        print(f"  EKF update rate: {rate:.1f} Hz")
        
        if rate >= 50:
            print(f"{Colors.GREEN}✓ Update rate: Excellent (≥50 Hz){Colors.END}")
        elif rate >= 20:
            print(f"{Colors.YELLOW}⚠ Update rate: Good (≥20 Hz){Colors.END}")
        else:
            print(f"{Colors.RED}✗ Update rate: Poor (<20 Hz){Colors.END}")
        
    except Exception as e:
        print(f"{Colors.RED}✗ Performance test failed: {e}{Colors.END}")
        return False
    
    return True

def install_missing_packages():
    """Provide installation commands for missing packages"""
    print_section("Installation Commands")
    
    print("If any tests failed, install missing packages with:")
    print()
    print("System packages:")
    print("  sudo apt-get update")
    print("  sudo apt-get install python3-numpy python3-matplotlib")
    print()
    print("Python packages:")
    print("  pip3 install pyzmq pyserial psutil")
    print()
    print("For development:")
    print("  pip3 install pytest coverage")

def main():
    """Main test function"""
    print_header()
    
    # Test system information
    test_system_info()
    
    # Test basic libraries
    test_basic_libraries()
    
    # Test required libraries
    numpy_ok = test_numpy()
    matplotlib_ok = test_matplotlib()
    zmq_ok = test_zmq()
    serial_ok = test_serial()
    psutil_ok = test_psutil()
    
    # Test EKF simulation
    ekf_ok = test_ekf_simulation()
    
    # Test performance
    perf_ok = test_performance()
    
    # Summary
    print_section("Test Summary")
    
    all_tests = [
        ("NumPy", numpy_ok),
        ("Matplotlib", matplotlib_ok),
        ("ZeroMQ", zmq_ok),
        ("PySerial", serial_ok),
        ("psutil", psutil_ok),
        ("EKF Simulation", ekf_ok),
        ("Performance", perf_ok)
    ]
    
    passed = sum(1 for _, ok in all_tests if ok)
    total = len(all_tests)
    
    print(f"Tests passed: {passed}/{total}")
    
    for name, ok in all_tests:
        status = f"{Colors.GREEN}✓ PASS{Colors.END}" if ok else f"{Colors.RED}✗ FAIL{Colors.END}"
        print(f"  {name}: {status}")
    
    if passed == total:
        print(f"\n{Colors.GREEN}{Colors.BOLD}🎉 All tests passed! Raspberry Pi is ready for RoboMaster S1 EKF project.{Colors.END}")
    else:
        print(f"\n{Colors.YELLOW}{Colors.BOLD}⚠ Some tests failed. Check installation commands below.{Colors.END}")
        install_missing_packages()
    
    print(f"\n{Colors.BLUE}Test completed at: {time.strftime('%Y-%m-%d %H:%M:%S')}{Colors.END}")

if __name__ == "__main__":
    main()

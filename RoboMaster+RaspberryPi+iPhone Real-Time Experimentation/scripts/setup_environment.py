"""
Environment setup script for RoboMaster S1 EKF system
"""

import os
import sys
import subprocess
import platform

def check_python_version():
    """Check if Python version is compatible"""
    print("Checking Python version...")
    
    if sys.version_info < (3, 7):
        print("ERROR: Python 3.7 or higher is required")
        return False
    
    print(f"Python {sys.version} - OK")
    return True

def install_requirements():
    """Install required packages"""
    print("Installing required packages...")
    
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])
        print("Requirements installed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"ERROR: Failed to install requirements: {e}")
        return False

def setup_directories():
    """Create necessary directories"""
    print("Setting up directories...")
    
    directories = [
        "logs",
        "data",
        "config"
    ]
    
    for directory in directories:
        os.makedirs(directory, exist_ok=True)
        print(f"Created directory: {directory}")
    
    return True

def check_robomaster_connection():
    """Check if RoboMaster S1 can be detected"""
    print("Checking RoboMaster S1 connection...")
    
    try:
        import robomaster
        from robomaster import robot
        
        # Try to create robot instance
        rm_robot = robot.Robot()
        print("RoboMaster SDK imported successfully")
        
        # Note: Actual connection test would require the robot to be available
        print("Note: Connect your RoboMaster S1 and ensure it's in SDK mode")
        
        return True
    except ImportError:
        print("ERROR: RoboMaster SDK not installed or not working")
        return False
    except Exception as e:
        print(f"WARNING: Could not test RoboMaster connection: {e}")
        return True

def setup_network_config():
    """Setup network configuration"""
    print("Setting up network configuration...")
    
    if platform.system() == "Linux":  # Likely Raspberry Pi
        print("Detected Linux system (Raspberry Pi)")
        
        # Get current IP address
        try:
            import socket
            hostname = socket.gethostname()
            local_ip = socket.gethostbyname(hostname)
            print(f"Current IP address: {local_ip}")
        except:
            print("Could not determine IP address")
        
        print("Make sure your Ground PC can reach this IP address")
    
    return True

def run_system_test():
    """Run basic system test"""
    print("Running system test...")
    
    try:
        # Test imports
        sys.path.append('src')
        
        from ekf.ekf_core import ExtendedKalmanFilter, SensorData
        from robomaster_interface.robomaster_client import RoboMasterClient
        from data_collection.data_logger import DataLogger
        from communication.network_client import NetworkClient
        from visualization.real_time_plotter import RealTimePlotter
        
        print("All modules imported successfully")
        
        # Test EKF creation
        ekf = ExtendedKalmanFilter()
        print("EKF initialization - OK")
        
        # Test data logger
        logger = DataLogger()
        logger.close()
        print("Data logger - OK")
        
        print("System test completed successfully")
        return True
        
    except Exception as e:
        print(f"ERROR: System test failed: {e}")
        return False

def main():
    """Main setup function"""
    print("=== RoboMaster S1 EKF System Setup ===")
    
    success = True
    
    # Check Python version
    if not check_python_version():
        success = False
    
    # Setup directories
    if not setup_directories():
        success = False
    
    # Install requirements
    if not install_requirements():
        success = False
    
    # Check RoboMaster connection
    if not check_robomaster_connection():
        success = False
    
    # Setup network
    if not setup_network_config():
        success = False
    
    # Run system test
    if not run_system_test():
        success = False
    
    if success:
        print("\\n=== Setup completed successfully! ===")
        print("\\nNext steps:")
        print("1. Connect your RoboMaster S1 to the same network")
        print("2. Update config/network_config.json with your Ground PC IP")
        print("3. Run 'python src/main.py' on Raspberry Pi")
        print("4. Run 'python src/ground_station.py' on Ground PC")
    else:
        print("\\n=== Setup completed with errors ===")
        print("Please fix the errors above before running the system")
    
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
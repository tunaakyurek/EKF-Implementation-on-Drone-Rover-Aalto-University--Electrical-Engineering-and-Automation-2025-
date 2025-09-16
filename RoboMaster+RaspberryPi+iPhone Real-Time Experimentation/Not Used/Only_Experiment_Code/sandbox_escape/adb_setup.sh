#!/bin/bash

# RoboMaster S1 ADB Setup Script
# This script helps set up ADB access to the RoboMaster S1
# for direct code execution experiments.

echo "=== RoboMaster S1 ADB Setup ==="
echo "This script will help you set up ADB access to your RoboMaster S1"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Windows (Git Bash/WSL) or Linux
check_platform() {
    print_info "Detecting platform..."
    
    if [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]] || [[ "$OS" == "Windows_NT" ]]; then
        PLATFORM="windows"
        ADB_CMD="adb.exe"
        print_info "Platform: Windows"
    else
        PLATFORM="linux"
        ADB_CMD="adb"
        print_info "Platform: Linux/Unix"
    fi
}

# Check if ADB is installed
check_adb() {
    print_info "Checking for ADB installation..."
    
    if command -v $ADB_CMD &> /dev/null; then
        ADB_VERSION=$($ADB_CMD version | head -n1)
        print_success "ADB found: $ADB_VERSION"
        return 0
    else
        print_error "ADB not found in PATH"
        echo ""
        echo "Please install Android SDK Platform Tools:"
        echo "https://developer.android.com/studio/releases/platform-tools"
        echo ""
        if [[ $PLATFORM == "windows" ]]; then
            echo "For Windows:"
            echo "1. Download platform-tools-latest-windows.zip"
            echo "2. Extract to C:\\platform-tools"
            echo "3. Add C:\\platform-tools to your PATH"
            echo "4. Or run this script from the platform-tools directory"
        else
            echo "For Linux:"
            echo "sudo apt-get install android-tools-adb  # Ubuntu/Debian"
            echo "sudo yum install android-tools         # CentOS/RHEL"
            echo "brew install android-platform-tools    # macOS"
        fi
        return 1
    fi
}

# Check USB connection
check_usb() {
    print_info "Checking USB connection..."
    
    if [[ $PLATFORM == "windows" ]]; then
        # On Windows, check if any USB device is connected
        # This is a simplified check
        print_info "Make sure RoboMaster S1 is connected via USB"
    else
        # On Linux, check for USB devices
        if command -v lsusb &> /dev/null; then
            USB_DEVICES=$(lsusb | grep -i "dji\|robomaster" | wc -l)
            if [[ $USB_DEVICES -gt 0 ]]; then
                print_success "RoboMaster USB device detected"
            else
                print_warning "No RoboMaster USB device detected"
                print_info "Make sure the S1 is powered on and connected"
            fi
        fi
    fi
}

# Provide sandbox escape code
show_escape_code() {
    print_info "Generating sandbox escape code..."
    echo ""
    echo "==== COPY THIS CODE TO ROBOMASTER APP LAB ===="
    echo ""
    cat << 'EOF'
def root_me(module):
    __import__ = rm_define.__dict__['__builtins__']['__import__']
    return __import__(module, globals(), locals(), [], 0)

builtins = root_me('builtins')
subprocess = root_me('subprocess')
proc = subprocess.Popen('/system/bin/adb_en.sh', shell=True, 
                       executable='/system/bin/sh', 
                       stdout=subprocess.PIPE, 
                       stderr=subprocess.PIPE)
EOF
    echo ""
    echo "=============================================="
    echo ""
    print_warning "IMPORTANT STEPS:"
    echo "1. Open RoboMaster app on your phone/tablet"
    echo "2. Connect to your RoboMaster S1 (WiFi or router)"
    echo "3. Go to Lab → Python"
    echo "4. Create a new Python project"
    echo "5. Copy and paste the code above"
    echo "6. Run the code (should display 'Execution Complete')"
    echo "7. Keep the app open and return to this script"
    echo ""
}

# Test ADB connection
test_adb_connection() {
    print_info "Testing ADB connection..."
    
    # Kill any existing ADB server
    $ADB_CMD kill-server 2>/dev/null
    
    # Start ADB server
    $ADB_CMD start-server
    
    # Check for devices
    echo ""
    print_info "Scanning for devices..."
    DEVICES=$($ADB_CMD devices | grep -v "List of devices" | grep "device$" | wc -l)
    
    if [[ $DEVICES -gt 0 ]]; then
        print_success "RoboMaster S1 detected via ADB!"
        echo ""
        echo "Connected devices:"
        $ADB_CMD devices
        echo ""
        return 0
    else
        print_error "No ADB devices found"
        echo ""
        echo "Troubleshooting:"
        echo "1. Make sure you ran the escape code in the RoboMaster app"
        echo "2. Keep the RoboMaster app open"
        echo "3. Try a different USB cable"
        echo "4. Check USB cable supports data transfer (not just charging)"
        echo "5. Try reconnecting the USB cable"
        echo ""
        return 1
    fi
}

# Get system info from S1
get_system_info() {
    print_info "Collecting system information from RoboMaster S1..."
    
    echo ""
    echo "==== SYSTEM INFORMATION ===="
    
    # Basic system info
    echo ""
    echo "--- Device Info ---"
    $ADB_CMD shell "getprop ro.product.model" 2>/dev/null
    $ADB_CMD shell "getprop ro.build.version.release" 2>/dev/null
    
    # CPU info
    echo ""
    echo "--- CPU Info ---"
    $ADB_CMD shell "cat /proc/cpuinfo | grep -E '(processor|model name|Hardware)'" 2>/dev/null
    
    # Memory info
    echo ""
    echo "--- Memory Info ---"
    $ADB_CMD shell "cat /proc/meminfo | head -5" 2>/dev/null
    
    # Python info
    echo ""
    echo "--- Python Info ---"
    $ADB_CMD shell "ls -la /data/python_files/bin/" 2>/dev/null
    $ADB_CMD shell "/data/python_files/bin/python --version" 2>/dev/null
    
    # Available space
    echo ""
    echo "--- Storage Info ---"
    $ADB_CMD shell "df -h /data" 2>/dev/null
    
    echo ""
    echo "=========================="
}

# Upload test file
upload_test_file() {
    print_info "Creating and uploading test file..."
    
    # Create a simple test script
    cat > test_s1.py << 'EOF'
#!/usr/bin/env python
"""
Test script for RoboMaster S1 direct execution
"""

import sys
import time
import os

print("=== RoboMaster S1 Test Script ===")
print(f"Python version: {sys.version}")
print(f"Current directory: {os.getcwd()}")
print(f"Script location: {__file__}")

# Test basic functionality
print("\nTesting basic Python features...")
for i in range(5):
    print(f"Count: {i+1}")
    time.sleep(0.5)

print("\nTest completed successfully!")
EOF

    # Upload the file
    if $ADB_CMD push test_s1.py /data/script/file/ 2>/dev/null; then
        print_success "Test file uploaded to /data/script/file/test_s1.py"
        
        # Make it executable
        $ADB_CMD shell "chmod 755 /data/script/file/test_s1.py" 2>/dev/null
        
        # Execute it
        print_info "Executing test script..."
        echo ""
        $ADB_CMD shell "/data/python_files/bin/python /data/script/file/test_s1.py"
        echo ""
        
        print_success "Test script executed successfully!"
        
        # Clean up
        rm -f test_s1.py
        
    else
        print_error "Failed to upload test file"
        return 1
    fi
}

# Main setup process
main() {
    echo ""
    print_info "Starting RoboMaster S1 ADB setup process..."
    echo ""
    
    # Step 1: Check platform and ADB
    check_platform
    if ! check_adb; then
        exit 1
    fi
    
    # Step 2: Check USB connection
    check_usb
    
    # Step 3: Show escape code
    show_escape_code
    
    # Step 4: Wait for user to run escape code
    echo -n "Press Enter after you have run the escape code in the RoboMaster app..."
    read
    
    # Step 5: Test ADB connection
    if ! test_adb_connection; then
        print_error "ADB connection failed. Please check the troubleshooting steps."
        exit 1
    fi
    
    # Step 6: Get system info
    get_system_info
    
    # Step 7: Upload and test execution
    upload_test_file
    
    # Success message
    echo ""
    print_success "ADB setup completed successfully!"
    echo ""
    print_info "You can now:"
    echo "  - Upload files: $ADB_CMD push local_file.py /data/script/file/"
    echo "  - Execute Python: $ADB_CMD shell \"/data/python_files/bin/python /data/script/file/script.py\""
    echo "  - Access shell: $ADB_CMD shell"
    echo "  - List files: $ADB_CMD shell \"ls /data/script/file/\""
    echo ""
    print_warning "Remember to keep the RoboMaster app open to maintain ADB access!"
}

# Run main function
main "$@"
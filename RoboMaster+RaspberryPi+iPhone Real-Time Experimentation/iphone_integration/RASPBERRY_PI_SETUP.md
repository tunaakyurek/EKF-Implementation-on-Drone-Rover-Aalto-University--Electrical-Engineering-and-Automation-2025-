# Raspberry Pi Setup Guide for iPhone Integration

## Quick Setup

### Option 1: Install via pip (Recommended)
```bash
# Install iPhone integration requirements
pip install -r iphone_integration/requirements_raspberry_pi.txt

# Verify installation
python iphone_integration/verify_integration.py
```

### Option 2: Install system packages (Alternative)
```bash
# Update package list
sudo apt update

# Install system packages (faster on Pi)
sudo apt install python3-numpy python3-scipy python3-matplotlib python3-pandas

# Verify installation
python iphone_integration/verify_integration.py
```

## Available Versions on Raspberry Pi (piwheels)

Based on the error you encountered, here are the numpy versions available on piwheels:
- `1.24.0`, `1.24.2` (recommended range: 1.24.x)
- `1.25.0`, `1.25.1`, `1.25.2` 
- `1.26.0`, `1.26.1`, `1.26.2`, `1.26.3`, `1.26.4`

## Troubleshooting

### If you get version conflicts:
```bash
# Try specific versions that are available
pip install numpy==1.24.2 scipy matplotlib pandas

# Or use the flexible requirements
pip install -r iphone_integration/requirements_raspberry_pi.txt
```

### For headless Raspberry Pi (no display):
Add this to your Python scripts before importing matplotlib:
```python
import matplotlib
matplotlib.use('Agg')  # Use non-GUI backend
import matplotlib.pyplot as plt
```

## Verification

After installation, run:
```bash
cd RoboMaster_S1_EKF
python iphone_integration/verify_integration.py
```

Expected output:
```
✅ All verifications passed! The system is properly integrated.
```

## Performance Notes

- System packages (`apt install`) are often faster on Raspberry Pi
- piwheels provides pre-compiled ARM wheels (faster than pip compilation)
- The iPhone integration system is optimized for real-time performance on Pi 4

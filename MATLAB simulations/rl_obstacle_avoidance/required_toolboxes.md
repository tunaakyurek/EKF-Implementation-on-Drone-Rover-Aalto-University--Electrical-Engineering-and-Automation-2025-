# Required MATLAB Toolboxes and Dependencies

## Overview
This document outlines all the MATLAB toolboxes and dependencies required to run the enhanced 9-DOF Drone EKF system with RL-based obstacle avoidance capabilities.

## Core MATLAB Version
- **Minimum MATLAB Version**: R2020a or later
- **Recommended Version**: R2022b or later for optimal performance

## Required MATLAB Toolboxes

### 1. Deep Learning Toolbox
**Purpose**: Neural network implementation for the RL agent
- Used for: Actor-critic networks, training algorithms, GPU acceleration
- Key functions: `layerGraph`, `trainNetwork`, `predict`, `dlnetwork`
- **License Type**: Commercial
- **Alternative**: For basic functionality, custom implementation possible but not recommended

### 2. Reinforcement Learning Toolbox  
**Purpose**: RL algorithms and environment interfaces
- Used for: Agent training, experience replay, exploration strategies
- Key functions: `rlDDPGAgent`, `rlEnvironment`, `train`, `rlExperienceBuffer`
- **License Type**: Commercial
- **Note**: Can be partially replaced with custom implementation using the provided agent code

### 3. Aerospace Toolbox
**Purpose**: Coordinate transformations and aerospace-specific functions
- Used for: Attitude representations, coordinate frame conversions
- Key functions: `angle2dcm`, `dcm2angle`, coordinate frame utilities
- **License Type**: Commercial
- **Alternative**: Manual implementation of rotation matrices (provided in `rotation_matrix.m`)

### 4. Control System Toolbox
**Purpose**: Control system design and analysis
- Used for: PID controllers, system analysis, stability checks
- Key functions: `pid`, `step`, `bode`, `lsim`
- **License Type**: Commercial
- **Alternative**: Custom controller implementation (basic versions provided)

### 5. Signal Processing Toolbox
**Purpose**: Signal filtering and processing
- Used for: Sensor data filtering, noise generation, spectral analysis
- Key functions: `filter`, `butter`, `fft`, `filtfilt`
- **License Type**: Commercial
- **Alternative**: Basic filtering can be implemented manually

### 6. Image Processing Toolbox
**Purpose**: 2D/3D map processing and visualization
- Used for: Occupancy grid processing, morphological operations
- Key functions: `imgaussfilt`, `imdilate`, `bwmorph`, `regionprops`
- **License Type**: Commercial
- **Alternative**: Basic operations can be implemented with standard MATLAB functions

### 7. Statistics and Machine Learning Toolbox
**Purpose**: Statistical analysis and machine learning utilities
- Used for: Random sampling, statistical tests, clustering
- Key functions: `randn`, `normrnd`, `kmeans`, statistical distributions
- **License Type**: Commercial
- **Alternative**: Basic statistical functions available in base MATLAB

### 8. Parallel Computing Toolbox (Optional but Recommended)
**Purpose**: Accelerated training and parallel simulation
- Used for: GPU acceleration, parallel training loops
- Key functions: `parfor`, `gpuArray`, `gather`
- **License Type**: Commercial
- **Note**: Significantly improves training speed but not required

## Optional Advanced Toolboxes

### 9. Navigation Toolbox
**Purpose**: Advanced navigation algorithms
- Used for: Path planning, localization algorithms
- Key functions: `plannerRRT`, `controllerPurePursuit`
- **License Type**: Commercial
- **Note**: Custom implementations provided for basic functionality

### 10. Computer Vision Toolbox
**Purpose**: Advanced perception and visual processing
- Used for: Feature detection, camera calibration, 3D vision
- **License Type**: Commercial
- **Note**: Only needed for visual sensor simulation

## Hardware Requirements

### Minimum Requirements
- **RAM**: 8 GB (16 GB recommended)
- **Storage**: 5 GB free space for toolboxes and data
- **CPU**: Multi-core processor (Intel i5 or equivalent)

### Recommended for RL Training
- **RAM**: 32 GB or more
- **GPU**: NVIDIA GPU with CUDA support (GTX 1060 or better)
- **Storage**: SSD with 20 GB free space
- **CPU**: Intel i7 or AMD Ryzen 7 (8+ cores)

## Installation Instructions

### 1. MATLAB Installation
```matlab
% Check MATLAB version
version

% Check installed toolboxes
ver
```

### 2. Toolbox Installation
Through MATLAB Add-On Explorer:
1. Open MATLAB
2. Go to Home tab → Add-Ons → Get Add-Ons
3. Search for and install required toolboxes
4. Restart MATLAB after installation

### 3. License Verification
```matlab
% Check toolbox licenses
license('test', 'Deep_Learning_Toolbox')
license('test', 'Reinforcement_Learning_Toolbox')
license('test', 'Aerospace_Toolbox')
license('test', 'Control_Toolbox')
license('test', 'Signal_Toolbox')
license('test', 'Image_Toolbox')
```

## Dependencies and Setup

### 1. Project Structure
```
EKF_Drone_9State/
├── rl_obstacle_avoidance/      # New RL system
├── noanim_benchmarks/          # Existing benchmarks
├── RoboMaster_Sim/            # Rover simulation
├── parameters.m               # Main parameters
├── main_random.m             # Main simulation
└── README.md                 # Documentation
```

### 2. Path Setup
```matlab
% Add all necessary paths
addpath(genpath('.'));
addpath('rl_obstacle_avoidance');
addpath('noanim_benchmarks/filters');
```

### 3. Parameter Configuration
```matlab
% Load parameters
params = rl_parameters();

% Verify configuration
fprintf('RL system configured for %s\n', params.rl.obstacle_scenario);
```

## Troubleshooting Common Issues

### 1. Missing Toolbox Functions
**Problem**: Function not found errors
**Solution**: 
```matlab
% Check which toolbox contains the function
which function_name -all

% Install missing toolbox or use alternative implementation
```

### 2. GPU Memory Issues
**Problem**: Out of GPU memory during training
**Solution**:
```matlab
% Clear GPU memory
gpuDevice(1);
reset(gpuDevice);

% Reduce batch size in parameters
params.rl.batch_size = 64;  % Reduce from default 128
```

### 3. Training Convergence Issues
**Problem**: RL agent not learning effectively
**Solution**:
```matlab
% Adjust learning rates
params.rl.lr_actor = 1e-5;    % Reduce actor learning rate
params.rl.lr_critic = 5e-4;   % Reduce critic learning rate

% Enable curriculum learning
params.rl.use_curriculum = true;
```

## Performance Optimization

### 1. GPU Acceleration
```matlab
% Enable GPU training (if available)
params.rl.hardware.use_gpu = true;

% Check GPU availability
if gpuDeviceCount > 0
    fprintf('GPU available: %s\n', gpuDevice().Name);
else
    fprintf('No GPU detected - using CPU\n');
end
```

### 2. Parallel Training
```matlab
% Enable parallel processing
params.rl.hardware.parallel_envs = 4;  % Number of parallel environments
params.rl.hardware.num_workers = 8;    % Number of worker threads
```

### 3. Memory Management
```matlab
% Monitor memory usage
memory

% Clear unnecessary variables
clear large_variables

% Use single precision for networks (reduces memory)
params.rl.use_single_precision = true;
```

## Alternative Solutions for Missing Toolboxes

### 1. Without Deep Learning Toolbox
Use simplified neural networks:
```matlab
% Basic neural network implementation provided
% in rl_agent_basic.m (simplified version)
```

### 2. Without Reinforcement Learning Toolbox
Use custom RL implementation:
```matlab
% Complete DDPG implementation provided
% in rl_agent.m with custom replay buffer
```

### 3. Without Aerospace Toolbox
Use provided rotation utilities:
```matlab
% rotation_matrix.m provides all necessary
% coordinate transformations
```

## Verification Script

Run this script to verify your installation:

```matlab
%% Installation Verification Script
function verify_installation()
    fprintf('=== MATLAB Toolbox Verification ===\n\n');
    
    % Check MATLAB version
    v = version('-release');
    fprintf('MATLAB Version: %s\n', v);
    
    % Required toolboxes
    required = {
        'Deep_Learning_Toolbox', 'Deep Learning Toolbox';
        'Reinforcement_Learning_Toolbox', 'Reinforcement Learning Toolbox';
        'Aerospace_Toolbox', 'Aerospace Toolbox';
        'Control_Toolbox', 'Control System Toolbox';
        'Signal_Toolbox', 'Signal Processing Toolbox';
        'Image_Toolbox', 'Image Processing Toolbox'
    };
    
    fprintf('\nToolbox Status:\n');
    for i = 1:size(required, 1)
        if license('test', required{i,1})
            fprintf('✓ %s: Available\n', required{i,2});
        else
            fprintf('✗ %s: Missing\n', required{i,2});
        end
    end
    
    % Optional toolboxes
    optional = {
        'Parallel_Computing_Toolbox', 'Parallel Computing Toolbox';
        'Navigation_Toolbox', 'Navigation Toolbox';
        'Computer_Vision_Toolbox', 'Computer Vision Toolbox'
    };
    
    fprintf('\nOptional Toolboxes:\n');
    for i = 1:size(optional, 1)
        if license('test', optional{i,1})
            fprintf('✓ %s: Available\n', optional{i,2});
        else
            fprintf('- %s: Not installed\n', optional{i,2});
        end
    end
    
    % GPU check
    fprintf('\nHardware:\n');
    if gpuDeviceCount > 0
        gpu = gpuDevice(1);
        fprintf('✓ GPU: %s (%.1f GB)\n', gpu.Name, gpu.AvailableMemory/1e9);
    else
        fprintf('- GPU: No CUDA-capable GPU detected\n');
    end
    
    % Memory check
    [user, sys] = memory;
    fprintf('✓ RAM: %.1f GB available\n', user.MemAvailableAllArrays/1e9);
    
    fprintf('\n=== Verification Complete ===\n');
end
```

## Support and Resources

### Official Documentation
- [MATLAB Deep Learning Toolbox](https://www.mathworks.com/products/deep-learning.html)
- [Reinforcement Learning Toolbox](https://www.mathworks.com/products/reinforcement-learning.html)
- [Aerospace Toolbox](https://www.mathworks.com/products/aerospace-toolbox.html)

### Community Resources
- MATLAB Central File Exchange
- MathWorks Community Forums
- GitHub repositories for alternative implementations

### Academic Licenses
- Students and academic institutions may be eligible for reduced-cost licenses
- Contact MathWorks academic sales for educational pricing

---

**Note**: This system can run with reduced functionality if some toolboxes are missing. Core EKF simulation requires only base MATLAB, while RL training benefits significantly from the Deep Learning and Reinforcement Learning Toolboxes.

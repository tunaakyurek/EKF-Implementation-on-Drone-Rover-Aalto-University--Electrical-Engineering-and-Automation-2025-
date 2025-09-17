# Smartphone-Based Sensor Fusion for Autonomous Vehicle State Estimation

## Abstract

This repository contains the comprehensive work conducted during an internship focused on developing advanced sensor fusion algorithms for autonomous vehicle state estimation using smartphone-based sensor systems. The research encompasses both theoretical simulation studies in MATLAB and practical real-time implementation using iPhone sensors integrated with RoboMaster S1 robotic platforms. The work demonstrates novel approaches to Extended Kalman Filter (EKF) design, Unscented Kalman Filter (UKF) implementation, and real-time sensor fusion for autonomous navigation applications.

## Research Objectives

The primary objectives of this research were to:

1. Develop and validate advanced sensor fusion algorithms for autonomous vehicle state estimation
2. Implement smartphone-based sensor integration systems for real-time navigation
3. Compare and optimize different filtering approaches (EKF vs UKF) for various operational scenarios
4. Create comprehensive simulation frameworks for algorithm validation and parameter optimization
5. Deploy real-time implementations on embedded systems with hardware-in-the-loop validation

## Project Structure

### 1. MATLAB Drone Simulations

The `MATLAB simulations/` directory contains comprehensive quadrotor simulation studies developed as the theoretical foundation for real-world drone implementation:

#### Core Simulation Framework
- **Complete 6-DOF Quadrotor Dynamics**: Full nonlinear quadrotor dynamics with proper aerodynamic modeling and thrust/torque relationships
- **Extended Kalman Filter Implementation**: Production-ready 9-DOF and 12-DOF EKF implementations following Särkkä's Bayesian filtering notation
- **Unscented Kalman Filter Development**: Advanced UKF algorithms with sigma-point transformations for comparative analysis
- **Multi-rate Sensor Fusion**: High-rate IMU (50-200 Hz) integrated with low-rate GPS (1-10 Hz) following standard INS/GNSS architecture

#### Advanced Algorithm Development
- **Parameter Optimization Studies**: Systematic optimization of process noise (Q) and measurement noise (R) matrices using various optimization techniques
- **Comparative EKF vs UKF Analysis**: Detailed performance comparisons under different flight scenarios (hover, waypoint tracking, aggressive maneuvers)
- **Obstacle Avoidance Integration**: Integration of state estimation with autonomous navigation and dynamic obstacle avoidance algorithms
- **Reinforcement Learning Enhancement**: Advanced RL-based approaches for adaptive parameter tuning and autonomous decision making

#### Simulation Validation Framework
- **Digital Twin Development**: High-fidelity simulation models matching planned hardware configurations (QAV250 and S500 platforms)
- **Sensor Noise Modeling**: Realistic IMU noise characteristics, GPS accuracy modeling, and magnetometer/barometer integration
- **Flight Scenario Testing**: Comprehensive test scenarios including hover stability, waypoint navigation, and aggressive maneuvering
- **Performance Metrics**: RMSE analysis, NEES consistency testing, and computational efficiency benchmarking

### 2. Real-Time Implementation

#### A. RoboMaster S1 Prototype System
The `RoboMaster+RaspberryPi+iPhone Real-Time Experimentation/` directory contains the initial prototype implementation:

- **Extended Kalman Filter Core**: Production-ready 8-DOF and 12-DOF EKF implementations optimized for real-time operation
- **iPhone Sensor Integration**: Complete sensor data acquisition and processing pipeline for iPhone IMU, GPS, and magnetometer data
- **RoboMaster S1 Interface**: Hardware abstraction layer for RoboMaster S1 robotic platform communication
- **Real-time Communication**: Network-based data streaming and visualization system
- **Autonomous Control**: Closed-loop autonomous navigation system with waypoint following and obstacle avoidance
- **Sensor Calibration**: Automatic bias estimation and sensor alignment procedures

#### B. Holybro Drone Implementation (QAV250 & S500)
Building upon the MATLAB simulations and RoboMaster prototype, the system was designed for deployment on professional drone platforms:

**Hardware Configuration:**
- **QAV250 Racing Drone**: Compact quadrotor platform for agile flight testing and indoor operations
- **S500 V2 Frame**: Larger platform for outdoor testing and payload integration
- **Pixhawk 6C Flight Controller**: Industry-standard autopilot with high-rate IMU sensors
- **M10 GPS Module**: Precise GNSS positioning with multi-constellation support
- **Raspberry Pi 4**: Onboard companion computer for real-time EKF processing
- **433 MHz Telemetry**: Long-range communication for ground station connectivity

**Software Architecture:**
- **ROS Noetic Framework**: Modular architecture with standardized topic interfaces
- **C++ Implementation**: High-performance EKF node optimized for embedded deployment
- **MAVROS Integration**: Seamless interface with Pixhawk autopilot via MAVLink protocol
- **Gazebo/PX4-SITL**: Hardware-in-the-loop simulation environment for validation
- **Asynchronous Processing**: Multi-rate sensor fusion with IMU-driven prediction and GPS correction

**Real-time EKF Design:**
- **State Vector**: 15-DOF comprehensive state [position, velocity, attitude, gyro bias, accel bias]
- **Prediction Model**: Nonlinear quadrotor dynamics with proper frame transformations (body/world)
- **Measurement Model**: GPS position updates with optional magnetometer/barometer integration
- **Multi-rate Processing**: 50-200 Hz IMU prediction with 1-10 Hz GPS correction
- **Särkkä Notation**: Consistent mathematical framework following Bayesian filtering principles

### 3. Documentation and Analysis

The `Resources - References/` directory contains:

- **Technical Reports**: Comprehensive analysis reports documenting MATLAB simulation results and real-world experimental validation
- **Mathematical Formulations**: Complete mathematical derivations following Särkkä's Bayesian filtering framework
- **Performance Analysis**: Detailed performance metrics, RMSE analysis, and consistency testing results
- **Academic References**: Curated collection of relevant research papers including UAV state estimation, sensor fusion, and quadrotor control literature

## Technical Contributions

### Algorithm Development

1. **Comprehensive EKF Framework**: Developed complete Extended Kalman Filter implementations from MATLAB simulation through embedded C++ deployment, following rigorous Bayesian filtering principles
2. **Multi-platform Sensor Fusion**: Created unified sensor fusion approaches spanning smartphone sensors (iPhone), robotic platforms (RoboMaster S1), and professional drone hardware (Pixhawk 6C)
3. **Multi-rate Processing Architecture**: Implemented efficient asynchronous processing for high-rate IMU (50-200 Hz) and low-rate GPS (1-10 Hz) sensor fusion
4. **Advanced Parameter Optimization**: Developed systematic approaches for Q/R matrix tuning using both analytical methods and empirical flight data

### Simulation-to-Reality Pipeline

1. **Digital Twin Development**: Created high-fidelity MATLAB simulations that accurately predict real-world drone behavior and EKF performance
2. **Hardware-in-the-Loop Validation**: Established seamless transition from Gazebo/PX4-SITL simulation to actual hardware deployment
3. **Cross-platform Verification**: Validated algorithms across multiple hardware platforms ensuring robustness and generalizability
4. **Systematic Testing Framework**: Developed comprehensive testing protocols from bench tests to controlled flight scenarios

### System Integration and Deployment

1. **Professional Drone Integration**: Successfully integrated custom EKF algorithms with industry-standard flight controllers (Pixhawk 6C) via MAVROS/MAVLink
2. **Real-time Embedded Implementation**: Optimized algorithms for deployment on resource-constrained embedded systems (Raspberry Pi 4)
3. **Modular Software Architecture**: Designed flexible ROS-based architecture enabling rapid prototyping and configuration changes
4. **Operational Safety Systems**: Implemented robust error handling, sensor validation, and fallback mechanisms for safe autonomous operation

### Performance Analysis and Validation

1. **Quantitative Performance Metrics**: Established comprehensive evaluation criteria including RMSE analysis, NEES consistency testing, and computational efficiency benchmarking
2. **Multi-scenario Validation**: Validated performance across diverse flight scenarios (hover, waypoint tracking, aggressive maneuvers, indoor/outdoor operations)
3. **Comparative Algorithm Analysis**: Conducted systematic EKF vs UKF performance comparisons under various operational conditions
4. **Production-ready Documentation**: Created detailed technical documentation following academic standards with reproducible results and clear methodology

## Key Results and Findings

### MATLAB Simulation Achievements
- **Algorithm Validation**: Comprehensive validation of EKF and UKF implementations under diverse flight scenarios with quantitative performance metrics
- **Parameter Optimization**: Systematic optimization of filter parameters achieving improved estimation accuracy and numerical stability
- **Comparative Analysis**: Detailed performance comparison between filtering approaches providing clear guidance for real-world implementation
- **Digital Twin Accuracy**: High-fidelity simulation models accurately predicting real-world drone behavior and sensor characteristics

### Real-time Implementation Success
- **Multi-platform Deployment**: Successful deployment across three distinct hardware platforms (iPhone sensors, RoboMaster S1, Holybro drones)
- **Real-time Performance**: Achieved stable real-time operation with 50-200 Hz IMU processing and multi-rate GPS fusion on embedded systems
- **Professional Integration**: Seamless integration with industry-standard flight controllers and communication protocols (MAVROS/MAVLink)
- **Operational Validation**: Demonstrated robust performance across indoor/outdoor environments and various flight scenarios

### Technical Innovation
- **Simulation-to-Reality Pipeline**: Established seamless workflow from MATLAB simulation through hardware-in-the-loop testing to flight validation
- **Adaptive Algorithm Design**: Developed algorithms capable of operating across diverse sensor configurations and platform constraints
- **Safety-critical Implementation**: Implemented robust error handling and validation systems suitable for autonomous flight operations
- **Academic-grade Documentation**: Created comprehensive technical documentation following rigorous academic standards with reproducible results

## Future Work and Applications

### Immediate Extensions
- **Advanced Filtering Techniques**: Implementation of ES-EKF/IEKF variants and adaptive Q/R matrix tuning
- **Additional Sensor Integration**: Incorporation of magnetometer, barometer, and visual-inertial odometry systems
- **Rauch-Tung-Striebel Smoothing**: Offline smoothing for post-flight analysis and improved trajectory estimation
- **Multi-vehicle Systems**: Extension to cooperative localization and formation flight applications

### Broader Applications
- **Commercial UAV Systems**: Professional drone applications for surveying, inspection, and autonomous delivery
- **Urban Air Mobility**: Advanced air taxi and eVTOL vehicle navigation systems
- **Autonomous Ground Vehicles**: Mobile robotics and self-driving vehicle state estimation
- **Smartphone Navigation**: Enhanced pedestrian and vehicle navigation systems using smartphone sensors
- **Industrial IoT**: Sensor fusion for industrial monitoring and automation applications

## References

[1] Särkkä, S. (2013). *Bayesian Filtering and Smoothing*. Cambridge University Press.

[2] Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

[3] Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation: Theory Algorithms and Software*. John Wiley & Sons.

[4] Grewal, M. S., & Andrews, A. P. (2014). *Kalman Filtering: Theory and Practice with MATLAB*. John Wiley & Sons.

[5] Julier, S. J., & Uhlmann, J. K. (2004). Unscented filtering and nonlinear estimation. *Proceedings of the IEEE*, 92(3), 401-422.

[6] Wan, E. A., & Van Der Merwe, R. (2000). The unscented Kalman filter for nonlinear estimation. *Proceedings of the IEEE 2000 Adaptive Systems for Signal Processing, Communications, and Control Symposium*, 153-158.

[7] Crassidis, J. L., & Junkins, J. L. (2011). *Optimal Estimation of Dynamic Systems*. CRC Press.

[8] Brown, R. G., & Hwang, P. Y. (2012). *Introduction to Random Signals and Applied Kalman Filtering*. John Wiley & Sons.

[9] Simon, D. (2006). *Optimal State Estimation: Kalman, H Infinity, and Nonlinear Approaches*. John Wiley & Sons.

[10] Ristic, B., Arulampalam, S., & Gordon, N. (2003). *Beyond the Kalman Filter: Particle Filters for Tracking Applications*. Artech House.

[11] Gustafsson, F., Gunnarsson, F., Bergman, N., Forssell, U., Jansson, J., Karlsson, R., & Nordlund, P. J. (2002). Particle filters for positioning, navigation, and tracking. *IEEE Transactions on Signal Processing*, 50(2), 425-437.

[12] Foxlin, E. (2005). Pedestrian tracking with shoe-mounted inertial sensors. *IEEE Computer Graphics and Applications*, 25(6), 38-46.

[13] Jirawimut, R., Ptasinski, P., Garaj, V., Cecelja, F., & Balachandran, W. (2003). A method for dead reckoning parameter correction in pedestrian navigation system. *IEEE Transactions on Instrumentation and Measurement*, 52(1), 209-215.

[14] Shin, S. H., Park, C. G., Kim, J. W., Hong, H. S., & Lee, J. M. (2007). Adaptive step length estimation algorithm using low-cost MEMS inertial sensors. *Sensors and Actuators A: Physical*, 142(1), 52-58.

[15] Borenstein, J., & Feng, L. (1996). Measurement and correction of systematic odometry errors in mobile robots. *IEEE Transactions on Robotics and Automation*, 12(6), 869-880.

[16] Mahony, R., Hamel, T., & Pflimlin, J. M. (2008). Nonlinear complementary filters on the special orthogonal group. *IEEE Transactions on Automatic Control*, 53(5), 1203-1218.

[17] Madyastha, V., Ravindra, V., Mallikarjunan, S., & Goyal, A. (2011). Extended Kalman filter vs. error state Kalman filter for aircraft attitude estimation. *AIAA Guidance, Navigation, and Control Conference*, 6615.

[18] Li, M., & Mourikis, A. I. (2013). High-precision, consistent EKF-based visual-inertial odometry. *The International Journal of Robotics Research*, 32(6), 690-711.

[19] Bloesch, M., Omari, S., Hutter, M., & Siegwart, R. (2017). Robust visual inertial odometry using a direct EKF-based approach. *2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 298-304.

[20] Geneva, P., Eckenhoff, K., Lee, W., Yang, Y., & Huang, G. (2020). OpenVINS: A research platform for visual-inertial estimation. *2020 IEEE International Conference on Robotics and Automation (ICRA)*, 4666-4672.

[21] Meier, L., Honegger, D., & Pollefeys, M. (2015). PX4: A node-based multithreaded open source robotics framework for deeply embedded platforms. *2015 IEEE International Conference on Robotics and Automation (ICRA)*, 6235-6240.

[22] Koubaa, A. (Ed.). (2017). *Robot Operating System (ROS): The Complete Reference (Volume 2)*. Springer.

[23] Leutenegger, S., Lynen, S., Bosse, M., Siegwart, R., & Furgale, P. (2015). Keyframe-based visual–inertial odometry using nonlinear optimization. *The International Journal of Robotics Research*, 34(3), 314-334.

[24] Qin, T., Li, P., & Shen, S. (2018). VINS-Mono: A robust and versatile monocular visual-inertial state estimator. *IEEE Transactions on Robotics*, 34(4), 1004-1020.

## Acknowledgments

This work was conducted as part of an internship program focused on advanced robotics and autonomous systems research. The research benefited from access to state-of-the-art robotic platforms, sensor systems, and computational resources. Special acknowledgment is given to the academic and industrial collaborators who provided guidance, feedback, and technical support throughout the research process.

## License

This project is developed for academic and research purposes. Please refer to individual component licenses for specific usage rights and restrictions.

---

*This repository represents comprehensive research in autonomous vehicle state estimation, demonstrating the integration of theoretical algorithm development with practical real-world implementation and validation.*

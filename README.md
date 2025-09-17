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

### 1. MATLAB Simulations

The `MATLAB simulations/` directory contains comprehensive simulation studies including:

- **Extended Kalman Filter Implementation**: Complete 9-DOF and 12-DOF EKF implementations with nonlinear dynamics modeling
- **Unscented Kalman Filter Development**: Advanced UKF algorithms with sigma-point transformations for handling nonlinear state estimation
- **Parameter Optimization Studies**: Systematic optimization of filter parameters using various optimization techniques
- **Comparative Analysis**: Detailed performance comparisons between EKF and UKF approaches under different operational conditions
- **Obstacle Avoidance Integration**: Integration of state estimation with autonomous navigation and obstacle avoidance algorithms
- **Reinforcement Learning Enhancement**: Advanced RL-based approaches for adaptive parameter tuning and decision making

### 2. Real-Time Implementation

The `RoboMaster+RaspberryPi+iPhone Real-Time Experimentation/` directory contains:

#### Core System Implementation
- **Extended Kalman Filter Core**: Production-ready 8-DOF and 12-DOF EKF implementations optimized for real-time operation
- **iPhone Sensor Integration**: Complete sensor data acquisition and processing pipeline for iPhone IMU, GPS, and magnetometer data
- **RoboMaster S1 Interface**: Hardware abstraction layer for RoboMaster S1 robotic platform communication
- **Real-time Communication**: Network-based data streaming and visualization system

#### Advanced Features
- **Autonomous Control**: Closed-loop autonomous navigation system with waypoint following and obstacle avoidance
- **Sensor Calibration**: Automatic bias estimation and sensor alignment procedures
- **Data Logging and Analysis**: Comprehensive data collection and offline analysis tools
- **Performance Monitoring**: Real-time performance metrics and validation systems

### 3. Documentation and Analysis

The `Resources - References/` directory contains:

- **Technical Reports**: Comprehensive analysis reports documenting simulation and real-world experimental results
- **Mathematical Formulations**: Complete mathematical derivations and algorithm specifications
- **Performance Analysis**: Detailed performance metrics, accuracy assessments, and comparative studies
- **Academic References**: Curated collection of relevant research papers and technical documentation

## Technical Contributions

### Algorithm Development

1. **Enhanced EKF Implementation**: Developed robust Extended Kalman Filter implementations with improved numerical stability and computational efficiency
2. **Smartphone Sensor Fusion**: Created novel approaches for integrating smartphone sensors (IMU, GPS, magnetometer) for autonomous vehicle applications
3. **Real-time Optimization**: Implemented computationally efficient algorithms suitable for embedded systems deployment
4. **Adaptive Parameter Tuning**: Developed automated parameter optimization techniques for improved filter performance

### System Integration

1. **Hardware-Software Integration**: Successfully integrated smartphone sensors with robotic platforms for real-time operation
2. **Communication Protocols**: Implemented robust communication systems for real-time data streaming and control
3. **Calibration Procedures**: Developed automatic calibration and bias estimation procedures for improved accuracy
4. **Validation Framework**: Created comprehensive testing and validation frameworks for algorithm verification

### Performance Analysis

1. **Comparative Studies**: Conducted detailed performance comparisons between different filtering approaches
2. **Accuracy Assessment**: Implemented comprehensive accuracy metrics and validation procedures
3. **Computational Analysis**: Analyzed computational requirements and optimization opportunities
4. **Real-world Validation**: Validated algorithms through extensive real-world testing scenarios

## Key Results and Findings

The research demonstrated significant improvements in autonomous vehicle state estimation accuracy and reliability through:

- Enhanced sensor fusion algorithms achieving improved position and orientation estimation accuracy
- Successful real-time implementation of complex filtering algorithms on resource-constrained embedded systems
- Comprehensive validation of smartphone-based sensor systems for autonomous navigation applications
- Development of robust calibration and parameter optimization procedures for practical deployment

## Future Work and Applications

The developed systems and algorithms have broad applications in:

- Autonomous vehicle navigation and control systems
- Mobile robotics and unmanned aerial vehicle applications
- Smartphone-based navigation and augmented reality systems
- Industrial automation and monitoring systems

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

## Acknowledgments

This work was conducted as part of an internship program focused on advanced robotics and autonomous systems research. The research benefited from access to state-of-the-art robotic platforms, sensor systems, and computational resources. Special acknowledgment is given to the academic and industrial collaborators who provided guidance, feedback, and technical support throughout the research process.

## License

This project is developed for academic and research purposes. Please refer to individual component licenses for specific usage rights and restrictions.

---

*This repository represents comprehensive research in autonomous vehicle state estimation, demonstrating the integration of theoretical algorithm development with practical real-world implementation and validation.*

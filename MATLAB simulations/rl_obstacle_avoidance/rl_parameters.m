%% rl_parameters.m - Reinforcement Learning Parameters Configuration
% PURPOSE
% Central configuration file for RL-based obstacle avoidance system.
% Defines environment, agent, and training parameters.

function params = rl_parameters()
%% Load base drone parameters
% Get the directory where this function is located
[script_path, ~, ~] = fileparts(mfilename('fullpath'));
parent_path = fileparts(script_path);
addpath(parent_path);

% Load existing drone simulation parameters
run(fullfile(parent_path, 'parameters.m'));

% Remove parent path
rmpath(parent_path);

%% RL Environment Parameters
% Start with base parameters and add RL-specific ones
% The base parameters are now in the workspace, so we need to access them
% We'll create a new params struct that includes both base and RL parameters
base_params = struct();
base_params.Ts = params.Ts;
base_params.mass = params.mass;
base_params.I = params.I;
base_params.g = params.g;
base_params.Q = params.Q;
base_params.R_gps = params.R_gps;
base_params.R_baro = params.R_baro;
base_params.R_mag = params.R_mag;
base_params.adaptive_noise = params.adaptive_noise;
base_params.innovation_gate_gps = params.innovation_gate_gps;
base_params.innovation_gate_baro = params.innovation_gate_baro;
base_params.innovation_gate_mag = params.innovation_gate_mag;
base_params.innovation_gate_imu = params.innovation_gate_imu;
base_params.max_angular_rate = params.max_angular_rate;
base_params.drag_coeff = params.drag_coeff;
base_params.gyro_momentum = params.gyro_momentum;
base_params.mag_NED = params.mag_NED;
base_params.arm_length = params.arm_length;
base_params.prop_radius = params.prop_radius;
base_params.CT = params.CT;
base_params.CQ = params.CQ;
base_params.Q_vel = params.Q_vel;
base_params.Q_att = params.Q_att;

% Sensor parameters
base_params.IMU = params.IMU;
base_params.GPS = params.GPS;
base_params.Baro = params.Baro;
base_params.Mag = params.Mag;

% Create the combined params structure
params = base_params;
params.rl = struct();

% Map and Environment (Optimized for Navigation Training)
params.rl.map_resolution = 0.5;        % meters per grid cell
params.rl.map_bounds = [-40, 40, -40, 40, 0, 25];  % [x_min, x_max, y_min, y_max, z_min, z_max] - Smaller for focused training
params.rl.max_terrain_height = 2;      % maximum terrain elevation (m) - Mostly flat for navigation focus
params.rl.terrain_complexity = 0.2;    % 0-1 scale for terrain complexity - Low complexity

% Obstacle Scenarios (Focus on Cylindrical Columns)
params.rl.obstacle_scenario = 'columns'; % 'columns', 'mixed', 'pillars', 'random'
params.rl.obstacle_density = 8;        % percentage of map filled with obstacles - Reduced for navigation
params.rl.navigation_corridor_width = 4; % minimum corridor width (meters)
params.rl.column_spacing_min = 6;      % minimum spacing between columns (meters)

% Episode Parameters
params.rl.max_episode_steps = 500;     % maximum steps per episode (reduced for faster training)
params.rl.success_distance = 2.0;      % distance to goal for success (m)

% Action Space
params.rl.max_velocity = 5.0;          % maximum velocity command (m/s)
params.rl.max_yaw_rate = deg2rad(45);  % maximum yaw rate command (rad/s)

%% RL Agent Parameters (DDPG)

% Network Architecture
params.rl.hidden_dims = [512, 256, 128]; % hidden layer dimensions
params.rl.use_batch_norm = true;         % batch normalization
params.rl.dropout_rate = 0.1;            % dropout rate for regularization

%% Limited Sensor Simulation (PDF Requirement)
params.rl.sensor_mode = 'limited';       % 'full', 'limited', 'ranges', 'depth'
params.rl.limited_sensors = struct();
params.rl.limited_sensors.num_range_beams = 16;     % Number of lidar-like range beams
params.rl.limited_sensors.max_range = 15.0;         % Maximum sensor range (m)
params.rl.limited_sensors.fov_degrees = 180;        % Field of view (degrees)
params.rl.limited_sensors.depth_resolution = [32, 24]; % Depth image resolution
params.rl.limited_sensors.include_ekf_uncertainty = true; % Include EKF covariance in obs

% Training Hyperparameters
params.rl.lr_actor = 1e-4;              % actor learning rate
params.rl.lr_critic = 1e-3;             % critic learning rate
params.rl.gamma = 0.99;                 % discount factor
params.rl.tau = 0.005;                  % soft update rate for target networks
params.rl.batch_size = 128;             % minibatch size

% Experience Replay
params.rl.buffer_size = 1e6;            % replay buffer size
params.rl.min_buffer_size = 10000;      % minimum buffer size before training

% Exploration - IMPROVED
params.rl.noise_scale = 0.5;            % initial exploration noise scale (increased from 0.2)
params.rl.noise_decay = 0.999;          % noise decay per step (slower decay)
params.rl.min_noise = 0.05;             % minimum noise level (increased from 0.01)
params.rl.exploration_steps = 5000;     % steps of pure exploration before training

% Training Schedule
params.rl.train_frequency = 4;          % training steps per environment step
params.rl.target_update_frequency = 2;  % target network update frequency

%% Curriculum Learning - IMPROVED
params.rl.use_curriculum = true;
params.rl.curriculum_thresholds = [-200, -100, 0, 100, 200]; % reward thresholds for advancement (more realistic)
params.rl.curriculum_scenarios = {
    struct('obstacle_density', 2,  'terrain_complexity', 0.1, 'max_episode_steps', 500,  'success_distance', 5.0),
    struct('obstacle_density', 4,  'terrain_complexity', 0.2, 'max_episode_steps', 800,  'success_distance', 4.0),
    struct('obstacle_density', 6,  'terrain_complexity', 0.3, 'max_episode_steps', 1200, 'success_distance', 3.0),
    struct('obstacle_density', 8,  'terrain_complexity', 0.4, 'max_episode_steps', 1500, 'success_distance', 2.5),
    struct('obstacle_density', 10, 'terrain_complexity', 0.5, 'max_episode_steps', 2000, 'success_distance', 2.0)
};

%% Enhanced Reward Function Parameters (PDF-Aligned) - BALANCED
params.rl.rewards = struct();
params.rl.rewards.goal_reached = 1000;          % reward for reaching goal
params.rl.rewards.collision = -100;             % penalty for collision (reduced from -500)
params.rl.rewards.near_collision = -10;         % penalty for getting too close (< 1.5m) (reduced from -50)
params.rl.rewards.out_of_bounds = -50;          % penalty for leaving map (reduced from -100)
params.rl.rewards.progress_scale = 20;          % scale for progress rewards (increased from 10)
params.rl.rewards.energy_penalty = 0.01;       % penalty for large actions (reduced from 0.1)
params.rl.rewards.smoothness_penalty = 0.01;   % penalty for jerky motion (reduced from 0.05)
params.rl.rewards.waypoint_bonus = 50;         % bonus for reaching waypoints (increased from 20)
params.rl.rewards.efficiency_bonus = 2.0;      % bonus for path efficiency (increased from 1.0)
params.rl.rewards.ekf_confidence_bonus = 1.0;  % bonus for good state estimation (increased from 0.5)
params.rl.rewards.proximity_penalty_scale = 2.0; % scale for distance-based penalties (reduced from 5.0)
params.rl.rewards.safe_distance = 2.0;         % safe distance from obstacles (m) (reduced from 3.0)
params.rl.rewards.survival_bonus = 0.1;        % small bonus for each step survived
params.rl.rewards.exploration_bonus = 1.0;     % bonus for exploring new areas

%% State Estimation Integration
params.rl.use_ekf_uncertainty = true;    % include EKF uncertainty in observations
params.rl.uncertainty_weight = 0.1;      % weight for uncertainty in rewards
params.rl.state_noise_injection = 0.02;  % noise level for state observations
params.rl.sensor_dropout_prob = 0.05;    % probability of sensor dropouts

%% Training Configuration
params.rl.training = struct();
params.rl.training.num_episodes = 10000;        % total training episodes
params.rl.training.evaluation_frequency = 100;  % episodes between evaluations
params.rl.training.save_frequency = 500;        % episodes between model saves
params.rl.training.log_frequency = 10;          % episodes between logging

% Evaluation Parameters
params.rl.training.eval_episodes = 10;          % episodes per evaluation
params.rl.training.eval_deterministic = true;   % use deterministic policy for eval
params.rl.training.success_threshold = 0.8;     % success rate for curriculum advancement

%% Visualization and Logging
params.rl.visualization = struct();
params.rl.visualization.render_training = false;     % render during training
params.rl.visualization.render_evaluation = true;    % render during evaluation
params.rl.visualization.save_trajectories = true;    % save trajectory data
params.rl.visualization.plot_frequency = 100;        % episodes between plots

% Logging Configuration
params.rl.logging = struct();
params.rl.logging.log_rewards = true;               % log episode rewards
params.rl.logging.log_losses = true;                % log training losses
params.rl.logging.log_success_rate = true;          % log success rates
params.rl.logging.log_trajectory_metrics = true;    % log path efficiency, etc.
params.rl.logging.tensorboard = false;              % use TensorBoard (if available)

%% Hardware and Performance
params.rl.hardware = struct();
params.rl.hardware.use_gpu = true;                  % use GPU if available
params.rl.hardware.parallel_envs = 1;               % number of parallel environments
params.rl.hardware.num_workers = 4;                 % number of worker threads

%% Advanced Features (PDF-Enhanced)
params.rl.advanced = struct();
params.rl.advanced.prioritized_replay = false;      % prioritized experience replay
params.rl.advanced.dueling_critic = false;          % dueling network architecture
params.rl.advanced.distributional_critic = false;   % distributional RL
params.rl.advanced.hindsight_experience = false;    % hindsight experience replay
params.rl.advanced.domain_randomization = true;     % randomize environment parameters
params.rl.advanced.expert_demonstrations = true;    % use expert demonstrations
params.rl.advanced.behavior_cloning = true;         % enable behavior cloning mode
params.rl.advanced.demo_augmented_rl = true;        % initialize RL with demonstrations

%% Expert Demonstration System (PDF Requirement)
params.rl.expert = struct();
params.rl.expert.use_ekf_expert = true;             % Use EKF-guided drone as expert
params.rl.expert.demo_buffer_size = 50000;          % Expert demonstration buffer size
params.rl.expert.demo_collection_episodes = 1000;   % Episodes to collect expert demos
params.rl.expert.bc_pretrain_epochs = 100;          % Behavior cloning pretraining epochs
params.rl.expert.demo_replay_ratio = 0.25;          % Ratio of demos in RL replay buffer

% Domain Randomization Parameters
if params.rl.advanced.domain_randomization
    params.rl.domain_rand = struct();
    params.rl.domain_rand.gravity_range = [0.9, 1.1];        % gravity scaling factor
    params.rl.domain_rand.mass_range = [0.8, 1.2];           % mass scaling factor
    params.rl.domain_rand.wind_speed_max = 2.0;              % maximum wind speed (m/s)
    params.rl.domain_rand.sensor_noise_scale = [0.5, 2.0];   % sensor noise scaling
    params.rl.domain_rand.actuator_delay_range = [0, 0.05];  % actuator delay range (s)
end

%% Safety and Constraints
params.rl.safety = struct();
params.rl.safety.max_altitude = 25;                 % maximum flight altitude (m)
params.rl.safety.min_altitude = 1;                  % minimum flight altitude (m)
params.rl.safety.emergency_stop_distance = 0.5;     % emergency stop distance (m)
params.rl.safety.velocity_limit_override = true;    % override velocity limits in danger
params.rl.safety.attitude_limit_override = true;    % override attitude limits in danger

%% Integration with Existing EKF
params.rl.ekf_integration = struct();
params.rl.ekf_integration.use_ekf_states = true;    % use EKF estimates instead of true states
params.rl.ekf_integration.inject_ekf_noise = true; % add realistic EKF uncertainty
params.rl.ekf_integration.sensor_failure_sim = true; % simulate sensor failures
params.rl.ekf_integration.gps_outage_prob = 0.02;   % probability of GPS outage per step
params.rl.ekf_integration.mag_interference_prob = 0.01; % probability of mag interference

%% Model Saving and Loading
params.rl.model = struct();
params.rl.model.save_dir = 'rl_models';              % directory for saved models
params.rl.model.save_best_only = true;               % only save best performing models
params.rl.model.model_name_prefix = 'drone_nav_';    % prefix for model filenames
params.rl.model.include_timestamp = true;            % include timestamp in filename

fprintf('RL parameters loaded successfully.\n');
fprintf('Environment: %s obstacles, %.1f complexity\n', ...
        params.rl.obstacle_scenario, params.rl.terrain_complexity);
fprintf('Agent: DDPG with %d-%d-%d hidden layers\n', params.rl.hidden_dims);
fprintf('Training: %d episodes with curriculum learning\n', params.rl.training.num_episodes);

end

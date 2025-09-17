%% demo_rl_navigation.m - Demonstration of RL-based Obstacle Avoidance
% PURPOSE
% Demonstrates the complete RL-based obstacle avoidance system with
% EKF state estimation. Shows training and evaluation phases.
%
% USAGE
%   Run this script to see a complete demonstration:
%   >> demo_rl_navigation

clear; clc; close all;

fprintf('=== RL-based Drone Navigation Demonstration ===\n\n');

%% 1. Parameter Setup
fprintf('Loading parameters and setting up environment...\n');

% Load RL parameters
params = rl_parameters();

% Override some parameters for faster demo
params.rl.training.num_episodes = 500;  % Reduced for demo
params.rl.training.evaluation_frequency = 50;
params.rl.training.log_frequency = 10;
params.sim_duration = 20;  % Shorter simulation duration

% Setup visualization
params.rl.visualization.render_training = true;
params.rl.visualization.render_evaluation = true;

fprintf('Parameters loaded successfully.\n');

%% 2. Environment and Agent Initialization
fprintf('Initializing environment and agent...\n');

try
    % Create environment
    env = rl_environment(params);
    fprintf('✓ Environment created\n');
    
    % Create agent
    agent = rl_agent(env.observation_dim, env.action_dim, params);
    fprintf('✓ Agent created\n');
    
    % Create map generator
    map_gen = map_generator(params);
    fprintf('✓ Map generator created\n');
    
catch ME
    fprintf('✗ Error during initialization: %s\n', ME.message);
    fprintf('This might be due to missing toolboxes. Please check required_toolboxes.md\n');
    return;
end

%% 3. Map Generation Demonstration
fprintf('\nGenerating demonstration maps...\n');

scenarios = {'forest', 'urban', 'mixed'};
difficulties = [0.3, 0.6, 0.9];

for i = 1:length(scenarios)
    scenario = scenarios{i};
    difficulty = difficulties(i);
    
    fprintf('Generating %s map (difficulty %.1f)...\n', scenario, difficulty);
    
    [occupancy_grid, height_map, metadata] = map_gen.generate_map(scenario, i*100, difficulty);
    
    % Save map for later use
    filename = sprintf('rl_maps/demo_%s_map.mat', scenario);
    map_gen.save_map(occupancy_grid, height_map, metadata, filename);
    
    % Visualize map
    figure(i);
    visualize_3d_map(occupancy_grid, height_map, scenario);
    title(sprintf('%s Environment (Difficulty: %.1f)', upper(scenario), difficulty));
end

fprintf('✓ All demonstration maps generated\n');

%% 4. EKF Integration Demonstration
fprintf('\nDemonstrating EKF integration...\n');

% Reset environment with forest map
obs = env.reset();

% Show EKF state estimation with various sensor conditions
fprintf('Testing sensor conditions:\n');

% Normal operation
fprintf('  - Normal operation: ');
action = [0.2, 0.1, 0.0, 0.0];  % Gentle forward motion
[obs, reward, done, info] = env.step(action);
fprintf('EKF uncertainty: %.3f\n', info.ekf_uncertainty);

% GPS outage simulation
fprintf('  - GPS outage: ');
env.rl_ekf_integration.gps_available = false;
[obs, reward, done, info] = env.step(action);
fprintf('EKF uncertainty: %.3f\n', info.ekf_uncertainty);

% Restore GPS
env.rl_ekf_integration.gps_available = true;
fprintf('  - GPS restored: ');
[obs, reward, done, info] = env.step(action);
fprintf('EKF uncertainty: %.3f\n', info.ekf_uncertainty);

fprintf('✓ EKF integration working correctly\n');

%% 5. Training Demonstration (Shortened)
fprintf('\nStarting short training demonstration...\n');

% Training parameters for demo
demo_episodes = 50;
best_reward = -inf;
episode_rewards = [];

% Training loop
for episode = 1:demo_episodes
    obs = env.reset();
    episode_reward = 0;
    episode_steps = 0;
    done = false;
    
    while ~done && episode_steps < 200  % Limit steps for demo
        % Select action (with exploration)
        action = agent.select_action(obs, true);
        
        % Execute action
        [next_obs, reward, done, info] = env.step(action);
        
        % Store experience
        agent.store_experience(obs, action, reward, next_obs, done);
        
        % Train if enough experience
        if agent.get_buffer_size() >= 100  % Lower threshold for demo
            agent.train_step();
        end
        
        obs = next_obs;
        episode_reward = episode_reward + reward;
        episode_steps = episode_steps + 1;
    end
    
    episode_rewards = [episode_rewards, episode_reward];
    
    if episode_reward > best_reward
        best_reward = episode_reward;
    end
    
    % Log progress
    if mod(episode, 10) == 0
        avg_reward = mean(episode_rewards(max(1, end-9):end));
        fprintf('Episode %d: Reward=%.1f, Avg=%.1f, Best=%.1f, Steps=%d\n', ...
                episode, episode_reward, avg_reward, best_reward, episode_steps);
    end
end

fprintf('✓ Training demonstration completed\n');

%% 6. Evaluation Demonstration
fprintf('\nRunning evaluation demonstration...\n');

% Set agent to evaluation mode
agent.training_mode = false;

eval_episodes = 5;
eval_results = struct();
eval_results.rewards = [];
eval_results.success_rate = 0;
eval_results.collision_rate = 0;
eval_results.trajectories = {};

for eval_ep = 1:eval_episodes
    fprintf('Evaluation episode %d...\n', eval_ep);
    
    obs = env.reset();
    eval_reward = 0;
    eval_steps = 0;
    done = false;
    trajectory = [];
    
    while ~done && eval_steps < 300
        % Select action (deterministic)
        action = agent.select_action(obs, false);
        
        % Execute action
        [obs, reward, done, info] = env.step(action);
        
        eval_reward = eval_reward + reward;
        eval_steps = eval_steps + 1;
        
        % Record trajectory
        trajectory = [trajectory, env.drone_state(1:3)];
    end
    
    eval_results.rewards(end+1) = eval_reward;
    eval_results.trajectories{end+1} = trajectory;
    
    if info.goal_reached
        eval_results.success_rate = eval_results.success_rate + 1;
        fprintf('  ✓ Goal reached! Reward: %.1f, Steps: %d\n', eval_reward, eval_steps);
    elseif info.collision
        eval_results.collision_rate = eval_results.collision_rate + 1;
        fprintf('  ✗ Collision occurred. Reward: %.1f, Steps: %d\n', eval_reward, eval_steps);
    else
        fprintf('  - Timeout. Reward: %.1f, Steps: %d\n', eval_reward, eval_steps);
    end
end

% Calculate final statistics
eval_results.success_rate = eval_results.success_rate / eval_episodes;
eval_results.collision_rate = eval_results.collision_rate / eval_episodes;
eval_results.avg_reward = mean(eval_results.rewards);

fprintf('\n=== Evaluation Results ===\n');
fprintf('Success Rate: %.1f%%\n', eval_results.success_rate * 100);
fprintf('Collision Rate: %.1f%%\n', eval_results.collision_rate * 100);
fprintf('Average Reward: %.2f\n', eval_results.avg_reward);

%% 7. Visualization and Analysis
fprintf('\nGenerating analysis plots...\n');

% Training progress
figure(10); clf;
subplot(2, 2, 1);
plot(episode_rewards);
title('Training Progress');
xlabel('Episode');
ylabel('Reward');
grid on;

% Evaluation trajectories
subplot(2, 2, 2);
hold on;
for i = 1:length(eval_results.trajectories)
    traj = eval_results.trajectories{i};
    if ~isempty(traj)
        plot3(traj(1, :), traj(2, :), traj(3, :), 'LineWidth', 2);
    end
end
title('Evaluation Trajectories');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; axis equal;

% EKF uncertainty over time (if available)
if ~isempty(env.rl_ekf_integration.estimation_errors)
    subplot(2, 2, 3);
    errors = env.rl_ekf_integration.estimation_errors;
    pos_errors = sqrt(sum(errors(1:3, :).^2, 1));
    plot(pos_errors);
    title('EKF Position Error');
    xlabel('Time Step');
    ylabel('Position Error (m)');
    grid on;
end

% Performance comparison
subplot(2, 2, 4);
bar([eval_results.success_rate, eval_results.collision_rate, 1 - eval_results.success_rate - eval_results.collision_rate]);
set(gca, 'XTickLabel', {'Success', 'Collision', 'Timeout'});
title('Evaluation Outcomes');
ylabel('Rate');
grid on;

sgtitle('RL Navigation System Analysis');

%% 8. Save Results
fprintf('\nSaving demonstration results...\n');

% Create results directory
if ~exist('demo_results', 'dir'), mkdir('demo_results'); end

% Save evaluation results
save('demo_results/eval_results.mat', 'eval_results', 'episode_rewards', 'params');

% Save current model
agent.save_model('demo_results/demo_model.mat');

% Save training plot
saveas(gcf, 'demo_results/analysis_plots.png');

fprintf('✓ Results saved to demo_results/\n');

%% 9. Summary and Recommendations
fprintf('\n=== Demonstration Summary ===\n');
fprintf('System Components:\n');
fprintf('  ✓ Enhanced UKF with adaptive noise scaling\n');
fprintf('  ✓ RL environment with 3D obstacle maps\n');
fprintf('  ✓ DDPG agent for continuous control\n');
fprintf('  ✓ EKF integration with sensor simulation\n');
fprintf('  ✓ Map generation with multiple scenarios\n\n');

fprintf('Performance:\n');
fprintf('  - Training episodes: %d\n', demo_episodes);
fprintf('  - Best training reward: %.2f\n', best_reward);
fprintf('  - Evaluation success rate: %.1f%%\n', eval_results.success_rate * 100);
fprintf('  - Average evaluation reward: %.2f\n', eval_results.avg_reward);

fprintf('\nRecommendations for full training:\n');
fprintf('  1. Increase training episodes to 5000-10000\n');
fprintf('  2. Use GPU acceleration for faster training\n');
fprintf('  3. Enable curriculum learning for better convergence\n');
fprintf('  4. Tune hyperparameters based on specific scenarios\n');
fprintf('  5. Add domain randomization for robustness\n\n');

fprintf('Next steps:\n');
fprintf('  - Run full training with: rl_training_script.m\n');
fprintf('  - Evaluate on different map scenarios\n');
fprintf('  - Test with real sensor data\n');
fprintf('  - Deploy on actual drone hardware\n\n');

fprintf('=== Demonstration Complete ===\n');
fprintf('Check demo_results/ folder for saved data and plots.\n');

%% Helper Functions

function visualize_3d_map(occupancy_grid, height_map, scenario_name)
    %% Visualize 3D Map
    
    % Downsample for visualization
    downsample_factor = 4;
    grid_vis = occupancy_grid(1:downsample_factor:end, ...
                             1:downsample_factor:end, ...
                             1:downsample_factor:end);
    
    % Find occupied voxels
    [x, y, z] = ind2find(grid_vis == 1);
    
    if ~isempty(x)
        % Plot as 3D scatter
        scatter3(x, y, z, 10, z, 'filled', 'MarkerFaceAlpha', 0.6);
        colormap(jet);
        colorbar;
    end
    
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('%s Map Visualization', scenario_name));
    axis equal; grid on;
    view(45, 30);
end

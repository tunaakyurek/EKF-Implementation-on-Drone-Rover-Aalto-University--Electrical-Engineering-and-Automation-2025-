%% train_rl_system.m - Complete RL System Training with Comprehensive Logging
% PURPOSE
% Train the complete RL obstacle avoidance system with EKF/UKF state estimation
% and generate comprehensive training plots and performance metrics.
%
% IMPORTANT: Uses EKF/UKF for state estimation (NO ground truth during training)
% This ensures realistic training conditions for real-world deployment.

clear; clc; close all;

fprintf('=== RL System Training with EKF/UKF State Estimation ===\n\n');

%% 1. Configuration and Setup
fprintf('Step 1: Setting up training configuration...\n');

% Training configuration
config = struct();
config.filter_type = 'EKF';  % 'EKF' or 'UKF' - choose state estimator
config.num_episodes = 2000;  % Number of training episodes
config.max_steps_per_episode = 1000;  % Maximum steps per episode
config.save_frequency = 100;  % Save model every N episodes
config.plot_frequency = 50;   % Update plots every N episodes

% Override some RL parameters for focused training
fprintf('Loading RL parameters...\n');
addpath('rl_obstacle_avoidance');
addpath('.');  % Add current directory for drone dynamics functions
params = rl_parameters();

% Training-specific parameter overrides
params.rl.training.num_episodes = config.num_episodes;
params.rl.training.max_episode_steps = config.max_steps_per_episode;
params.rl.sensor_mode = 'limited';  % Use limited sensors for realistic training
params.rl.use_ekf_uncertainty = true;  % Include EKF uncertainty in observations

% Curriculum learning schedule
params.rl.curriculum.start_difficulty = 0.1;
params.rl.curriculum.end_difficulty = 1.0;
params.rl.curriculum.difficulty_schedule = 'linear';  % 'linear', 'exponential'

fprintf('✓ Configuration loaded\n');
fprintf('  Filter type: %s (NO ground truth used!)\n', config.filter_type);
fprintf('  Episodes: %d\n', config.num_episodes);
fprintf('  Sensor mode: %s\n', params.rl.sensor_mode);

%% 2. Initialize Training Environment
fprintf('\nStep 2: Initializing training environment...\n');

% Create environment
env = rl_environment(params);

% Create limited sensor observer for realistic training
limited_observer = limited_sensor_observer(params);

% Create RL agent
if params.rl.advanced.expert_demonstrations && exist('expert_demonstrations.mat', 'file')
    fprintf('Loading expert demonstrations...\n');
    load('expert_demonstrations.mat');
    expert_demo = expert_demonstrator(params, env);
    expert_demo.load_demonstrations('expert_demonstrations.mat');
    
    % Create demo-augmented agent
    agent = demo_augmented_rl_agent(env.observation_dim, env.action_dim, params, expert_demo);
    fprintf('✓ Demo-augmented RL agent created\n');
else
    fprintf('No expert demonstrations found, using standard RL agent\n');
    agent = rl_agent(env.observation_dim, env.action_dim, params);
    fprintf('✓ Standard RL agent created\n');
end

% Initialize training logger
training_log = initialize_training_log(config);

fprintf('✓ Environment and agent initialized\n');
fprintf('  Observation dim: %d\n', env.observation_dim);
fprintf('  Action dim: %d\n', env.action_dim);

%% 3. Training Loop with Comprehensive Logging
fprintf('\nStep 3: Starting RL training loop...\n');
fprintf('⚠️  IMPORTANT: Using %s state estimation (realistic training conditions)\n\n', config.filter_type);

start_time = tic;
best_reward = -inf;

for episode = 1:config.num_episodes
    episode_start = tic;
    
    % Reset environment
    obs = env.reset();
    
    % Initialize episode tracking
    episode_reward = 0;
    episode_steps = 0;
    episode_collisions = 0;
    goal_reached = false;
    
    % Episode state estimation tracking (EKF/UKF only - NO ground truth!)
    episode_est_errors = [];
    episode_est_uncertainty = [];
    episode_innovations = [];
    
    % Episode loop - FIXED to respect termination conditions
    last_info = struct();
    while true
        % Select action using current policy
        action = agent.select_action(obs, true);  % With exploration
        
        % Execute action in environment
        [next_obs, reward, done, info] = env.step(action);

        % Note: env already embeds EKF estimate in its observation. Do not
        % override the observation shape here to avoid dimension mismatch.
        
        % Log state estimation performance (EKF/UKF vs true state for analysis)
        % NOTE: True state is ONLY used for analysis - NOT available to agent!
        if isfield(info, 'true_state') && isfield(info, 'estimated_state')
            est_error = norm(info.true_state(1:3) - info.estimated_state(1:3));
            est_uncertainty = trace(info.covariance(1:3, 1:3));
            episode_est_errors = [episode_est_errors, est_error];
            episode_est_uncertainty = [episode_est_uncertainty, est_uncertainty];
        end
        
        % Store experience in replay buffer (obs is previous EKF-based obs after first step)
        agent.store_experience(obs, action, reward, next_obs, done);
        
        % Train agent
        if agent.get_buffer_size() > agent.batch_size
            agent.train_step();
        end
        
        % Update episode metrics
        episode_reward = episode_reward + reward;
        episode_steps = episode_steps + 1;
        last_info = info;
        
        % Check for episode termination
        if done
            break;
        end
        
        if info.collision
            episode_collisions = episode_collisions + 1;
        end
        
        if info.goal_reached
            goal_reached = true;
        end
        
        obs = next_obs;
    end
    
    episode_time = toc(episode_start);
    
    % Log episode results
    training_log = log_episode_results(training_log, episode, episode_reward, ...
        episode_steps, episode_collisions, goal_reached, episode_time, ...
        episode_est_errors, episode_est_uncertainty, agent);
    
    % Update curriculum difficulty
    agent.update_curriculum(episode_reward);
    
    % Progress reporting (more frequent for visibility)
    if mod(episode, 5) == 0 || episode == 1
        avg_reward = mean(training_log.episode_rewards(max(1, episode-49):episode));
        success_rate = mean(training_log.goal_reached(max(1, episode-49):episode)) * 100;
        avg_est_error = mean(training_log.avg_estimation_error(max(1, episode-19):episode));
        done_reason = 'limit';
        if isfield(last_info,'goal_reached') && last_info.goal_reached, done_reason = 'goal'; end
        if isfield(last_info,'collision') && last_info.collision, done_reason = 'collision'; end
        fprintf('Episode %4d: R=%6.1f, Avg50=%6.1f, Succ50=%4.1f%%, EstErr20=%.2fm, Steps=%d, Reason=%s, Time=%.1fs\n', ...
                episode, episode_reward, avg_reward, success_rate, avg_est_error, episode_steps, done_reason, episode_time);
    end
    
    % Save best model
    if episode_reward > best_reward
        best_reward = episode_reward;
        agent.save_model('rl_models/best_rl_model.mat');
    end
    
    % Periodic saves and plotting
    if mod(episode, config.save_frequency) == 0
        agent.save_model(sprintf('rl_models/rl_model_episode_%d.mat', episode));
    end
    
    if mod(episode, config.plot_frequency) == 0 && episode > 50
        update_training_plots(training_log, episode, config.filter_type);
    end
end

total_training_time = toc(start_time);

fprintf('\n✓ Training completed!\n');
fprintf('  Total time: %.1f minutes\n', total_training_time/60);
fprintf('  Best reward: %.2f\n', best_reward);
fprintf('  Final success rate: %.1f%%\n', mean(training_log.goal_reached(end-99:end)) * 100);

%% 4. Save Final Results and Generate Comprehensive Plots
fprintf('\nStep 4: Saving results and generating final plots...\n');

% Save final model
agent.save_model('../rl_models/final_rl_model.mat');

% Save training log
training_filename = sprintf('../rl_logs/training_log_%s_%s.mat', config.filter_type, datestr(now, 'yyyymmdd_HHMMSS'));
save(training_filename, 'training_log', 'config', 'params');

% Generate comprehensive training plots
generate_comprehensive_training_plots(training_log, config);

% Generate filter performance analysis
generate_filter_performance_plots(training_log, config);

% Stay in main directory (no cd needed)

fprintf('✓ All training results saved\n');
fprintf('✓ Comprehensive plots generated\n');

%% Summary
fprintf('\n=== TRAINING SUMMARY ===\n');
fprintf('Filter Used: %s (realistic state estimation)\n', config.filter_type);
fprintf('Training Episodes: %d\n', config.num_episodes);
fprintf('Final Performance:\n');
fprintf('  Average Reward (last 100): %.2f\n', mean(training_log.episode_rewards(end-99:end)));
fprintf('  Success Rate (last 100): %.1f%%\n', mean(training_log.goal_reached(end-99:end)) * 100);
fprintf('  Average Estimation Error: %.2fm\n', mean(training_log.avg_estimation_error(end-99:end)));
fprintf('  Collision Rate: %.1f%%\n', mean(training_log.collision_rate(end-99:end)) * 100);

fprintf('\nFiles Generated:\n');
fprintf('  - final_rl_model.mat: Trained RL agent\n');
fprintf('  - best_rl_model.mat: Best performing model\n');
fprintf('  - training_log_*.mat: Complete training data\n');
fprintf('  - Training performance plots\n');
fprintf('  - Filter performance analysis\n');

fprintf('\nReady for evaluation phase! ✅\n');

%% Helper Functions

function training_log = initialize_training_log(config)
    %% Initialize Training Data Logger
    
    N = config.num_episodes;
    
    training_log = struct();
    training_log.config = config;
    training_log.episode_rewards = zeros(1, N);
    training_log.episode_lengths = zeros(1, N);
    training_log.goal_reached = false(1, N);
    training_log.collision_rate = zeros(1, N);
    training_log.episode_times = zeros(1, N);
    
    % State estimation tracking
    training_log.avg_estimation_error = zeros(1, N);
    training_log.avg_estimation_uncertainty = zeros(1, N);
    training_log.filter_performance = zeros(1, N);
    
    % Learning metrics
    training_log.actor_losses = [];
    training_log.critic_losses = [];
    training_log.exploration_noise = zeros(1, N);
    training_log.curriculum_difficulty = zeros(1, N);
    
    training_log.episode_count = 0;
end
function obs = build_observation_from_estimate(x_est, P_est, params)
    %% Build RL observation vector from EKF estimate and uncertainty
    % x_est: [pos(3); vel(3); att(3)]
    obs_core = [x_est(:).'];
    if isfield(params.rl, 'use_ekf_uncertainty') && params.rl.use_ekf_uncertainty
        % Use diagonal of covariance for simplicity
        diagP = diag(P_est).';
        obs = [obs_core, diagP];
    else
        obs = obs_core;
    end
    obs = obs(:); % column vector
end

function training_log = log_episode_results(training_log, episode, reward, steps, ...
    collisions, goal_reached, episode_time, est_errors, est_uncertainty, agent)
    %% Log Results from Completed Episode
    
    training_log.episode_count = episode;
    training_log.episode_rewards(episode) = reward;
    training_log.episode_lengths(episode) = steps;
    training_log.goal_reached(episode) = goal_reached;
    training_log.collision_rate(episode) = collisions / steps;
    training_log.episode_times(episode) = episode_time;
    
    % State estimation performance
    if ~isempty(est_errors)
        training_log.avg_estimation_error(episode) = mean(est_errors);
        training_log.avg_estimation_uncertainty(episode) = mean(est_uncertainty);
        training_log.filter_performance(episode) = 1.0 / (1.0 + mean(est_errors));  % Normalized performance
    end
    
    % Learning metrics
    training_log.exploration_noise(episode) = agent.noise_scale;
    training_log.curriculum_difficulty(episode) = 1.0;  % Default difficulty
    
    % Store recent losses
    if ~isempty(agent.actor_losses)
        training_log.actor_losses = [training_log.actor_losses, agent.actor_losses(end)];
    end
    if ~isempty(agent.critic_losses)
        training_log.critic_losses = [training_log.critic_losses, agent.critic_losses(end)];
    end
end

function update_training_plots(training_log, current_episode, filter_type)
    %% Update Training Progress Plots
    
    % Create or update figure
    fig = findobj('Type', 'figure', 'Name', 'RL Training Progress');
    if isempty(fig)
        fig = figure('Name', 'RL Training Progress', 'Position', [100, 100, 1200, 800]);
    end
    figure(fig);
    
    episodes = 1:current_episode;
    
    % Reward progress
    subplot(2, 3, 1);
    plot(episodes, training_log.episode_rewards(episodes), 'b-', 'LineWidth', 1);
    hold on;
    if current_episode > 100
        moving_avg = movmean(training_log.episode_rewards(episodes), 100);
        plot(episodes, moving_avg, 'r-', 'LineWidth', 2);
    end
    xlabel('Episode');
    ylabel('Reward');
    title('Episode Rewards');
    grid on;
    hold off;
    
    % Success rate
    subplot(2, 3, 2);
    if current_episode > 50
        success_window = movmean(double(training_log.goal_reached(episodes)), 50) * 100;
        plot(episodes, success_window, 'g-', 'LineWidth', 2);
    end
    xlabel('Episode');
    ylabel('Success Rate (%)');
    title('Goal Achievement Rate');
    grid on;
    ylim([0, 100]);
    
    % State estimation error
    subplot(2, 3, 3);
    plot(episodes, training_log.avg_estimation_error(episodes), 'c-', 'LineWidth', 1);
    xlabel('Episode');
    ylabel('Estimation Error (m)');
    title(sprintf('%s State Estimation Error', filter_type));
    grid on;
    
    % Learning curves
    subplot(2, 3, 4);
    if length(training_log.actor_losses) > 10
        plot(training_log.actor_losses, 'm-', 'LineWidth', 1, 'DisplayName', 'Actor Loss');
        hold on;
        plot(training_log.critic_losses, 'k-', 'LineWidth', 1, 'DisplayName', 'Critic Loss');
        xlabel('Training Step');
        ylabel('Loss');
        title('Learning Curves');
        legend('Location', 'best');
        grid on;
        hold off;
    end
    
    % Exploration and curriculum
    subplot(2, 3, 5);
    yyaxis left;
    plot(episodes, training_log.exploration_noise(episodes), 'b-', 'LineWidth', 1);
    ylabel('Exploration Noise');
    yyaxis right;
    plot(episodes, training_log.curriculum_difficulty(episodes), 'r-', 'LineWidth', 1);
    ylabel('Curriculum Difficulty');
    xlabel('Episode');
    title('Exploration & Curriculum');
    grid on;
    
    % Episode lengths and collision rates
    subplot(2, 3, 6);
    yyaxis left;
    plot(episodes, training_log.episode_lengths(episodes), 'g-', 'LineWidth', 1);
    ylabel('Episode Length');
    yyaxis right;
    collision_rate_pct = training_log.collision_rate(episodes) * 100;
    plot(episodes, collision_rate_pct, 'r-', 'LineWidth', 1);
    ylabel('Collision Rate (%)');
    xlabel('Episode');
    title('Episode Stats');
    grid on;
    
    sgtitle(sprintf('RL Training Progress (Episode %d) - %s Filter', current_episode, filter_type), ...
            'FontSize', 14, 'FontWeight', 'bold');
    
    drawnow;
end

function generate_comprehensive_training_plots(training_log, config)
    %% Generate Final Comprehensive Training Analysis Plots
    
    episodes = 1:training_log.episode_count;
    
    %% Main Training Results Figure
    figure('Name', 'RL Training Results', 'Position', [50, 50, 1400, 900]);
    
    % Episode rewards with moving average
    subplot(3, 3, 1);
    plot(episodes, training_log.episode_rewards(episodes), 'b-', 'LineWidth', 0.5, 'Color', [0.7, 0.7, 1]);
    hold on;
    moving_avg = movmean(training_log.episode_rewards(episodes), 100);
    plot(episodes, moving_avg, 'b-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Reward');
    title('Episode Rewards');
    grid on;
    
    % Success rate evolution
    subplot(3, 3, 2);
    success_rate = movmean(double(training_log.goal_reached(episodes)), 50) * 100;
    plot(episodes, success_rate, 'g-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Success Rate (%)');
    title('Goal Achievement Rate');
    grid on;
    ylim([0, 100]);
    
    % State estimation performance
    subplot(3, 3, 3);
    plot(episodes, training_log.avg_estimation_error(episodes), 'c-', 'LineWidth', 1);
    hold on;
    est_avg = movmean(training_log.avg_estimation_error(episodes), 20);
    plot(episodes, est_avg, 'c-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Estimation Error (m)');
    title(sprintf('%s State Estimation Error', config.filter_type));
    grid on;
    
    % Learning curves
    subplot(3, 3, 4);
    if length(training_log.actor_losses) > 10
        smoothed_actor = movmean(training_log.actor_losses, 50);
        smoothed_critic = movmean(training_log.critic_losses, 50);
        plot(smoothed_actor, 'm-', 'LineWidth', 2, 'DisplayName', 'Actor Loss');
        hold on;
        plot(smoothed_critic, 'k-', 'LineWidth', 2, 'DisplayName', 'Critic Loss');
        xlabel('Training Step');
        ylabel('Loss');
        title('Learning Curves');
        legend('Location', 'best');
        grid on;
    end
    
    % Exploration decay
    subplot(3, 3, 5);
    plot(episodes, training_log.exploration_noise(episodes), 'b-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Exploration Noise');
    title('Exploration Schedule');
    grid on;
    
    % Curriculum difficulty
    subplot(3, 3, 6);
    plot(episodes, training_log.curriculum_difficulty(episodes), 'r-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Difficulty');
    title('Curriculum Learning');
    grid on;
    
    % Episode length distribution
    subplot(3, 3, 7);
    histogram(training_log.episode_lengths(episodes), 30, 'FaceColor', 'green', 'FaceAlpha', 0.7);
    xlabel('Episode Length');
    ylabel('Frequency');
    title('Episode Length Distribution');
    grid on;
    
    % Collision rate evolution
    subplot(3, 3, 8);
    collision_rate_smooth = movmean(training_log.collision_rate(episodes), 50) * 100;
    plot(episodes, collision_rate_smooth, 'r-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Collision Rate (%)');
    title('Collision Rate Evolution');
    grid on;
    
    % Performance summary
    subplot(3, 3, 9);
    final_success = mean(training_log.goal_reached(end-99:end)) * 100;
    final_reward = mean(training_log.episode_rewards(end-99:end));
    final_est_error = mean(training_log.avg_estimation_error(end-99:end));
    
    bar_data = [final_success/100, (final_reward+500)/1000, 1-final_est_error/10];
    bar_data = max(0, min(1, bar_data));  % Normalize to [0,1]
    
    b = bar(bar_data, 'FaceColor', 'flat');
    b.CData = [0.2, 0.8, 0.2; 0.2, 0.2, 0.8; 0.8, 0.2, 0.2];
    set(gca, 'XTickLabel', {'Success', 'Reward', 'Accuracy'});
    ylabel('Normalized Performance');
    title('Final Performance');
    grid on;
    ylim([0, 1]);
    
    sgtitle(sprintf('RL Training Results (%s Filter) - %d Episodes', config.filter_type, config.num_episodes), ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    % Save figure
    savefig(sprintf('../rl_logs/RL_Training_Results_%s.fig', config.filter_type));
    saveas(gcf, sprintf('../rl_logs/RL_Training_Results_%s.png', config.filter_type));
    
    fprintf('✓ Training results plots saved\n');
end

function generate_filter_performance_plots(training_log, config)
    %% Generate State Estimation Performance Analysis
    
    episodes = 1:training_log.episode_count;
    
    figure('Name', 'State Estimation Performance', 'Position', [150, 150, 1200, 600]);
    
    % Estimation error evolution
    subplot(2, 3, 1);
    plot(episodes, training_log.avg_estimation_error(episodes), 'b-', 'LineWidth', 1);
    hold on;
    error_smooth = movmean(training_log.avg_estimation_error(episodes), 50);
    plot(episodes, error_smooth, 'r-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Position Error (m)');
    title(sprintf('%s Position Estimation Error', config.filter_type));
    legend('Raw', 'Smoothed', 'Location', 'best');
    grid on;
    
    % Estimation uncertainty
    subplot(2, 3, 2);
    plot(episodes, training_log.avg_estimation_uncertainty(episodes), 'g-', 'LineWidth', 1);
    hold on;
    unc_smooth = movmean(training_log.avg_estimation_uncertainty(episodes), 50);
    plot(episodes, unc_smooth, 'k-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Position Uncertainty');
    title('State Estimation Uncertainty');
    legend('Raw', 'Smoothed', 'Location', 'best');
    grid on;
    
    % Filter performance score
    subplot(2, 3, 3);
    plot(episodes, training_log.filter_performance(episodes), 'm-', 'LineWidth', 1);
    hold on;
    perf_smooth = movmean(training_log.filter_performance(episodes), 50);
    plot(episodes, perf_smooth, 'c-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Performance Score');
    title('Filter Performance Score');
    legend('Raw', 'Smoothed', 'Location', 'best');
    grid on;
    
    % Error distribution
    subplot(2, 3, 4);
    histogram(training_log.avg_estimation_error(episodes), 30, 'FaceColor', 'blue', 'FaceAlpha', 0.7);
    xlabel('Position Error (m)');
    ylabel('Frequency');
    title('Error Distribution');
    grid on;
    
    % Error vs Success correlation
    subplot(2, 3, 5);
    scatter(training_log.avg_estimation_error(episodes), double(training_log.goal_reached(episodes)), ...
            20, 'filled', 'MarkerFaceAlpha', 0.6);
    xlabel('Estimation Error (m)');
    ylabel('Goal Reached');
    title('Error vs Success');
    grid on;
    
    % Performance timeline
    subplot(2, 3, 6);
    yyaxis left;
    plot(episodes, error_smooth, 'b-', 'LineWidth', 2);
    ylabel('Estimation Error (m)', 'Color', 'b');
    yyaxis right;
    success_smooth = movmean(double(training_log.goal_reached(episodes)), 50) * 100;
    plot(episodes, success_smooth, 'r-', 'LineWidth', 2);
    ylabel('Success Rate (%)', 'Color', 'r');
    xlabel('Episode');
    title('Error vs Success Evolution');
    grid on;
    
    sgtitle(sprintf('%s State Estimation Analysis', config.filter_type), ...
            'FontSize', 14, 'FontWeight', 'bold');
    
    % Save figure
    savefig(sprintf('../rl_logs/Filter_Performance_%s.fig', config.filter_type));
    saveas(gcf, sprintf('../rl_logs/Filter_Performance_%s.png', config.filter_type));
    
    fprintf('✓ Filter performance plots saved\n');
end

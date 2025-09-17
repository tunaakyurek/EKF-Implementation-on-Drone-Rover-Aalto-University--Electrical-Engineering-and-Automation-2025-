%% evaluate_trained_rl.m - Comprehensive Evaluation of Trained RL Model
% PURPOSE
% Load and evaluate the trained RL model on obstacle maps with comprehensive
% performance analysis, trajectory visualization, and success metrics.

clear; clc; close all;

fprintf('=== Trained RL Model Evaluation ===\n\n');

%% 1. Load Trained Model and Setup
fprintf('Step 1: Loading trained model...\n');

addpath('rl_obstacle_avoidance');
addpath('.');
params = rl_parameters();

% Load the best trained model
if exist('rl_models/best_rl_model.mat', 'file')
    fprintf('Loading best trained model...\n');
    model_data = load('rl_models/best_rl_model.mat');
    model_weights = model_data.model_data;
    fprintf('✓ Best model loaded\n');
elseif exist('rl_models/final_rl_model.mat', 'file')
    fprintf('Loading final trained model...\n');
    model_data = load('rl_models/final_rl_model.mat');
    model_weights = model_data.model_data;
    fprintf('✓ Final model loaded\n');
else
    error('No trained model found! Please run train_rl_system.m first.');
end

% Reconstruct agent from saved weights
fprintf('Reconstructing agent from saved weights...\n');
trained_agent = rl_agent(model_weights.obs_dim, model_weights.action_dim, params);
trained_agent.actor_network = model_weights.actor_network;
trained_agent.critic_network = model_weights.critic_network;
% For evaluation, we don't need target networks, just use the main networks
trained_agent.target_actor = model_weights.actor_network;
trained_agent.target_critic = model_weights.critic_network;
trained_agent.training_mode = false;  % Set to evaluation mode
fprintf('✓ Agent reconstructed\n');

%% 2. Evaluation Configuration
config = struct();
config.num_test_episodes = 50;  % Number of test episodes
config.test_scenarios = {'easy', 'medium', 'hard'};  % Different difficulty levels
config.use_filter = 'EKF';  % 'EKF' or 'UKF' for state estimation
config.save_trajectories = true;

fprintf('✓ Evaluation configuration set\n');
fprintf('  Test episodes: %d per scenario\n', config.num_test_episodes);
fprintf('  Filter: %s\n', config.use_filter);

%% 3. Run Comprehensive Evaluation
fprintf('\nStep 2: Running comprehensive evaluation...\n');

evaluation_results = struct();

for scenario_idx = 1:length(config.test_scenarios)
    scenario = config.test_scenarios{scenario_idx};
    fprintf('\nTesting scenario: %s\n', upper(scenario));
    
    % Set difficulty level
    switch scenario
        case 'easy'
            params.rl.obstacle_density = 5;
            params.rl.terrain_complexity = 0.1;
        case 'medium'
            params.rl.obstacle_density = 15;
            params.rl.terrain_complexity = 0.5;
        case 'hard'
            params.rl.obstacle_density = 25;
            params.rl.terrain_complexity = 0.8;
    end
    
    % Create environment for this scenario
    env = rl_environment(params);
    
    % Run evaluation episodes
    scenario_results = run_evaluation_episodes(env, trained_agent, config);
    evaluation_results.(scenario) = scenario_results;
    
    fprintf('  Success rate: %.1f%%\n', scenario_results.success_rate * 100);
    fprintf('  Avg reward: %.2f\n', scenario_results.avg_reward);
    fprintf('  Avg estimation error: %.2fm\n', scenario_results.avg_estimation_error);
end

%% 4. Generate Comprehensive Evaluation Plots
fprintf('\nStep 3: Generating evaluation plots...\n');

generate_evaluation_plots(evaluation_results, config);
generate_trajectory_plots(evaluation_results, config);
generate_filter_accuracy_plots(evaluation_results, config);

% Stay in main directory

%% 5. Summary Report
fprintf('\n=== EVALUATION SUMMARY ===\n');
fprintf('Model Performance Across Scenarios:\n\n');
fprintf('Scenario | Success Rate | Avg Reward | Est Error | Collision Rate\n');
fprintf('---------|--------------|------------|-----------|---------------\n');

for scenario_idx = 1:length(config.test_scenarios)
    scenario = config.test_scenarios{scenario_idx};
    results = evaluation_results.(scenario);
    fprintf('%-8s | %9.1f%% | %10.2f | %7.2fm | %11.1f%%\n', ...
            upper(scenario), results.success_rate*100, results.avg_reward, ...
            results.avg_estimation_error, results.collision_rate*100);
end

fprintf('\nKey Findings:\n');
overall_success = mean([evaluation_results.easy.success_rate, ...
                       evaluation_results.medium.success_rate, ...
                       evaluation_results.hard.success_rate]);
fprintf('- Overall success rate: %.1f%%\n', overall_success*100);
fprintf('- %s state estimation used (no ground truth!)\n', config.use_filter);
fprintf('- All plots and data saved to rl_logs/\n');

fprintf('\n✅ Evaluation complete!\n');

%% Helper Functions
function results = run_evaluation_episodes(env, agent, config)
    %% Run Evaluation Episodes for One Scenario
    
    N = config.num_test_episodes;
    results = struct();
    
    % Initialize tracking arrays
    episode_rewards = zeros(N, 1);
    episode_lengths = zeros(N, 1);
    success_flags = false(N, 1);
    collision_counts = zeros(N, 1);
    estimation_errors = cell(N, 1);
    trajectories = cell(N, 1);
    
    for episode = 1:N
        % Reset environment
        obs = env.reset();
        
        episode_reward = 0;
        episode_steps = 0;
        episode_est_errors = [];
        episode_trajectory = [];
        
        for step = 1:1000  % Max 1000 steps per episode
            % Get action from trained agent (no exploration)
            action = agent.select_action(obs, false);
            
            % Execute action
            [next_obs, reward, done, info] = env.step(action);
            
            % Track performance
            episode_reward = episode_reward + reward;
            episode_steps = episode_steps + 1;
            
            % Track estimation error if available
            if isfield(info, 'estimation_error')
                episode_est_errors = [episode_est_errors, info.estimation_error];
            end
            
            % Track trajectory
            if config.save_trajectories && isfield(info, 'true_state')
                episode_trajectory = [episode_trajectory, info.true_state(1:3)];
            end
            
            obs = next_obs;
            
            if done
                success_flags(episode) = info.goal_reached;
                if isfield(info, 'collision_count')
                    collision_counts(episode) = info.collision_count;
                end
                break;
            end
        end
        
        % Store episode results
        episode_rewards(episode) = episode_reward;
        episode_lengths(episode) = episode_steps;
        estimation_errors{episode} = episode_est_errors;
        if config.save_trajectories
            trajectories{episode} = episode_trajectory;
        end
        
        if mod(episode, 10) == 0
            fprintf('    Episode %d/%d completed\n', episode, N);
        end
    end
    
    % Calculate summary statistics
    results.success_rate = mean(success_flags);
    results.avg_reward = mean(episode_rewards);
    results.avg_episode_length = mean(episode_lengths);
    results.collision_rate = mean(collision_counts > 0);
    
    % Calculate average estimation error
    all_errors = [];
    for i = 1:N
        all_errors = [all_errors, estimation_errors{i}];
    end
    results.avg_estimation_error = mean(all_errors);
    
    % Store detailed data
    results.episode_rewards = episode_rewards;
    results.episode_lengths = episode_lengths;
    results.success_flags = success_flags;
    results.estimation_errors = estimation_errors;
    results.trajectories = trajectories;
end

function generate_evaluation_plots(evaluation_results, config)
    %% Generate Performance Comparison Plots
    
    scenarios = config.test_scenarios;
    
    figure('Name', 'RL Model Evaluation Results', 'Position', [100, 100, 1400, 800]);
    
    % Success rates
    subplot(2, 3, 1);
    success_rates = zeros(1, length(scenarios));
    for i = 1:length(scenarios)
        success_rates(i) = evaluation_results.(scenarios{i}).success_rate * 100;
    end
    bar(success_rates, 'FaceColor', 'green', 'FaceAlpha', 0.7);
    set(gca, 'XTickLabel', upper(scenarios));
    ylabel('Success Rate (%)');
    title('Success Rate by Scenario');
    grid on;
    ylim([0, 100]);
    
    % Average rewards
    subplot(2, 3, 2);
    avg_rewards = zeros(1, length(scenarios));
    for i = 1:length(scenarios)
        avg_rewards(i) = evaluation_results.(scenarios{i}).avg_reward;
    end
    bar(avg_rewards, 'FaceColor', 'blue', 'FaceAlpha', 0.7);
    set(gca, 'XTickLabel', upper(scenarios));
    ylabel('Average Reward');
    title('Average Reward by Scenario');
    grid on;
    
    % Estimation errors
    subplot(2, 3, 3);
    est_errors = zeros(1, length(scenarios));
    for i = 1:length(scenarios)
        est_errors(i) = evaluation_results.(scenarios{i}).avg_estimation_error;
    end
    bar(est_errors, 'FaceColor', 'red', 'FaceAlpha', 0.7);
    set(gca, 'XTickLabel', upper(scenarios));
    ylabel('Estimation Error (m)');
    title(sprintf('%s Estimation Error', config.use_filter));
    grid on;
    
    % Episode length distributions
    subplot(2, 3, 4);
    for i = 1:length(scenarios)
        lengths = evaluation_results.(scenarios{i}).episode_lengths;
        histogram(lengths, 20, 'FaceAlpha', 0.6, 'DisplayName', upper(scenarios{i}));
        hold on;
    end
    xlabel('Episode Length');
    ylabel('Frequency');
    title('Episode Length Distributions');
    legend('Location', 'best');
    grid on;
    
    % Reward distributions
    subplot(2, 3, 5);
    for i = 1:length(scenarios)
        rewards = evaluation_results.(scenarios{i}).episode_rewards;
        histogram(rewards, 20, 'FaceAlpha', 0.6, 'DisplayName', upper(scenarios{i}));
        hold on;
    end
    xlabel('Episode Reward');
    ylabel('Frequency');
    title('Reward Distributions');
    legend('Location', 'best');
    grid on;
    
    % Performance summary
    subplot(2, 3, 6);
    collision_rates = zeros(1, length(scenarios));
    for i = 1:length(scenarios)
        collision_rates(i) = evaluation_results.(scenarios{i}).collision_rate * 100;
    end
    bar(collision_rates, 'FaceColor', 'orange', 'FaceAlpha', 0.7);
    set(gca, 'XTickLabel', upper(scenarios));
    ylabel('Collision Rate (%)');
    title('Collision Rate by Scenario');
    grid on;
    
    sgtitle('Trained RL Model Performance Evaluation', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Save plots
    savefig('../rl_logs/RL_Evaluation_Results.fig');
    saveas(gcf, '../rl_logs/RL_Evaluation_Results.png');
    
    fprintf('✓ Evaluation plots saved\n');
end

function generate_trajectory_plots(evaluation_results, config)
    %% Generate Trajectory Visualization Plots
    
    if ~config.save_trajectories
        return;
    end
    
    scenarios = config.test_scenarios;
    
    figure('Name', 'RL Trajectory Analysis', 'Position', [200, 200, 1200, 800]);
    
    for i = 1:length(scenarios)
        subplot(2, 2, i);
        
        scenario = scenarios{i};
        trajectories = evaluation_results.(scenario).trajectories;
        success_flags = evaluation_results.(scenario).success_flags;
        
        % Plot successful trajectories in green, failed in red
        for j = 1:min(10, length(trajectories))  % Plot up to 10 trajectories
            traj = trajectories{j};
            if ~isempty(traj) && size(traj, 2) > 1
                if success_flags(j)
                    plot3(traj(1,:), traj(2,:), traj(3,:), 'g-', 'LineWidth', 1.5);
                else
                    plot3(traj(1,:), traj(2,:), traj(3,:), 'r--', 'LineWidth', 1);
                end
                hold on;
            end
        end
        
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title(sprintf('%s Scenario Trajectories', upper(scenario)));
        grid on;
        axis equal;
        view(3);
    end
    
    % Add legend in last subplot
    subplot(2, 2, 4);
    hold on;
    plot3(NaN, NaN, NaN, 'g-', 'LineWidth', 2, 'DisplayName', 'Successful');
    plot3(NaN, NaN, NaN, 'r--', 'LineWidth', 2, 'DisplayName', 'Failed');
    legend('Location', 'best');
    
    sgtitle('3D Trajectory Analysis', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Save plots
    savefig('../rl_logs/RL_Trajectory_Analysis.fig');
    saveas(gcf, '../rl_logs/RL_Trajectory_Analysis.png');
    
    fprintf('✓ Trajectory plots saved\n');
end

function generate_filter_accuracy_plots(evaluation_results, config)
    %% Generate Filter Accuracy Analysis
    
    scenarios = config.test_scenarios;
    
    figure('Name', 'Filter Accuracy Analysis', 'Position', [300, 300, 1000, 600]);
    
    % Estimation error evolution
    subplot(2, 2, 1);
    for i = 1:length(scenarios)
        scenario = scenarios{i};
        all_errors = [];
        for j = 1:length(evaluation_results.(scenario).estimation_errors)
            errors = evaluation_results.(scenario).estimation_errors{j};
            all_errors = [all_errors, errors];
        end
        
        if ~isempty(all_errors)
            plot(all_errors, 'LineWidth', 1, 'DisplayName', upper(scenario));
            hold on;
        end
    end
    xlabel('Time Step');
    ylabel('Estimation Error (m)');
    title(sprintf('%s Estimation Error Evolution', config.use_filter));
    legend('Location', 'best');
    grid on;
    
    % Error distributions
    subplot(2, 2, 2);
    for i = 1:length(scenarios)
        scenario = scenarios{i};
        all_errors = [];
        for j = 1:length(evaluation_results.(scenario).estimation_errors)
            errors = evaluation_results.(scenario).estimation_errors{j};
            all_errors = [all_errors, errors];
        end
        
        if ~isempty(all_errors)
            histogram(all_errors, 30, 'FaceAlpha', 0.6, 'DisplayName', upper(scenario));
            hold on;
        end
    end
    xlabel('Estimation Error (m)');
    ylabel('Frequency');
    title('Error Distribution by Scenario');
    legend('Location', 'best');
    grid on;
    
    % Success vs Error correlation
    subplot(2, 2, 3);
    all_errors = [];
    all_success = [];
    
    for i = 1:length(scenarios)
        scenario = scenarios{i};
        for j = 1:length(evaluation_results.(scenario).estimation_errors)
            errors = evaluation_results.(scenario).estimation_errors{j};
            if ~isempty(errors)
                avg_error = mean(errors);
                success = evaluation_results.(scenario).success_flags(j);
                all_errors = [all_errors, avg_error];
                all_success = [all_success, success];
            end
        end
    end
    
    scatter(all_errors, all_success, 50, 'filled', 'MarkerFaceAlpha', 0.6);
    xlabel('Average Estimation Error (m)');
    ylabel('Mission Success');
    title('Success vs Estimation Error');
    grid on;
    
    % Filter performance summary
    subplot(2, 2, 4);
    avg_errors = zeros(1, length(scenarios));
    for i = 1:length(scenarios)
        avg_errors(i) = evaluation_results.(scenarios{i}).avg_estimation_error;
    end
    
    bar(avg_errors, 'FaceColor', 'cyan', 'FaceAlpha', 0.7);
    set(gca, 'XTickLabel', upper(scenarios));
    ylabel('Average Error (m)');
    title(sprintf('%s Performance Summary', config.use_filter));
    grid on;
    
    sgtitle(sprintf('%s State Estimation Accuracy Analysis', config.use_filter), ...
            'FontSize', 14, 'FontWeight', 'bold');
    
    % Save plots
    savefig(sprintf('../rl_logs/Filter_Accuracy_%s.fig', config.use_filter));
    saveas(gcf, sprintf('../rl_logs/Filter_Accuracy_%s.png', config.use_filter));
    
    fprintf('✓ Filter accuracy plots saved\n');
end

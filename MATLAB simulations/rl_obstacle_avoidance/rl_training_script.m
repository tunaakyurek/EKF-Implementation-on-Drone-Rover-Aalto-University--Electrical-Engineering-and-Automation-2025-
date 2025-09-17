%% rl_training_script.m - Main Training Script for RL-based Obstacle Avoidance
% PURPOSE
% Complete training script that orchestrates the RL training process for
% autonomous drone navigation with obstacle avoidance capabilities.
%
% FEATURES
% - Curriculum learning with progressive difficulty
% - Comprehensive logging and visualization
% - Model checkpointing and evaluation
% - Integration with existing EKF simulation
%
% USAGE
%   Run this script directly to start training:
%   >> rl_training_script

clear; clc; close all;

%% Setup and Initialization
fprintf('=== RL-based Drone Obstacle Avoidance Training ===\n\n');

% Load parameters
params = rl_parameters();

% Create output directories
if ~exist('rl_models', 'dir'), mkdir('rl_models'); end
if ~exist('rl_logs', 'dir'), mkdir('rl_logs'); end
if ~exist('rl_maps', 'dir'), mkdir('rl_maps'); end

% Initialize environment and agent
fprintf('Initializing environment and agent...\n');
env = rl_environment(params);
agent = rl_agent(env.observation_dim, env.action_dim, params);

% Initialize map generator
map_gen = map_generator(params);

% Training state
training_stats = struct();
training_stats.episode_rewards = [];
training_stats.success_rates = [];
training_stats.episode_lengths = [];
training_stats.actor_losses = [];
training_stats.critic_losses = [];
training_stats.evaluation_scores = [];

% Best model tracking
best_eval_score = -inf;
episodes_since_improvement = 0;

%% Main Training Loop
fprintf('Starting training for %d episodes...\n', params.rl.training.num_episodes);
tic;

for episode = 1:params.rl.training.num_episodes
    
    %% Episode Setup
    episode_start_time = tic;
    
    % Update curriculum level if needed
    if params.rl.use_curriculum && episode > 100
        current_level = agent.curriculum_level;
        curriculum_params = params.rl.curriculum_scenarios{min(current_level, end)};
        
        % Update environment parameters for current curriculum level
        env.params.rl.obstacle_density = curriculum_params.obstacle_density;
        env.params.rl.terrain_complexity = curriculum_params.terrain_complexity;
        env.max_episode_steps = curriculum_params.max_episode_steps;
    end
    
    % Reset environment for new episode
    observation = env.reset();
    
    % Episode tracking
    episode_reward = 0;
    episode_steps = 0;
    episode_done = false;
    
    %% Episode Execution Loop
    while ~episode_done && episode_steps < env.max_episode_steps
        
        % Select action
        action = agent.select_action(observation, true);  % Training mode with exploration
        
        % Execute action in environment
        [next_observation, reward, episode_done, info] = env.step(action);
        
        % Store experience in replay buffer
        agent.store_experience(observation, action, reward, next_observation, episode_done);
        
        % Training step (if enough experiences)
        if agent.get_buffer_size() >= params.rl.min_buffer_size
            for train_step = 1:params.rl.train_frequency
                agent.train_step();
            end
        end
        
        % Update for next step
        observation = next_observation;
        episode_reward = episode_reward + reward;
        episode_steps = episode_steps + 1;
        
        % Early termination on collision or success
        if info.collision || info.goal_reached
            break;
        end
    end
    
    %% Episode Completion
    episode_time = toc(episode_start_time);
    
    % Update agent's episode tracking
    agent.episode_count = agent.episode_count + 1;
    agent.update_curriculum(episode_reward);
    
    % Store training statistics
    training_stats.episode_rewards(end+1) = episode_reward;
    training_stats.episode_lengths(end+1) = episode_steps;
    
    % Calculate recent success rate
    if episode >= 100
        recent_episodes = training_stats.episode_rewards(end-99:end);
        success_rate = sum(recent_episodes > 0) / 100;  % Simple success metric
        training_stats.success_rates(end+1) = success_rate;
    else
        training_stats.success_rates(end+1) = 0;
    end
    
    %% Logging and Monitoring
    if mod(episode, params.rl.training.log_frequency) == 0
        avg_reward = mean(training_stats.episode_rewards(max(1, end-99):end));
        success_rate = training_stats.success_rates(end);
        
        fprintf('Episode %d: Reward=%.1f, AvgReward=%.1f, Success=%.2f%%, Steps=%d, Time=%.2fs\n', ...
                episode, episode_reward, avg_reward, success_rate*100, episode_steps, episode_time);
        
        if ~isempty(agent.actor_losses)
            fprintf('  Actor Loss: %.4f, Critic Loss: %.4f, Noise: %.3f\n', ...
                    agent.actor_losses(end), agent.critic_losses(end), agent.noise_scale);
        end
    end
    
    %% Evaluation and Model Saving
    if mod(episode, params.rl.training.evaluation_frequency) == 0
        fprintf('\n--- Evaluation at Episode %d ---\n', episode);
        
        % Run evaluation episodes
        eval_score = run_evaluation(env, agent, params);
        training_stats.evaluation_scores(end+1) = eval_score;
        
        fprintf('Evaluation Score: %.2f\n', eval_score);
        
        % Save best model
        if eval_score > best_eval_score
            best_eval_score = eval_score;
            episodes_since_improvement = 0;
            
            % Save best model
            model_filename = sprintf('rl_models/best_model_episode_%d_score_%.2f.mat', ...
                                   episode, eval_score);
            agent.save_model(model_filename);
            fprintf('New best model saved: %s\n', model_filename);
        else
            episodes_since_improvement = episodes_since_improvement + params.rl.training.evaluation_frequency;
        end
        
        % Early stopping if no improvement
        if episodes_since_improvement > 2000  % 20 evaluations without improvement
            fprintf('Early stopping - no improvement for %d episodes\n', episodes_since_improvement);
            break;
        end
        
        fprintf('--- End Evaluation ---\n\n');
    end
    
    %% Periodic Model Checkpointing
    if mod(episode, params.rl.training.save_frequency) == 0
        checkpoint_filename = sprintf('rl_models/checkpoint_episode_%d.mat', episode);
        agent.save_model(checkpoint_filename);
        
        % Save training statistics
        stats_filename = sprintf('rl_logs/training_stats_episode_%d.mat', episode);
        save(stats_filename, 'training_stats', 'params');
        
        fprintf('Checkpoint saved at episode %d\n', episode);
    end
    
    %% Visualization Updates
    if params.rl.visualization.render_training && mod(episode, 50) == 0
        visualize_training_progress(training_stats, episode);
    end
end

%% Training Completion
total_time = toc;
fprintf('\n=== Training Complete ===\n');
fprintf('Total training time: %.2f hours\n', total_time / 3600);
fprintf('Best evaluation score: %.2f\n', best_eval_score);

% Final evaluation
fprintf('\nRunning final evaluation...\n');
final_score = run_evaluation(env, agent, params);
fprintf('Final evaluation score: %.2f\n', final_score);

% Save final model and complete statistics
final_model_filename = sprintf('rl_models/final_model_score_%.2f.mat', final_score);
agent.save_model(final_model_filename);

final_stats_filename = 'rl_logs/final_training_stats.mat';
save(final_stats_filename, 'training_stats', 'params', 'total_time', 'final_score');

% Generate final training report
generate_training_report(training_stats, params, final_score, total_time);

fprintf('Training completed successfully!\n');

%% Helper Functions

function eval_score = run_evaluation(env, agent, params)
    %% Run Evaluation Episodes
    
    eval_episodes = params.rl.training.eval_episodes;
    eval_rewards = zeros(eval_episodes, 1);
    eval_success = zeros(eval_episodes, 1);
    eval_efficiency = zeros(eval_episodes, 1);
    
    % Set agent to evaluation mode (no exploration)
    agent.training_mode = false;
    
    for eval_ep = 1:eval_episodes
        obs = env.reset();
        eval_reward = 0;
        eval_steps = 0;
        eval_done = false;
        
        initial_distance = norm(env.drone_state(1:3) - env.target_waypoints(:, end));
        
        while ~eval_done && eval_steps < env.max_episode_steps
            action = agent.select_action(obs, false);  % No exploration
            [obs, reward, eval_done, info] = env.step(action);
            eval_reward = eval_reward + reward;
            eval_steps = eval_steps + 1;
        end
        
        eval_rewards(eval_ep) = eval_reward;
        eval_success(eval_ep) = info.goal_reached;
        
        % Calculate path efficiency
        if info.goal_reached
            final_distance = norm(env.drone_state(1:3) - env.target_waypoints(:, end));
            path_length = size(env.trajectory_history, 2) * env.params.rl.map_resolution;
            optimal_length = initial_distance;
            eval_efficiency(eval_ep) = optimal_length / max(path_length, optimal_length);
        end
    end
    
    % Restore training mode
    agent.training_mode = true;
    
    % Calculate evaluation score
    avg_reward = mean(eval_rewards);
    success_rate = mean(eval_success);
    avg_efficiency = mean(eval_efficiency(eval_efficiency > 0));
    
    eval_score = avg_reward + 100 * success_rate + 50 * avg_efficiency;
    
    fprintf('  Avg Reward: %.2f, Success Rate: %.2f%%, Avg Efficiency: %.3f\n', ...
            avg_reward, success_rate * 100, avg_efficiency);
end

function visualize_training_progress(training_stats, episode)
    %% Visualize Training Progress
    
    figure(99); clf;
    
    subplot(2, 2, 1);
    plot(training_stats.episode_rewards);
    title('Episode Rewards');
    xlabel('Episode');
    ylabel('Reward');
    grid on;
    
    subplot(2, 2, 2);
    if ~isempty(training_stats.success_rates)
        plot(training_stats.success_rates * 100);
        title('Success Rate (%)');
        xlabel('Episode');
        ylabel('Success Rate');
        grid on;
    end
    
    subplot(2, 2, 3);
    plot(training_stats.episode_lengths);
    title('Episode Length');
    xlabel('Episode');
    ylabel('Steps');
    grid on;
    
    subplot(2, 2, 4);
    if ~isempty(training_stats.evaluation_scores)
        plot(training_stats.evaluation_scores);
        title('Evaluation Scores');
        xlabel('Evaluation');
        ylabel('Score');
        grid on;
    end
    
    sgtitle(sprintf('Training Progress - Episode %d', episode));
    drawnow;
end

function generate_training_report(training_stats, params, final_score, total_time)
    %% Generate Training Report
    
    report_filename = 'rl_logs/training_report.txt';
    fid = fopen(report_filename, 'w');
    
    fprintf(fid, '=== RL Training Report ===\n\n');
    fprintf(fid, 'Training Parameters:\n');
    fprintf(fid, '  Episodes: %d\n', params.rl.training.num_episodes);
    fprintf(fid, '  Learning Rates: Actor=%.4f, Critic=%.4f\n', params.rl.lr_actor, params.rl.lr_critic);
    fprintf(fid, '  Network Architecture: [%s]\n', sprintf('%d ', params.rl.hidden_dims));
    fprintf(fid, '  Curriculum Learning: %s\n', mat2str(params.rl.use_curriculum));
    fprintf(fid, '\n');
    
    fprintf(fid, 'Training Results:\n');
    fprintf(fid, '  Total Time: %.2f hours\n', total_time / 3600);
    fprintf(fid, '  Final Score: %.2f\n', final_score);
    fprintf(fid, '  Average Reward (last 100): %.2f\n', mean(training_stats.episode_rewards(end-99:end)));
    fprintf(fid, '  Final Success Rate: %.2f%%\n', training_stats.success_rates(end) * 100);
    fprintf(fid, '  Total Episodes: %d\n', length(training_stats.episode_rewards));
    fprintf(fid, '\n');
    
    fprintf(fid, 'Performance Statistics:\n');
    fprintf(fid, '  Best Evaluation Score: %.2f\n', max(training_stats.evaluation_scores));
    fprintf(fid, '  Average Episode Length: %.1f steps\n', mean(training_stats.episode_lengths));
    fprintf(fid, '  Convergence Episode: %d\n', find(training_stats.success_rates > 0.8, 1));
    
    fclose(fid);
    fprintf('Training report saved to: %s\n', report_filename);
end

%% train_rl_system_improved.m - Enhanced RL Training with Better Monitoring
% PURPOSE
% Improved RL training script with better reward balancing, curriculum learning,
% and comprehensive monitoring for obstacle avoidance.

function train_rl_system_improved()
    %% Enhanced RL System Training with EKF/UKF State Estimation
    
    fprintf('=== Enhanced RL System Training with EKF/UKF State Estimation ===\n\n');
    
    % Step 1: Setup
    fprintf('Step 1: Setting up training configuration...\n');
    addpath('rl_obstacle_avoidance');
    addpath('.');
    
    % Load parameters
    params = rl_parameters();
    fprintf('✓ RL parameters loaded successfully.\n');
    fprintf('  Environment: %s obstacles, %.1f complexity\n', ...
            params.rl.obstacle_scenario, params.rl.terrain_complexity);
    fprintf('  Agent: DDPG with %s hidden layers\n', mat2str(params.rl.hidden_dims));
    fprintf('  Training: %d episodes with curriculum learning\n', params.rl.training.num_episodes);
    
    % Configuration
    filter_type = 'EKF';  % or 'UKF'
    num_episodes = 2000;  % Reduced for testing
    sensor_mode = 'limited';
    
    fprintf('✓ Configuration loaded\n');
    fprintf('  Filter type: %s (NO ground truth used!)\n', filter_type);
    fprintf('  Episodes: %d\n', num_episodes);
    fprintf('  Sensor mode: %s\n', sensor_mode);
    
    % Step 2: Initialize environment and agent
    fprintf('\nStep 2: Initializing training environment...\n');
    
    % Create environment
    env = rl_environment(params);
    fprintf('✓ RL Environment initialized:\n');
    fprintf('  Observation dim: %d\n', env.observation_dim);
    fprintf('  Action dim: %d\n', env.action_dim);
    fprintf('  Map resolution: %.2f m/cell\n', env.map_resolution);
    fprintf('  Map bounds: [%.1f %.1f %.1f %.1f %.1f %.1f]\n', env.map_bounds);
    
    % Create RL-EKF integration
    rl_ekf = rl_ekf_integration(params);
    fprintf('✓ RL-EKF Integration initialized\n');
    
    % Create agent
    agent = rl_agent(env.observation_dim, env.action_dim, params);
    fprintf('✓ RL Agent initialized:\n');
    fprintf('  Observation dim: %d\n', agent.obs_dim);
    fprintf('  Action dim: %d\n', agent.action_dim);
    fprintf('  Hidden layers: %s\n', mat2str(agent.hidden_dims));
    fprintf('  Buffer size: %d\n', agent.buffer_size);
    fprintf('  Exploration steps: %d\n', agent.exploration_steps);
    
    fprintf('✓ Environment and agent initialized\n');
    
    % Step 3: Training loop with enhanced monitoring
    fprintf('\nStep 3: Starting enhanced RL training loop...\n');
    fprintf('⚠️  IMPORTANT: Using %s state estimation (realistic training conditions)\n', filter_type);
    
    % Training variables
    training_log = struct();
    training_log.episode_rewards = zeros(num_episodes, 1);
    training_log.episode_lengths = zeros(num_episodes, 1);
    training_log.success_rates = zeros(num_episodes, 1);
    training_log.avg_rewards = zeros(num_episodes, 1);
    training_log.collision_rates = zeros(num_episodes, 1);
    training_log.curriculum_level = zeros(num_episodes, 1);
    training_log.exploration_phase = false(num_episodes, 1);
    
    best_reward = -inf;
    success_count = 0;
    
    % Create plots for real-time monitoring
    figure('Position', [100, 100, 1200, 800]);
    
    for episode = 1:num_episodes
        % Reset environment
        obs = env.reset();
        episode_reward = 0;
        episode_steps = 0;
        collision_occurred = false;
        
        % Episode loop
        while true
            % Select action
            action = agent.select_action(obs, true); % With exploration
            
            % Step environment
            [next_obs, reward, done, info] = env.step(action);
            
            % Store experience
            agent.store_experience(obs, action, reward, next_obs, done);
            
            % Update episode tracking
            episode_reward = episode_reward + reward;
            episode_steps = episode_steps + 1;
            
            if info.collision
                collision_occurred = true;
            end
            
            % Training step (only after exploration phase)
            if ~agent.is_in_exploration_phase()
                agent.train_step();
            end
            
            % Update observation
            obs = next_obs;
            
            % Check termination
            if done
                break;
            end
        end
        
        % Update curriculum learning
        agent.update_curriculum(episode_reward);
        
        % Log episode results
        training_log.episode_rewards(episode) = episode_reward;
        training_log.episode_lengths(episode) = episode_steps;
        training_log.curriculum_level(episode) = agent.curriculum_level;
        training_log.exploration_phase(episode) = agent.is_in_exploration_phase();
        
        if info.goal_reached
            success_count = success_count + 1;
        end
        
        training_log.success_rates(episode) = success_count / episode;
        training_log.collision_rates(episode) = sum(training_log.collision_rates(1:episode-1)) + collision_occurred;
        training_log.avg_rewards(episode) = mean(training_log.episode_rewards(1:episode));
        
        % Save best model
        if episode_reward > best_reward
            best_reward = episode_reward;
            agent.save_model('rl_models/best_rl_model.mat');
            fprintf('Episode %4d: Reward=%.1f, AvgReward=%.1f, Success=%.1f%%, Steps=%d, Collision=%d, Level=%d, Time=%.1fs\n', ...
                    episode, episode_reward, training_log.avg_rewards(episode), ...
                    training_log.success_rates(episode)*100, episode_steps, ...
                    collision_occurred, agent.curriculum_level, toc);
        else
            fprintf('Episode %4d: Reward=%.1f, AvgReward=%.1f, Success=%.1f%%, Steps=%d, Collision=%d, Level=%d, Time=%.1fs\n', ...
                    episode, episode_reward, training_log.avg_rewards(episode), ...
                    training_log.success_rates(episode)*100, episode_steps, ...
                    collision_occurred, agent.curriculum_level, toc);
        end
        
        % Periodic model saves
        if mod(episode, 100) == 0
            agent.save_model(sprintf('rl_models/rl_model_episode_%d.mat', episode));
            fprintf('Model saved to: rl_models/rl_model_episode_%d.mat\n', episode);
        end
        
        % Real-time plotting every 50 episodes
        if mod(episode, 50) == 0
            update_training_plots(training_log, episode);
        end
        
        % Early stopping if consistently successful
        if episode > 500 && training_log.success_rates(episode) > 0.8
            fprintf('🎉 Early stopping: Success rate > 80%% for recent episodes\n');
            break;
        end
    end
    
    % Final results
    fprintf('\n=== Training Complete ===\n');
    fprintf('Final Success Rate: %.1f%%\n', training_log.success_rates(end) * 100);
    fprintf('Final Average Reward: %.2f\n', training_log.avg_rewards(end));
    fprintf('Best Episode Reward: %.2f\n', best_reward);
    fprintf('Final Curriculum Level: %d\n', agent.curriculum_level);
    
    % Save final training log
    save('rl_models/training_log.mat', 'training_log', 'params');
    fprintf('Training log saved to: rl_models/training_log.mat\n');
    
    % Generate final plots
    generate_final_plots(training_log);
    
    fprintf('=== Enhanced Training Complete ===\n');
end

function update_training_plots(training_log, episode)
    %% Update real-time training plots
    
    subplot(2, 3, 1);
    plot(1:episode, training_log.episode_rewards(1:episode), 'b-', 'LineWidth', 1);
    hold on;
    plot(1:episode, training_log.avg_rewards(1:episode), 'r-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Reward');
    title('Episode Rewards');
    legend('Episode Reward', 'Average Reward', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 2);
    plot(1:episode, training_log.success_rates(1:episode) * 100, 'g-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Success Rate (%)');
    title('Success Rate');
    ylim([0, 100]);
    grid on;
    
    subplot(2, 3, 3);
    plot(1:episode, training_log.episode_lengths(1:episode), 'm-', 'LineWidth', 1);
    xlabel('Episode');
    ylabel('Episode Length');
    title('Episode Length');
    grid on;
    
    subplot(2, 3, 4);
    plot(1:episode, training_log.curriculum_level(1:episode), 'c-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Curriculum Level');
    title('Curriculum Learning');
    grid on;
    
    subplot(2, 3, 5);
    plot(1:episode, training_log.collision_rates(1:episode), 'r-', 'LineWidth', 1);
    xlabel('Episode');
    ylabel('Collision Count');
    title('Collision Rate');
    grid on;
    
    subplot(2, 3, 6);
    exploration_episodes = find(training_log.exploration_phase(1:episode));
    if ~isempty(exploration_episodes)
        plot(exploration_episodes, ones(size(exploration_episodes)), 'ro', 'MarkerSize', 4);
    end
    xlabel('Episode');
    ylabel('Exploration Phase');
    title('Exploration vs Learning');
    ylim([0, 2]);
    grid on;
    
    drawnow;
end

function generate_final_plots(training_log)
    %% Generate comprehensive final training plots
    
    figure('Position', [200, 200, 1400, 1000]);
    
    % Episode rewards
    subplot(3, 3, 1);
    plot(training_log.episode_rewards, 'b-', 'LineWidth', 1);
    hold on;
    plot(training_log.avg_rewards, 'r-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Reward');
    title('Episode Rewards');
    legend('Episode Reward', 'Average Reward', 'Location', 'best');
    grid on;
    
    % Success rate
    subplot(3, 3, 2);
    plot(training_log.success_rates * 100, 'g-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Success Rate (%)');
    title('Success Rate Over Time');
    ylim([0, 100]);
    grid on;
    
    % Episode lengths
    subplot(3, 3, 3);
    plot(training_log.episode_lengths, 'm-', 'LineWidth', 1);
    xlabel('Episode');
    ylabel('Episode Length');
    title('Episode Length Distribution');
    grid on;
    
    % Curriculum learning
    subplot(3, 3, 4);
    plot(training_log.curriculum_level, 'c-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Curriculum Level');
    title('Curriculum Learning Progress');
    grid on;
    
    % Collision rate
    subplot(3, 3, 5);
    plot(training_log.collision_rates, 'r-', 'LineWidth', 1);
    xlabel('Episode');
    ylabel('Collision Count');
    title('Collision Rate Over Time');
    grid on;
    
    % Reward distribution
    subplot(3, 3, 6);
    histogram(training_log.episode_rewards, 50, 'FaceAlpha', 0.7);
    xlabel('Reward');
    ylabel('Frequency');
    title('Reward Distribution');
    grid on;
    
    % Moving averages
    subplot(3, 3, 7);
    window = 100;
    if length(training_log.episode_rewards) >= window
        moving_avg = movmean(training_log.episode_rewards, window);
        plot(moving_avg, 'k-', 'LineWidth', 2);
        xlabel('Episode');
        ylabel('Moving Average Reward');
        title(sprintf('Moving Average (window=%d)', window));
        grid on;
    end
    
    % Learning progress
    subplot(3, 3, 8);
    plot(training_log.avg_rewards, 'b-', 'LineWidth', 2);
    hold on;
    plot(training_log.success_rates * 100, 'g-', 'LineWidth', 2);
    xlabel('Episode');
    ylabel('Value');
    title('Learning Progress');
    legend('Avg Reward', 'Success Rate (%)', 'Location', 'best');
    grid on;
    
    % Summary statistics
    subplot(3, 3, 9);
    text(0.1, 0.8, sprintf('Final Success Rate: %.1f%%', training_log.success_rates(end)*100), 'FontSize', 12);
    text(0.1, 0.6, sprintf('Final Avg Reward: %.2f', training_log.avg_rewards(end)), 'FontSize', 12);
    text(0.1, 0.4, sprintf('Best Reward: %.2f', max(training_log.episode_rewards)), 'FontSize', 12);
    text(0.1, 0.2, sprintf('Total Episodes: %d', length(training_log.episode_rewards)), 'FontSize', 12);
    axis off;
    title('Training Summary');
    
    sgtitle('Enhanced RL Training Results', 'FontSize', 16);
    
    % Save plots
    saveas(gcf, 'rl_models/enhanced_training_results.png');
    saveas(gcf, 'rl_models/enhanced_training_results.fig');
    fprintf('Training plots saved to: rl_models/enhanced_training_results.png/.fig\n');
end

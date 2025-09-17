%% debug_episode_termination.m - Debug Episode Termination Issues
% PURPOSE
% Debug script to identify why episodes are not terminating properly

function debug_episode_termination()
    fprintf('=== Episode Termination Debug ===\n');
    
    % Load parameters
    addpath('rl_obstacle_avoidance');
    addpath('.');
    params = rl_parameters();
    
    % Create environment
    env = rl_environment(params);
    agent = rl_agent(env.observation_dim, env.action_dim, params);
    
    fprintf('Environment setup:\n');
    fprintf('  Max episode steps: %d\n', env.max_episode_steps);
    fprintf('  Success distance: %.1f m\n', env.params.rl.success_distance);
    fprintf('  Map bounds: [%.1f %.1f %.1f %.1f %.1f %.1f]\n', env.map_bounds);
    
    % Test single episode with detailed logging
    fprintf('\nTesting single episode with detailed logging...\n');
    
    obs = env.reset();
    episode_reward = 0;
    step_count = 0;
    max_debug_steps = 100;  % Limit for debugging
    
    fprintf('Episode reset. Start: [%.1f %.1f %.1f], Goal: [%.1f %.1f %.1f]\n', ...
            env.drone_state(1), env.drone_state(2), env.drone_state(3), ...
            env.target_waypoints(1,end), env.target_waypoints(2,end), env.target_waypoints(3,end));
    
    while step_count < max_debug_steps
        step_count = step_count + 1;
        
        % Select action
        action = agent.select_action(obs, true);
        
        % Step environment
        [next_obs, reward, done, info] = env.step(action);
        
        % Log details every 10 steps
        if mod(step_count, 10) == 0 || done
            fprintf('Step %3d: Reward=%.2f, Done=%d, Collision=%d, Goal=%d, Steps=%d, Dist=%.2f\n', ...
                    step_count, reward, done, info.collision, info.goal_reached, ...
                    info.episode_step, info.distance_to_goal);
        end
        
        % Store experience
        agent.store_experience(obs, action, reward, next_obs, done);
        
        episode_reward = episode_reward + reward;
        obs = next_obs;
        
        % Check termination
        if done
            fprintf('Episode completed at step %d with reward %.2f\n', step_count, episode_reward);
            break;
        end
    end
    
    if step_count >= max_debug_steps
        fprintf('Episode did not complete within %d steps (max: %d)\n', max_debug_steps, env.max_episode_steps);
        fprintf('Current state: [%.1f %.1f %.1f]\n', env.drone_state(1), env.drone_state(2), env.drone_state(3));
        fprintf('Distance to goal: %.2f m\n', info.distance_to_goal);
        fprintf('Episode step: %d\n', info.episode_step);
    end
    
    fprintf('=== Debug Complete ===\n');
end

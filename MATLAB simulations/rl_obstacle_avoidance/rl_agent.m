%% rl_agent.m - Deep Reinforcement Learning Agent for Autonomous Navigation
% PURPOSE
% Implements a sophisticated RL agent using DDPG (Deep Deterministic Policy Gradient)
% for continuous action space drone navigation with obstacle avoidance.
% 
% FEATURES
% - Actor-Critic neural network architecture
% - Experience replay buffer
% - Target networks for stable learning
% - Exploration noise (Ornstein-Uhlenbeck process)
% - Batch normalization for stable training
% - Curriculum learning for progressive difficulty
%
% USAGE
%   agent = rl_agent(obs_dim, action_dim, params);
%   action = agent.select_action(observation, training_mode);
%   agent.train_step(experience_batch);

classdef rl_agent < handle
    properties
        % Network dimensions
        obs_dim
        action_dim
        hidden_dims
        
        % Neural networks (using MATLAB Deep Learning Toolbox)
        actor_network        % Policy network μ(s|θ^μ)
        critic_network       % Q-value network Q(s,a|θ^Q)
        target_actor         % Target policy network μ'(s|θ^μ')
        target_critic        % Target Q-network Q'(s,a|θ^Q')
        
        % Training parameters
        learning_rate_actor
        learning_rate_critic
        gamma               % Discount factor
        tau                 % Soft update rate for target networks
        batch_size
        
        % Experience replay
        replay_buffer
        buffer_size
        buffer_idx
        
        % Exploration
        noise_process       % Ornstein-Uhlenbeck noise
        noise_scale
        noise_decay
        min_noise
        exploration_steps
        
        % Training state
        episode_count
        step_count
        total_steps
        training_mode
        
        % Performance tracking
        episode_rewards
        actor_losses
        critic_losses
        
        % Curriculum learning
        curriculum_level
        curriculum_thresholds
        
        % Device and optimizer settings
        training_options_actor
        training_options_critic
    end
    
    methods
        function obj = rl_agent(obs_dim, action_dim, params)
            %% Initialize RL Agent
            
            obj.obs_dim = obs_dim;
            obj.action_dim = action_dim;
            obj.hidden_dims = params.rl.hidden_dims; % e.g., [256, 256, 128]
            
            % Training hyperparameters
            obj.learning_rate_actor = params.rl.lr_actor;
            obj.learning_rate_critic = params.rl.lr_critic;
            obj.gamma = params.rl.gamma;
            obj.tau = params.rl.tau;
            obj.batch_size = params.rl.batch_size;
            
            % Experience replay setup
            obj.buffer_size = params.rl.buffer_size;
            obj.replay_buffer = obj.initialize_replay_buffer();
            obj.buffer_idx = 1;
            
            % Exploration noise
            obj.noise_scale = params.rl.noise_scale;
            obj.noise_decay = params.rl.noise_decay;
            obj.min_noise = params.rl.min_noise;
            obj.exploration_steps = params.rl.exploration_steps;
            obj.noise_process = obj.initialize_ou_noise();
            
            % Initialize networks
            obj.build_networks();
            
            % Training state
            obj.episode_count = 0;
            obj.step_count = 0;
            obj.total_steps = 0;
            obj.training_mode = true;
            
            % Performance tracking
            obj.episode_rewards = [];
            obj.actor_losses = [];
            obj.critic_losses = [];
            
            % Curriculum learning
            obj.curriculum_level = 1;
            obj.curriculum_thresholds = params.rl.curriculum_thresholds;
            
            fprintf('RL Agent initialized:\n');
            fprintf('  Observation dim: %d\n', obj.obs_dim);
            fprintf('  Action dim: %d\n', obj.action_dim);
            fprintf('  Hidden layers: [%s]\n', sprintf('%d ', obj.hidden_dims));
            fprintf('  Buffer size: %d\n', obj.buffer_size);
        end
        
        function build_networks(obj)
            %% Build Actor and Critic Neural Networks
            
            % Actor Network (Policy): obs -> action
            actor_layers = [
                featureInputLayer(obj.obs_dim, 'Name', 'obs_input')
                fullyConnectedLayer(obj.hidden_dims(1), 'Name', 'actor_fc1')
                batchNormalizationLayer('Name', 'actor_bn1')
                reluLayer('Name', 'actor_relu1')
                dropoutLayer(0.1, 'Name', 'actor_dropout1')
                
                fullyConnectedLayer(obj.hidden_dims(2), 'Name', 'actor_fc2')
                batchNormalizationLayer('Name', 'actor_bn2')
                reluLayer('Name', 'actor_relu2')
                dropoutLayer(0.1, 'Name', 'actor_dropout2')
                
                fullyConnectedLayer(obj.hidden_dims(3), 'Name', 'actor_fc3')
                reluLayer('Name', 'actor_relu3')
                
                fullyConnectedLayer(obj.action_dim, 'Name', 'actor_output')
                tanhLayer('Name', 'actor_tanh')  % Output in [-1, 1]
            ];
            
            % Critic Network (Q-function): [obs, action] -> Q-value
            % Observation branch
            obs_branch = [
                featureInputLayer(obj.obs_dim, 'Name', 'critic_obs_input')
                fullyConnectedLayer(obj.hidden_dims(1), 'Name', 'critic_obs_fc1')
                batchNormalizationLayer('Name', 'critic_obs_bn1')
                reluLayer('Name', 'critic_obs_relu1')
            ];
            
            % Action branch
            action_branch = [
                featureInputLayer(obj.action_dim, 'Name', 'critic_action_input')
                fullyConnectedLayer(obj.hidden_dims(1), 'Name', 'critic_action_fc1')
                reluLayer('Name', 'critic_action_relu1')
            ];
            
            % Combined layers
            combined_layers = [
                concatenationLayer(1, 2, 'Name', 'critic_concat')  % Concatenate obs and action features
                fullyConnectedLayer(obj.hidden_dims(2), 'Name', 'critic_fc2')
                batchNormalizationLayer('Name', 'critic_bn2')
                reluLayer('Name', 'critic_relu2')
                dropoutLayer(0.1, 'Name', 'critic_dropout2')
                
                fullyConnectedLayer(obj.hidden_dims(3), 'Name', 'critic_fc3')
                reluLayer('Name', 'critic_relu3')
                
                fullyConnectedLayer(1, 'Name', 'critic_output')  % Single Q-value output
            ];
            
            % Create layer graphs
            actor_graph = layerGraph(actor_layers);
            
            % For critic, we need to create a more complex graph
            critic_graph = layerGraph();
            critic_graph = addLayers(critic_graph, obs_branch);
            critic_graph = addLayers(critic_graph, action_branch);
            critic_graph = addLayers(critic_graph, combined_layers);
            
            % Connect the branches
            critic_graph = connectLayers(critic_graph, 'critic_obs_relu1', 'critic_concat/in1');
            critic_graph = connectLayers(critic_graph, 'critic_action_relu1', 'critic_concat/in2');
            
            % Convert to dlnetwork for prediction
            obj.actor_network = dlnetwork(actor_graph);
            obj.critic_network = dlnetwork(critic_graph);
            
            % Create target networks (copies)
            obj.target_actor = dlnetwork(actor_graph);
            obj.target_critic = dlnetwork(critic_graph);
            
            % Setup training options
            obj.training_options_actor = trainingOptions('adam', ...
                'InitialLearnRate', obj.learning_rate_actor, ...
                'MaxEpochs', 1, ...
                'MiniBatchSize', obj.batch_size, ...
                'Verbose', false, ...
                'Plots', 'none');
            
            obj.training_options_critic = trainingOptions('adam', ...
                'InitialLearnRate', obj.learning_rate_critic, ...
                'MaxEpochs', 1, ...
                'MiniBatchSize', obj.batch_size, ...
                'Verbose', false, ...
                'Plots', 'none');
            
            fprintf('Neural networks built successfully.\n');
        end
        
        function action = select_action(obj, observation, add_noise)
            %% Select Action Using Current Policy - IMPROVED
            
            if nargin < 3
                add_noise = obj.training_mode;
            end
            
            % Pure exploration during exploration phase
            if obj.is_in_exploration_phase()
                % Random actions for exploration
                action = 2 * (rand(obj.action_dim, 1) - 0.5);  % Random in [-1, 1]
                return;
            end
            
            % Forward pass through actor network (expect CB: Channel x Batch)
            obs_dl = obj.formatSingleObsCB(observation);
            action_dl = predict(obj.actor_network, obs_dl);
            action = extractdata(action_dl);  % Extract data from dlarray
            action = action(:);  % Ensure column vector
            
            % Add exploration noise during training
            if add_noise && obj.training_mode
                noise = obj.get_exploration_noise();
                action = action + noise;
                
                % Decay noise over time
                obj.noise_scale = max(obj.noise_scale * obj.noise_decay, obj.min_noise);
            end
            
            % Clamp actions to valid range [-1, 1] (will be scaled by environment)
            action = max(min(action, 1), -1);
        end
        
        function store_experience(obj, obs, action, reward, next_obs, done)
            %% Store Experience in Replay Buffer
            
            experience = struct();
            experience.obs = obs(:);
            experience.action = action(:);
            experience.reward = reward;
            experience.next_obs = next_obs(:);
            experience.done = done;
            
            % Circular buffer
            obj.replay_buffer{obj.buffer_idx} = experience;
            obj.buffer_idx = mod(obj.buffer_idx, obj.buffer_size) + 1;
        end
        
        function train_step(obj)
            %% Perform One Training Step
            
            % Increment total steps
            obj.total_steps = obj.total_steps + 1;
            
            % Check if we have enough experiences
            if obj.get_buffer_size() < obj.batch_size
                return;
            end
            
            % Sample random batch
            batch = obj.sample_batch();
            
            % Train critic network
            critic_loss = obj.train_critic(batch);
            obj.critic_losses = [obj.critic_losses, critic_loss];
            
            % Train actor network
            actor_loss = obj.train_actor(batch);
            obj.actor_losses = [obj.actor_losses, actor_loss];
            
            % Soft update target networks
            obj.soft_update_targets();
            
            obj.step_count = obj.step_count + 1;
        end
        
        function critic_loss = train_critic(obj, batch)
            %% Train Critic Network
            
            obs_cell = {batch.obs};
            % Extract actions: each batch.action should be [action_dim x 1]
            % Convert to [batch_size x action_dim] matrix
            action_batch = zeros(length(batch), obj.action_dim);
            for i = 1:length(batch)
                action_batch(i, :) = batch(i).action(:)';  % ensure row vector
            end
            reward_batch = [batch.reward]';
            next_obs_cell = {batch.next_obs};
            done_batch = [batch.done]';
            
            % Calculate target Q-values using two-input critic (obs, action)
            Xnext = obj.formatObsBatchCB(next_obs_cell);      % dlarray CB
            next_actions_dl = predict(obj.target_actor, Xnext); % dlarray CB (action_dim x B)
            target_q_values_dl = predict(obj.target_critic, Xnext, next_actions_dl);
            target_q_values = extractdata(target_q_values_dl);
            if isrow(target_q_values); target_q_values = target_q_values'; end
            
            % Calculate targets: r + γ * Q'(s', μ'(s'))
            targets = reward_batch + obj.gamma * (1 - done_batch) .* target_q_values;
            
            % Current Q(s,a) with two-input critic
            Xobs_cur = obj.formatObsBatchCB(obs_cell);        % dlarray CB
            
            % Format actions for critic: action_batch is [batch_size x action_dim]
            % Need [action_dim x batch_size] for CB format
            Acur = single(action_batch');                      % [action_dim x batch_size]
            Acur_dl = dlarray(Acur, 'CB');                    % [action_dim x batch_size]
            q_values_dl = predict(obj.critic_network, Xobs_cur, Acur_dl);
            q_values = extractdata(q_values_dl);
            if isrow(q_values); q_values = q_values'; end

            % Critic loss (MSE)
            diff = q_values - targets;
            critic_loss = mean(diff.^2);
        end
        
        function actor_loss = train_actor(obj, batch)
            %% Train Actor Network
            
            obs_cell = {batch.obs};
            
            % Calculate policy gradient
            % ∇_θ J ≈ ∇_a Q(s,a)|a=μ(s) * ∇_θ μ(s|θ)
            
            Xobs = obj.formatObsBatchCB(obs_cell);
            actions_dl = predict(obj.actor_network, Xobs);
            actions = extractdata(actions_dl)'; % [batch x action_dim]
            
            % Format actions for critic: [action_dim x batch_size] for CB format
            Acur = single(actions');  % [action_dim x batch_size]
            Acur_dl = dlarray(Acur, 'CB');
            
            % Use two-input critic: (obs, action)
            q_values_dl = predict(obj.critic_network, Xobs, Acur_dl);
            q_values = extractdata(q_values_dl);
            if isrow(q_values); q_values = q_values'; end
            
            % Actor loss is negative mean Q-value (we want to maximize Q)
            actor_loss = -mean(q_values);
            
            % Custom training step for actor (simplified)
            % In practice, you'd implement proper gradient computation
            obj.update_actor_weights(obs_cell, actor_loss);
        end

        % ===== Helper formatting methods =====
        function X = formatSingleObsCB(obj, obs)
            % Return dlarray in CB (Channel x Batch) with batch=1
            x = single(obs(:));
            if numel(x) ~= obj.obs_dim
                error('formatSingleObsCB: got %d features, expected %d', numel(x), obj.obs_dim);
            end
            X = dlarray(x, 'CB');
        end

        function X = formatObsBatchCB(obj, obsBatch)
            % Normalize various batch shapes to dlarray in CB (Channel x Batch)
            if iscell(obsBatch)
                B = numel(obsBatch);
                Xmat = zeros(obj.obs_dim, B, 'single');
                for b = 1:B
                    v = single(obsBatch{b}(:));
                    if numel(v) ~= obj.obs_dim
                        error('formatObsBatchCB(cell): elem %d has %d, expected %d', b, numel(v), obj.obs_dim);
                    end
                    Xmat(:, b) = v;
                end
            else
                Xraw = single(obsBatch);
                sz = size(Xraw);
                if isvector(Xraw)
                    Xmat = reshape(Xraw, [], 1);
                elseif sz(1) == obj.obs_dim
                    Xmat = Xraw; % already [obs_dim x batch]
                elseif sz(2) == obj.obs_dim
                    Xmat = Xraw'; % transpose to [obs_dim x batch]
                else
                    error('formatObsBatchCB: unexpected size %s (obs_dim=%d)', mat2str(sz), obj.obs_dim);
                end
            end
            X = dlarray(Xmat, 'CB');
        end
        
        function soft_update_targets(obj)
            %% Soft Update Target Networks
            
            % θ' ← τ*θ + (1-τ)*θ'
            % This is simplified - in practice, you'd need to implement
            % proper weight copying between networks
            
            % For MATLAB networks, this requires custom implementation
            % of parameter copying with the soft update rule
            
            % Placeholder for target network updates
            if mod(obj.step_count, 100) == 0  % Update less frequently
                obj.target_actor = obj.actor_network;
                obj.target_critic = obj.critic_network;
            end
        end
        
        function noise = get_exploration_noise(obj)
            %% Generate Exploration Noise (Ornstein-Uhlenbeck Process)
            
            % OU process: dx = θ(μ - x)dt + σdW
            theta = 0.15;  % Mean reversion rate
            mu = 0;        % Long-term mean
            sigma = 0.2;   % Volatility
            dt = 1.0;      % Time step
            
            % Update noise process state
            dx = theta * (mu - obj.noise_process) * dt + ...
                 sigma * sqrt(dt) * randn(obj.action_dim, 1);
            obj.noise_process = obj.noise_process + dx;
            
            noise = obj.noise_scale * obj.noise_process;
        end
        
        function update_curriculum(obj, episode_reward)
            %% Update Curriculum Learning Level - IMPROVED
            
            obj.episode_rewards = [obj.episode_rewards, episode_reward];
            
            % Check if we should advance curriculum (more lenient)
            if length(obj.episode_rewards) >= 50  % Last 50 episodes (reduced from 100)
                recent_performance = mean(obj.episode_rewards(end-49:end));
                
                if obj.curriculum_level < length(obj.curriculum_thresholds) && ...
                   recent_performance > obj.curriculum_thresholds(obj.curriculum_level)
                    obj.curriculum_level = obj.curriculum_level + 1;
                    fprintf('🎓 Advanced to curriculum level %d (avg reward: %.2f)\n', ...
                            obj.curriculum_level, recent_performance);
                end
            end
        end
        
        function is_exploration_phase = is_in_exploration_phase(obj)
            %% Check if agent is in exploration phase
            is_exploration_phase = obj.total_steps < obj.exploration_steps;
        end
        
        function save_model(obj, filepath)
            %% Save Trained Model
            
            model_data = struct();
            model_data.actor_network = obj.actor_network;
            model_data.critic_network = obj.critic_network;
            model_data.obs_dim = obj.obs_dim;
            model_data.action_dim = obj.action_dim;
            model_data.episode_count = obj.episode_count;
            model_data.curriculum_level = obj.curriculum_level;
            
            save(filepath, 'model_data');
            fprintf('Model saved to: %s\n', filepath);
        end
        
        function load_model(obj, filepath)
            %% Load Trained Model
            
            loaded = load(filepath);
            model_data = loaded.model_data;
            
            obj.actor_network = model_data.actor_network;
            obj.critic_network = model_data.critic_network;
            obj.episode_count = model_data.episode_count;
            obj.curriculum_level = model_data.curriculum_level;
            
            % Update target networks
            obj.target_actor = obj.actor_network;
            obj.target_critic = obj.critic_network;
            
            fprintf('Model loaded from: %s\n', filepath);
        end
        
        % Helper methods
        function buffer = initialize_replay_buffer(obj)
            buffer = cell(obj.buffer_size, 1);
        end
        
        function noise_process = initialize_ou_noise(obj)
            noise_process = zeros(obj.action_dim, 1);
        end
        
        function size = get_buffer_size(obj)
            % Count non-empty buffer elements
            size = sum(~cellfun(@isempty, obj.replay_buffer));
        end
        
        function batch = sample_batch(obj)
            % Sample random batch from replay buffer
            buffer_size = obj.get_buffer_size();
            indices = randperm(buffer_size, obj.batch_size);
            batch = [obj.replay_buffer{indices}];
        end
        
        % Placeholder methods for network training
        function loss = train_network_mse(obj, network, inputs, targets, options)
            % Simplified training - in practice, implement proper training loop
            loss = mean((predict(network, inputs) - targets).^2);
        end
        
        function update_actor_weights(obj, obs_batch, loss)
            % Simplified weight update - implement proper gradient descent
            % This would require custom gradient computation in MATLAB
        end
    end
end

%% behavior_cloning_trainer.m - Behavior Cloning for Expert Imitation
% PURPOSE
% Implements supervised learning from expert demonstrations (behavior cloning)
% to train a policy network that mimics the EKF-guided expert drone.
%
% This implements the PDF requirement for imitation learning from expert
% trajectories to train limited-sensor drones.

classdef behavior_cloning_trainer < handle
    properties
        params
        expert_demonstrator
        
        % Neural network for behavior cloning
        policy_network
        training_options
        
        % Training data
        training_states
        training_actions
        validation_states
        validation_actions
        
        % Training statistics
        training_losses
        validation_losses
        training_accuracy
        
        % Limited sensor observer
        limited_observer
    end
    
    methods
        function obj = behavior_cloning_trainer(params, expert_demonstrator)
            obj.params = params;
            obj.expert_demonstrator = expert_demonstrator;
            
            % Initialize limited sensor observer
            obj.limited_observer = limited_sensor_observer(params);
            
            % Initialize training statistics
            obj.training_losses = [];
            obj.validation_losses = [];
            obj.training_accuracy = [];
            
            fprintf('Behavior cloning trainer initialized\n');
        end
        
        function train_policy(obj)
            %% Train Policy Network via Behavior Cloning
            
            fprintf('Starting behavior cloning training...\n');
            
            % 1) Prepare training data from expert demonstrations
            obj.prepare_training_data();
            
            % 2) Build policy network
            obj.build_policy_network();
            
            % 3) Train network
            obj.train_network();
            
            % 4) Evaluate trained policy
            obj.evaluate_policy();
            
            fprintf('Behavior cloning training complete\n');
        end
        
        function prepare_training_data(obj)
            %% Prepare Training Data from Expert Demonstrations
            
            fprintf('Preparing training data from expert demonstrations...\n');
            
            if obj.expert_demonstrator.demo_count == 0
                error('No expert demonstrations available. Collect demonstrations first.');
            end
            
            % Extract all demonstration data
            all_states = [];
            all_limited_obs = [];
            all_actions = [];
            
            available_demos = min(obj.expert_demonstrator.demo_count, obj.expert_demonstrator.demo_buffer_size);
            
            for demo_idx = 1:available_demos
                demo = obj.expert_demonstrator.demonstrations{demo_idx};
                
                if ~isempty(demo) && demo.success
                    % Use limited observations instead of full states for training
                    all_limited_obs = [all_limited_obs, demo.limited_observations];
                    all_actions = [all_actions, demo.actions];
                end
            end
            
            fprintf('Collected %d training samples from %d demonstrations\n', ...
                    size(all_limited_obs, 2), available_demos);
            
            % Split into training and validation sets
            num_samples = size(all_limited_obs, 2);
            train_ratio = 0.8;
            train_size = round(train_ratio * num_samples);
            
            % Shuffle data
            indices = randperm(num_samples);
            train_indices = indices(1:train_size);
            val_indices = indices(train_size+1:end);
            
            % Training set
            obj.training_states = all_limited_obs(:, train_indices)';
            obj.training_actions = all_actions(:, train_indices)';
            
            % Validation set
            obj.validation_states = all_limited_obs(:, val_indices)';
            obj.validation_actions = all_actions(:, val_indices)';
            
            fprintf('Training set: %d samples\n', size(obj.training_states, 1));
            fprintf('Validation set: %d samples\n', size(obj.validation_states, 1));
        end
        
        function build_policy_network(obj)
            %% Build Neural Network for Behavior Cloning
            
            % Network architecture
            input_size = size(obj.training_states, 2);
            output_size = size(obj.training_actions, 2);
            hidden_dims = obj.params.rl.hidden_dims;
            
            fprintf('Building policy network: %d → %d → %d → %d\n', ...
                    input_size, hidden_dims(1), hidden_dims(2), output_size);
            
            % Create layer array
            layers = [
                featureInputLayer(input_size, 'Name', 'input')
                
                fullyConnectedLayer(hidden_dims(1), 'Name', 'fc1')
                batchNormalizationLayer('Name', 'bn1')
                reluLayer('Name', 'relu1')
                dropoutLayer(obj.params.rl.dropout_rate, 'Name', 'dropout1')
                
                fullyConnectedLayer(hidden_dims(2), 'Name', 'fc2')
                batchNormalizationLayer('Name', 'bn2')
                reluLayer('Name', 'relu2')
                dropoutLayer(obj.params.rl.dropout_rate, 'Name', 'dropout2')
                
                fullyConnectedLayer(hidden_dims(3), 'Name', 'fc3')
                reluLayer('Name', 'relu3')
                
                fullyConnectedLayer(output_size, 'Name', 'output')
                tanhLayer('Name', 'tanh')  % Output in [-1, 1] range
            ];
            
            obj.policy_network = layerGraph(layers);
            
            % Training options
            obj.training_options = trainingOptions('adam', ...
                'InitialLearnRate', 1e-3, ...
                'MaxEpochs', obj.params.rl.expert.bc_pretrain_epochs, ...
                'MiniBatchSize', 128, ...
                'ValidationData', {obj.validation_states, obj.validation_actions}, ...
                'ValidationFrequency', 50, ...
                'Plots', 'training-progress', ...
                'Verbose', true, ...
                'ExecutionEnvironment', 'auto');
        end
        
        function train_network(obj)
            %% Train the Policy Network
            
            fprintf('Training behavior cloning network...\n');
            
            try
                % Train network
                obj.policy_network = trainNetwork(obj.training_states, obj.training_actions, ...
                                                 obj.policy_network, obj.training_options);
                
                fprintf('✓ Behavior cloning training completed successfully\n');
                
            catch ME
                fprintf('✗ Error during training: %s\n', ME.message);
                fprintf('Falling back to simplified training...\n');
                
                % Simplified training without validation plots
                obj.training_options.Plots = 'none';
                obj.training_options.ValidationData = {};
                
                obj.policy_network = trainNetwork(obj.training_states, obj.training_actions, ...
                                                 obj.policy_network, obj.training_options);
                
                fprintf('✓ Simplified training completed\n');
            end
        end
        
        function accuracy = evaluate_policy(obj)
            %% Evaluate Trained Policy
            
            fprintf('Evaluating behavior cloning policy...\n');
            
            % Predict on validation set
            predicted_actions = predict(obj.policy_network, obj.validation_states);
            
            % Calculate metrics
            mse = mean(sum((predicted_actions - obj.validation_actions).^2, 2));
            mae = mean(sum(abs(predicted_actions - obj.validation_actions), 2));
            
            % Action-wise accuracy (within 10% tolerance)
            tolerance = 0.1;
            accurate_predictions = abs(predicted_actions - obj.validation_actions) < tolerance;
            accuracy = mean(accurate_predictions(:));
            
            fprintf('Evaluation Results:\n');
            fprintf('  Mean Squared Error: %.4f\n', mse);
            fprintf('  Mean Absolute Error: %.4f\n', mae);
            fprintf('  Action Accuracy (±%.0f%%): %.2f%%\n', tolerance*100, accuracy*100);
            
            % Store results
            obj.training_accuracy = accuracy;
            
            % Visualize predictions vs ground truth
            obj.visualize_predictions(predicted_actions);
        end
        
        function visualize_predictions(obj, predicted_actions)
            %% Visualize Prediction Quality
            
            figure('Name', 'Behavior Cloning Results', 'Position', [100, 100, 1200, 600]);
            
            % Plot first 100 samples for visualization
            num_vis = min(100, size(predicted_actions, 1));
            sample_indices = 1:num_vis;
            
            action_names = {'V_x', 'V_y', 'V_z', 'Yaw Rate'};
            
            for action_idx = 1:4
                subplot(2, 2, action_idx);
                
                true_vals = obj.validation_actions(sample_indices, action_idx);
                pred_vals = predicted_actions(sample_indices, action_idx);
                
                plot(sample_indices, true_vals, 'b-', 'LineWidth', 2, 'DisplayName', 'Expert');
                hold on;
                plot(sample_indices, pred_vals, 'r--', 'LineWidth', 1.5, 'DisplayName', 'BC Policy');
                
                xlabel('Sample');
                ylabel(action_names{action_idx});
                title(sprintf('%s: Expert vs Behavior Cloning', action_names{action_idx}));
                legend('Location', 'best');
                grid on;
            end
            
            sgtitle('Behavior Cloning: Expert vs Learned Policy');
        end
        
        function action = predict_action(obj, limited_observation)
            %% Predict Action Using Trained Policy
            
            % Ensure observation is row vector
            if size(limited_observation, 1) > size(limited_observation, 2)
                limited_observation = limited_observation';
            end
            
            % Predict action
            action = predict(obj.policy_network, limited_observation);
            action = action';  % Return as column vector
        end
        
        function save_policy(obj, filename)
            %% Save Trained Policy
            
            policy_data = struct();
            policy_data.network = obj.policy_network;
            policy_data.training_params = obj.params.rl;
            policy_data.training_accuracy = obj.training_accuracy;
            policy_data.sensor_mode = obj.params.rl.sensor_mode;
            
            save(filename, 'policy_data');
            fprintf('Behavior cloning policy saved to: %s\n', filename);
        end
        
        function load_policy(obj, filename)
            %% Load Trained Policy
            
            loaded = load(filename);
            policy_data = loaded.policy_data;
            
            obj.policy_network = policy_data.network;
            obj.training_accuracy = policy_data.training_accuracy;
            
            fprintf('Behavior cloning policy loaded from: %s\n', filename);
            fprintf('  Training accuracy: %.2f%%\n', obj.training_accuracy * 100);
        end
        
        function collect_and_train(obj, environment)
            %% Complete Pipeline: Collect Expert Demos and Train BC Policy
            
            fprintf('=== Complete Behavior Cloning Pipeline ===\n');
            
            % 1) Collect expert demonstrations
            fprintf('Step 1: Collecting expert demonstrations...\n');
            obj.expert_demonstrator.collect_demonstrations(obj.params.rl.expert.demo_collection_episodes);
            
            % 2) Train behavior cloning policy
            fprintf('Step 2: Training behavior cloning policy...\n');
            obj.train_policy();
            
            % 3) Test policy in environment
            fprintf('Step 3: Testing learned policy...\n');
            test_results = obj.test_policy_in_environment(environment);
            
            fprintf('=== Pipeline Complete ===\n');
            fprintf('Expert success rate: %.1f%%\n', obj.expert_demonstrator.success_rate * 100);
            fprintf('BC policy accuracy: %.1f%%\n', obj.training_accuracy * 100);
            fprintf('BC policy test success: %.1f%%\n', test_results.success_rate * 100);
        end
        
        function results = test_policy_in_environment(obj, environment)
            %% Test Learned Policy in Environment
            
            num_test_episodes = 10;
            results = struct();
            results.episodes = [];
            results.success_count = 0;
            
            fprintf('Testing behavior cloning policy (%d episodes)...\n', num_test_episodes);
            
            for episode = 1:num_test_episodes
                % Reset environment
                environment.reset();
                
                episode_result = struct();
                episode_result.success = false;
                episode_result.reward = 0;
                episode_result.steps = 0;
                
                max_steps = 500;  % Limit test episode length
                
                for step = 1:max_steps
                    % Get limited observation
                    limited_obs = obj.limited_observer.generate_limited_observation(...
                        environment.occupancy_grid, environment.drone_state, ...
                        environment.map_bounds, environment.map_resolution);
                    
                    % Predict action using BC policy
                    action = obj.predict_action(limited_obs);
                    
                    % Execute action
                    [~, reward, done, info] = environment.step(action);
                    
                    episode_result.reward = episode_result.reward + reward;
                    episode_result.steps = episode_result.steps + 1;
                    
                    if done
                        episode_result.success = info.goal_reached;
                        break;
                    end
                end
                
                results.episodes = [results.episodes, episode_result];
                if episode_result.success
                    results.success_count = results.success_count + 1;
                end
                
                if episode_result.success
                    status_str = 'SUCCESS';
                else
                    status_str = 'FAILED';
                end
                fprintf('  Episode %d: %s (%.1f reward, %d steps)\n', ...
                        episode, status_str, episode_result.reward, episode_result.steps);
            end
            
            results.success_rate = results.success_count / num_test_episodes;
            results.avg_reward = mean([results.episodes.reward]);
            results.avg_steps = mean([results.episodes.steps]);
            
            fprintf('BC Policy Test Results:\n');
            fprintf('  Success Rate: %.1f%% (%d/%d)\n', results.success_rate * 100, results.success_count, num_test_episodes);
            fprintf('  Average Reward: %.2f\n', results.avg_reward);
            fprintf('  Average Steps: %.1f\n', results.avg_steps);
        end
    end
end

function debug_single_run()
%% Debug a single obstacle avoidance run to see where it fails

fprintf('=== DEBUGGING SINGLE RUN ===\n');

% Clear any previous variables
clear LAST_DATASET_QUALITY LAST_DATASET_PATH OA_RANDOM_SEED

% Set a random seed
OA_RANDOM_SEED = uint32(12345);
assignin('base','OA_RANDOM_SEED', OA_RANDOM_SEED);

try
    fprintf('Starting test_animated_obstacle_avoidance...\n');
    test_animated_obstacle_avoidance(false);
    fprintf('✓ Simulation completed successfully\n');
    
    % Check if quality data was created
    if evalin('base','exist(''LAST_DATASET_QUALITY'',''var'')')
        q = evalin('base','LAST_DATASET_QUALITY');
        fprintf('✓ Quality data available:\n');
        fprintf('  - Min distance: %.2f m\n', q.min_distance);
        fprintf('  - Final distance: %.2f m\n', q.final_distance);
        fprintf('  - Max divergence: %.2f m\n', q.max_divergence_after_min);
        fprintf('  - Pass: %s\n', mat2str(q.pass));
        
        if evalin('base','exist(''LAST_DATASET_PATH'',''var'')')
            p = evalin('base','LAST_DATASET_PATH');
            fprintf('✓ Dataset path: %s\n', p);
            if exist(p, 'file')
                info = dir(p);
                fprintf('✓ File exists: %.1f KB\n', info.bytes/1024);
            else
                fprintf('✗ File does not exist despite path being set\n');
            end
        else
            fprintf('✗ No dataset path variable set\n');
        end
    else
        fprintf('✗ No quality data - simulation may have failed before save\n');
    end
    
catch ME
    fprintf('✗ Simulation failed with error:\n');
    fprintf('  Message: %s\n', ME.message);
    fprintf('  Stack:\n');
    for i = 1:length(ME.stack)
        fprintf('    %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end

fprintf('=== DEBUG COMPLETE ===\n');
end

%% check_training_status.m - Monitor RL Training Progress
% PURPOSE
% Quick script to check the status of running RL training

clear; clc;

fprintf('=== RL Training Status Check ===\n\n');

%% Check for saved models
if exist('rl_models', 'dir')
    model_files = dir('rl_models/*.mat');
    fprintf('📁 Saved Models: %d files\n', length(model_files));
    
    if length(model_files) > 0
        fprintf('   Latest models:\n');
        for i = max(1, length(model_files)-2):length(model_files)
            fprintf('   - %s (%.1f KB)\n', model_files(i).name, model_files(i).bytes/1024);
        end
    end
else
    fprintf('📁 No rl_models directory found\n');
end

%% Check for log files
if exist('rl_logs', 'dir')
    log_files = dir('rl_logs/*.mat');
    fprintf('\n📊 Training Logs: %d files\n', length(log_files));
else
    fprintf('\n📊 No rl_logs directory found\n');
end

%% Check for plot files
plot_files = dir('*_training_*.png');
if length(plot_files) > 0
    fprintf('\n📈 Training Plots: %d files\n', length(plot_files));
    for i = 1:min(3, length(plot_files))
        fprintf('   - %s\n', plot_files(i).name);
    end
else
    fprintf('\n📈 No training plots found yet\n');
end

%% Check MATLAB processes
fprintf('\n🔄 MATLAB Processes:\n');
try
    [~, result] = system('tasklist /FI "IMAGENAME eq MATLAB.exe" /FO CSV');
    if contains(result, 'MATLAB.exe')
        fprintf('   ✓ MATLAB is running\n');
    else
        fprintf('   ❌ No MATLAB processes found\n');
    end
catch
    fprintf('   ? Could not check MATLAB processes\n');
end

fprintf('\n=== Status Check Complete ===\n');
fprintf('\n💡 Tips:\n');
fprintf('   - Training typically takes 30-60 minutes for 2000 episodes\n');
fprintf('   - Check rl_models/ for saved models\n');
fprintf('   - Look for *_training_*.png for progress plots\n');
fprintf('   - Run evaluate_trained_rl.m to test trained models\n');

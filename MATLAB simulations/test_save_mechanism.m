function test_save_mechanism()
%% Test if the dataset saving mechanism works

fprintf('Testing dataset save mechanism...\n');

% Create test data
dataset = struct();
dataset.time = [1; 2; 3];
dataset.est_pos = rand(3,3);
dataset.est_vel = rand(3,3);
dataset.est_att = rand(3,3);
dataset.target_pos = rand(3,3);
dataset.target_vel = rand(3,3);
dataset.start_ned = [0; 0; -10];
dataset.goal_ned = [30; 40; -12];

% Test directory creation
outdir = fullfile('outputs','data');
fprintf('Output directory: %s\n', outdir);
fprintf('Current working directory: %s\n', pwd);

if ~exist(outdir, 'dir')
    fprintf('Creating output directory...\n');
    try
        mkdir(outdir);
        fprintf('✓ Directory created successfully\n');
    catch ME
        fprintf('✗ Failed to create directory: %s\n', ME.message);
        return;
    end
else
    fprintf('✓ Output directory already exists\n');
end

% Test file saving
ts = datestr(now, 'yyyymmdd_HHMMSS');
mat_path = fullfile(outdir, sprintf('test_dataset_%s.mat', ts));
fprintf('Attempting to save to: %s\n', mat_path);

try
    save(mat_path, '-struct', 'dataset');
    fprintf('✓ MAT file saved successfully\n');
    
    % Verify file exists and check size
    if exist(mat_path, 'file')
        info = dir(mat_path);
        fprintf('✓ File verified: %s (%.1f KB)\n', mat_path, info.bytes/1024);
        
        % Test loading
        loaded = load(mat_path);
        fprintf('✓ File loads successfully with fields: %s\n', strjoin(fieldnames(loaded), ', '));
        
        % Clean up test file
        delete(mat_path);
        fprintf('✓ Test file cleaned up\n');
    else
        fprintf('✗ File was not created despite no save error\n');
    end
catch ME
    fprintf('✗ Failed to save MAT file: %s\n', ME.message);
    return;
end

fprintf('✓ Dataset save mechanism working correctly\n');
end

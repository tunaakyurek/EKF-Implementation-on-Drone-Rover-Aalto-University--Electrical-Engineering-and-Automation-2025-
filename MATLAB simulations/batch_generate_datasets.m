function batch_generate_datasets(num_runs, animate)
% batch_generate_datasets  Run repeated autonomous obstacle-avoidance sims and export EKF datasets
% Usage:
%   batch_generate_datasets(200, false)

if nargin < 1 || isempty(num_runs), num_runs = 200; end
if nargin < 2 || isempty(animate), animate = false; end

outdir = fullfile('outputs','data');
if ~exist(outdir,'dir'), mkdir(outdir); end

success_count = 0;
reject_count = 0;
fail_count = 0;

for i = 1:num_runs
    try
        fprintf('\n=== Dataset run %d/%d ===\n', i, num_runs);
        % Clear any previous run data
        evalin('base','clear LAST_DATASET_QUALITY LAST_DATASET_PATH OA_RANDOM_SEED');
        
        % Randomize seed per-run
        OA_RANDOM_SEED = uint32(randi([0, intmax('uint32')])); %#ok<NASGU>
        assignin('base','OA_RANDOM_SEED', OA_RANDOM_SEED);

        % Run generator (saves MAT/CSV and includes voxel map)
        test_animated_obstacle_avoidance(animate);
        
        % Filter out low-quality (diverging-after-min) trajectories
        if evalin('base','exist(''LAST_DATASET_QUALITY'',''var'')')
            q = evalin('base','LAST_DATASET_QUALITY');
            if ~q.pass
                baddir = fullfile(outdir,'rejected'); if ~exist(baddir,'dir'), mkdir(baddir); end
                p = evalin('base','LAST_DATASET_PATH');
                if exist(p,'file')
                    [~,name,ext] = fileparts(p);
                    movefile(p, fullfile(baddir, [name ext]));
                    % also move CSV if exists
                    csvp = fullfile(outdir, [name '.csv']);
                    if exist(csvp,'file'), movefile(csvp, fullfile(baddir, [name '.csv'])); end
                    reject_count = reject_count + 1;
                    fprintf('  Rejected run %d due to divergence (min=%.1fm, final=%.1fm, max_div=%.1fm)\n', ...
                        i, q.min_distance, q.final_distance, q.max_divergence_after_min);
                else
                    fprintf('  Run %d: quality check failed, no file to move\n', i);
                    fail_count = fail_count + 1;
                end
            else
                success_count = success_count + 1;
                fprintf('  Run %d: ACCEPTED (min=%.1fm, final=%.1fm)\n', i, q.min_distance, q.final_distance);
            end
        else
            fprintf('  Run %d: No quality data available (likely save failed)\n', i);
            fail_count = fail_count + 1;
        end
    catch ME
        fprintf('  Run %d CRASHED: %s\n', i, ME.message);
        fail_count = fail_count + 1;
    end
end

fprintf('\n=== BATCH COMPLETE ===\n');
fprintf('Success: %d datasets saved\n', success_count);
fprintf('Rejected: %d (moved to rejected/)\n', reject_count);  
fprintf('Failed: %d (crashed or save failed)\n', fail_count);
fprintf('Total: %d runs\n', success_count + reject_count + fail_count);
fprintf('Good datasets in: %s\n', outdir);
end



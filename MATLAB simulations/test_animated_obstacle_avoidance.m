%% test_animated_obstacle_avoidance.m - Animated Obstacle Avoidance with EKF
% PURPOSE
% Complete animated test system combining:
% - Built-in MATLAB toolbox flight control (RRT* + uavWaypointFollower + VFH)
% - EKF state estimation with realistic sensor simulation
% - 3D obstacle avoidance scenario with animation
% - Professional autonomous flight controller integration
%
% FEATURES
% - Real-time 3D animation of drone flight
% - Obstacle avoidance using VFH controller
% - EKF state estimation with sensor noise/failures
% - Professional flight control with PID loops
% - Performance monitoring and visualization
% - NO RL COMPONENTS - Pure MATLAB toolbox implementation

function test_animated_obstacle_avoidance(animate)
    clc; close all;
    fprintf('=== Animated Obstacle Avoidance with IMPROVED EKF (No RL) ===\n');

    if nargin < 1 || isempty(animate)
        animate = true;  % default: run with live animation
    end

    %% 0) Setup and Toolbox Checks
    addpath('.');
    addpath('rl_obstacle_avoidance');
    
    % Guard against diag shadowing (check for custom diag.m files only)
    diag_path = which('diag','-all');
    custom_diag = diag_path(~contains(diag_path, 'matlab') & ~contains(diag_path, 'toolbox'));
    if ~isempty(custom_diag)
        warning('Custom diag.m found on path:\n%s\nUsing builtin(''diag'',...) to avoid shadowing.', strjoin(custom_diag, '\n'));
    end
    clear diag   % in case a variable named 'diag' exists in workspace
    
    % Check required toolboxes
    req = {'Robotics System Toolbox', 'Navigation Toolbox', 'UAV Toolbox'};
    v = ver; have = ismember(req, {v.Name});
    for i = 1:numel(req)
        fprintf('  %-28s : %s\n', req{i}, tern(have(i),'FOUND','MISSING'));
        
    end
    if ~all(have(1:2))
        error('Robotics System Toolbox and Navigation Toolbox are required.');
    end

    %% 1) Initialize Parameters (IMPROVED)
    params = parameters_animated_obstacle_avoidance();
    
    % Simulation parameters
    T_end = 60;  % seconds
    dt = params.Ts.physics;  % keep controller, slew limits, and integration consistent
    t = 0:dt:T_end;
    N = numel(t);
    
    % Scenario parameters (define in NED; +Z down)
    start_ned = [0; 0; -10];      % was +10
    goal_ned  = [30; 40; -12];    % was [40; 30; -12] - swapped X,Y to get [40,30,12] in ENU
    bounds = [-5 50 -5 40 0 25];  % ENU workspace for planner/plot (Z up)
    
    fprintf('  Simulation duration: %.1f seconds\n', T_end);
    fprintf('  Start (NED): [%.1f %.1f %.1f]\n', start_ned);
    fprintf('  Goal  (NED): [%.1f %.1f %.1f]\n', goal_ned);

    %% 2) Build 3D Obstacle Map
    fprintf('  Building 3D obstacle map...\n');
    [map, obstacles] = build_obstacle_map(bounds);
    
    % Make obstacles and goal available to functions via base workspace
    assignin('base', 'obstacles', obstacles);
    assignin('base', 'goal_ned', goal_ned);
    
    %% 3) Plan Path with RRT*
    fprintf('  Planning path with RRT*...\n');
    % Plan in ENU using converted start/goal
    start_enu = ned2enu_vec(start_ned);
    goal_enu  = ned2enu_vec(goal_ned);
    [path_enu, planner] = plan_path_rrt(map, start_enu, goal_enu, bounds);
    fprintf('  Planned path with %d waypoints\n', size(path_enu, 1));
    
    %% 4) Smooth Trajectory
    fprintf('  Smoothing trajectory...\n');
    [traj_t, traj_pos_enu, traj_vel_enu, traj_yaw_enu] = smooth_trajectory(path_enu);
    
    %% 5) Initialize Systems
    fprintf('  Initializing systems...\n');
    
    % Initialize EKF state estimation
    ekf_state = initialize_ekf_state(start_ned);
    
    % Initialize autonomous flight controller
    flight_controller = autonomous_flight_controller(params);
    
    % Initialize guidance systems
    guidance_systems = initialize_guidance_systems(params);
    
    % Initialize true state
    true_state = [start_ned; zeros(3,1); zeros(3,1)];  % [pos; vel; att]
    
    % Initialize EKF covariance (force built-in diag to avoid shadowing)
    ekf_covariance = builtin('diag', [ ...
        1.0^2, 1.0^2, 1.0^2, ...          % pos
        0.5^2, 0.5^2, 0.5^2, ...          % vel
        deg2rad(5)^2, deg2rad(5)^2, deg2rad(10)^2 ... % att
    ]);
    assert(isequal(size(ekf_covariance),[9,9]), 'Initial ekf_covariance must be 9x9');
    
    %% 6) Initialize Animation (optional)
    if animate
        fprintf('  Setting up 3D animation...\n');
        [fig, ax, drone, path_line, true_line, est_line, hud] = setup_animation(bounds, path_enu, obstacles);
    else
        fig = []; ax = []; drone = []; path_line = []; true_line = []; est_line = []; hud = struct();
        fprintf('  Live animation disabled (animate=false)\n');
    end
    
    %% 7) Initialize Logging
    log_data = initialize_logging(N);
    
    %% 8) Main Simulation Loop
    fprintf('  Starting simulation...\n');
    
    for k = 1:N
        current_time = t(k);
        
        % Get current trajectory reference (ENU)
        % ENU references from the smoothed path
        [p_ref_enu, v_ref_enu, yaw_ref_enu] = get_trajectory_reference(current_time, traj_t, traj_pos_enu, traj_vel_enu, traj_yaw_enu);

        % Convert ENU -> NED for the controller/EKF/physics
        target_pos = enu2ned_vec(p_ref_enu);
        target_vel = enu2ned_vec(v_ref_enu);
        target_yaw = enu2ned_yaw(yaw_ref_enu);
        
        % Guidance (VFH uses ENU internally; controller uses NED)
        [vel_cmd, yaw_rate_cmd] = generate_guidance_commands(true_state, target_pos, target_vel, target_yaw, guidance_systems, obstacles);
        
        % Use VFH-blended velocity (guidance) as feed-forward for the controller
        target_state = [target_pos; vel_cmd; [0; 0; target_yaw]];
        
        % Use autonomous flight controller to compute control inputs
        [control_inputs, control_info] = flight_controller.compute_control(true_state, target_state, dt);
        
        % Generate sensor measurements from true state (use control inputs for IMU consistency)
        measurements = generate_sensor_measurements(true_state, control_inputs, params);

        % ===== Use sensor-only EKF (same as main_random.m) =====
        imu_meas = measurements.imu;  % [ax ay az gx gy gz]

        % IMU-rate prediction
        if mod(k-1, max(1, round(params.Ts.IMU/dt))) == 0
            [ekf_state, ekf_covariance] = ekf_sensor_only_oa(ekf_state, ekf_covariance, ...
                imu_meas, [], params, params.Ts.IMU, 'IMU');
        end

        % GPS update (10 Hz typical)
        if ~isempty(measurements.gps) && mod(k-1, max(1, round(params.Ts.GPS/dt))) == 0
            [ekf_state, ekf_covariance] = ekf_sensor_only_oa(ekf_state, ekf_covariance, ...
                imu_meas, measurements.gps, params, params.Ts.GPS, 'GPS');
        end

        % Barometer update
        if ~isempty(measurements.baro) && mod(k-1, max(1, round(params.Ts.Baro/dt))) == 0
            [ekf_state, ekf_covariance] = ekf_sensor_only_oa(ekf_state, ekf_covariance, ...
                imu_meas, measurements.baro, params, params.Ts.Baro, 'Baro');
        end

        % Magnetometer (yaw) update with gating to avoid high-dynamic corruption
        useMag = (norm(imu_meas(4:6)) < deg2rad(45)) && (abs(norm(imu_meas(1:3)) - abs(params.g(3))) < 1.5);
        if useMag && ~isempty(measurements.mag) && mod(k-1, max(1, round(params.Ts.Mag/dt))) == 0
            [ekf_state, ekf_covariance] = ekf_sensor_only_oa(ekf_state, ekf_covariance, ...
                imu_meas, measurements.mag, params, params.Ts.Mag, 'Mag');
        end
        
        % Update true state using control inputs
        true_state = simulate_drone_dynamics(true_state, control_inputs, dt, params);

        % Hard collision constraint
        drone_radius = 0.35;
        true_state = enforce_collision_constraints_ned(true_state, obstacles, drone_radius);
        
        % Log data (with sensor measurements and sensor errors)
        log_data = update_logging(log_data, k, true_state, ekf_state, ekf_covariance, ...
            target_pos, target_vel, current_time, measurements, dt);
        
        % Update animation (optional)
        if animate
            update_animation(fig, ax, drone, path_line, true_line, est_line, hud, obstacles, ...
                true_state, ekf_state, log_data, k);
            drawnow;
        end
        
        % Early stop when close to goal (no velocity constraint for immediate termination)
        goal_tol = 1.5;           % meters - tighter tolerance to get closer to goal
        dist_to_goal = norm(true_state(1:3) - goal_ned);
        if dist_to_goal < goal_tol
            fprintf('  Reached goal (distance: %.2fm < %.1fm). Ending simulation at t=%.1fs\n', ...
                dist_to_goal, goal_tol, current_time);
            break;
        end
        
        % Progress indicator and goal distance
        if mod(k, round(N/20)) == 0
            fprintf('  Progress: %.0f%% (t=%.1fs, dist to goal: %.2fm)\n', 100*k/N, current_time, dist_to_goal);
        end
    end

    %% Truncate logs to the actual simulation length (cut at goal reach)
    final_idx = find(log_data.time > 0, 1, 'last');
    if isempty(final_idx), final_idx = k; end
    fields = {'true_pos','true_vel','true_att','est_pos','est_vel','est_att','uncertainty', ...
              'target_pos','target_vel','estimation_error','time','meas_gps','true_gps','meas_baro','true_baro','meas_mag','true_mag'};
    for ii = 1:numel(fields)
        f = fields{ii};
        if isfield(log_data, f)
            A = log_data.(f);
            if ~isempty(A)
                if size(A,2) >= final_idx
                    log_data.(f) = A(:,1:final_idx);
                elseif isvector(A) && numel(A) >= final_idx
                    log_data.(f) = A(1:final_idx);
                end
            end
        end
    end
    t = t(1:final_idx);
    
    %% 9) Final Analysis and Plots
    fprintf('  Simulation complete. Generating analysis...\n');
    try
        generate_analysis_plots(log_data, t, params);
        fprintf('  Analysis plots generated successfully.\n');
    catch ME
        fprintf('  Warning: Analysis plots generation failed: %s\n', ME.message);
    end

    %% 10) Save EKF-only dataset for RL (also export voxelized map)
    try
        outdir = fullfile('outputs','data');
        if ~exist(outdir, 'dir'), mkdir(outdir); end
        ts = datestr(now, 'yyyymmdd_HHMMSS');
        if exist('k','var') && ~isempty(k)
            last_idx = k;
        else
            last_idx = numel(t);
        end
        last_idx = max(1, min(last_idx, size(log_data.est_pos, 2)));

        dataset.time        = reshape(log_data.time(1:last_idx), [], 1);
        dataset.est_pos     = log_data.est_pos(:, 1:last_idx);
        dataset.est_vel     = log_data.est_vel(:, 1:last_idx);
        dataset.est_att     = log_data.est_att(:, 1:last_idx);
        dataset.target_pos  = log_data.target_pos(:, 1:last_idx);
        dataset.target_vel  = log_data.target_vel(:, 1:last_idx);
        % Add scenario endpoints
        dataset.start_ned   = start_ned;
        dataset.goal_ned    = goal_ned;

        % Quality metric: ensure no large divergence after closest approach
        dist_to_goal = vecnorm(dataset.est_pos - goal_ned, 2, 1);
        [min_d, idx_min] = min(dist_to_goal);
        final_d = dist_to_goal(end);
        % Define pass if final distance is within min_d + tol, and no big sustained divergence
        tol_close = 3.0;            % meters
        pass_final = final_d <= (min_d + tol_close);
        % Check max increase after min index
        post = dist_to_goal(idx_min:end);
        max_diverge = max(0, max(post - min_d));
        pass_diverge = max_diverge <= 5.0;  % allow brief bumps, reject big detours
        quality = struct('min_distance',min_d,'final_distance',final_d, ...
                         'max_divergence_after_min',max_diverge, ...
                         'idx_min',idx_min,'pass',logical(pass_final && pass_diverge));

        % Serialize occupancy grid to a compact struct (indices of occupied voxels)
        try
            % For occupancyMap3D, use checkOccupancy to sample the grid
            [X,Y,Z] = meshgrid(-5:0.5:50, -5:0.5:40, 0:0.5:25);
            pts = [X(:), Y(:), Z(:)];
            occ_vals = checkOccupancy(map, pts);
            occ_idx = find(occ_vals > 0.5);
            map_data = struct();
            map_data.resolution = map.Resolution;
            map_data.occupied_points = pts(occ_idx, :);
            map_data.bounds = [-5 50 -5 40 0 25];
        catch ME
            fprintf('  Warning: Map serialization failed: %s\n', ME.message);
            map_data = struct('resolution', map.Resolution, 'occupied_points', [], 'bounds', []);
        end

        % Add map_data and quality to dataset struct
        dataset.map_data = map_data;
        dataset.quality = quality;

        mat_path = fullfile(outdir, sprintf('ekf_obstacle_avoidance_%s.mat', ts));
        save(mat_path, '-struct', 'dataset');
        assignin('base','LAST_DATASET_PATH', mat_path);
        assignin('base','LAST_DATASET_QUALITY', quality);

        % Also export a compact CSV (columns: t, ex, ey, ez, evx, evy, evz, er, ep, eyaw)
        ex = dataset.est_pos(1,:).'; ey = dataset.est_pos(2,:).'; ez = dataset.est_pos(3,:).';
        evx = dataset.est_vel(1,:).'; evy = dataset.est_vel(2,:).'; evz = dataset.est_vel(3,:).';
        er = dataset.est_att(1,:).';  ep = dataset.est_att(2,:).';  eyaw = dataset.est_att(3,:).';
        Tcsv = table(dataset.time, ex, ey, ez, evx, evy, evz, er, ep, eyaw, ...
            'VariableNames', {'t','ex','ey','ez','evx','evy','evz','roll','pitch','yaw'});
        csv_path = fullfile(outdir, sprintf('ekf_obstacle_avoidance_%s.csv', ts));
        try
            writetable(Tcsv, csv_path);
        catch
            % Fallback for environments without writetable
            fid = fopen(csv_path, 'w');
            fprintf(fid, 't,ex,ey,ez,evx,evy,evz,roll,pitch,yaw\n');
            fclose(fid);
            dlmwrite(csv_path, Tcsv.Variables, '-append');
        end
        fprintf('  EKF-only dataset saved: %s (and CSV)\n', mat_path);
    catch ME
        fprintf('  Warning: Failed to save EKF-only dataset: %s\n', ME.message);
    end
    
    fprintf('=== Test Complete ===\n');
end

function [map, obstacles] = build_obstacle_map(bounds)
    % Occupancy map
    map = occupancyMap3D(0.5);

    % Workspace free
    fx = bounds(1):1.0:bounds(2);
    fy = bounds(3):1.0:bounds(4);
    fz = bounds(5):1.0:bounds(6);
    [Xf, Yf, Zf] = ndgrid(fx, fy, fz);
    setOccupancy(map, [Xf(:) Yf(:) Zf(:)], 0);

    % Randomized obstacle list if OA_RANDOM_SEED provided, else canonical
    try
        if evalin('base','exist(''OA_RANDOM_SEED'',''var'')');
            seed = evalin('base','OA_RANDOM_SEED');
            rng(double(seed));
            numObs = randi([4,8]);
            obstacles = repmat(struct('center',[0,0],'radius',2.5,'height',18,'baseZ',0), numObs, 1);
            for i = 1:numObs
                cx = bounds(1) + (bounds(2)-bounds(1)) * rand();
                cy = bounds(3) + (bounds(4)-bounds(3)) * rand();
                obstacles(i).center = [cx, cy];
                obstacles(i).radius = 1.5 + 2.0*rand();     % 1.5..3.5 m
                obstacles(i).height = 12 + 14*rand();       % 12..26 m
                obstacles(i).baseZ  = 0;                    % ground
            end
        else
            % One canonical obstacle list (ENU frame, Z up)
            obstacles = [
                struct('center',[15,10], 'radius',3.0, 'height',20, 'baseZ',0);
                struct('center',[25,20], 'radius',2.5, 'height',18, 'baseZ',0);
                struct('center',[35, 8], 'radius',2.0, 'height',15, 'baseZ',0);
                struct('center',[10,25], 'radius',2.8, 'height',22, 'baseZ',0);
                struct('center',[30,25], 'radius',3.2, 'height',16, 'baseZ',0)
            ];
        end
    catch
        % Fallback to canonical
        obstacles = [
            struct('center',[15,10], 'radius',3.0, 'height',20, 'baseZ',0);
            struct('center',[25,20], 'radius',2.5, 'height',18, 'baseZ',0);
            struct('center',[35, 8], 'radius',2.0, 'height',15, 'baseZ',0);
            struct('center',[10,25], 'radius',2.8, 'height',22, 'baseZ',0);
            struct('center',[30,25], 'radius',3.2, 'height',16, 'baseZ',0)
        ];
    end

    % Rasterize cylinders into the 3D map
    th = linspace(0,2*pi,48);
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        for z = 0:0.5:obs.height
            X = obs.center(1) + obs.radius*cos(th);
            Y = obs.center(2) + obs.radius*sin(th);
            Z = obs.baseZ + z*ones(size(th));
            setOccupancy(map, [X.' Y.' Z.'], 1);
        end
    end

    % Inflate more than before: radius + safety
    inflate(map, 1.0);  % meters
end

function [path, planner] = plan_path_rrt(map, start_pos, goal_pos, bounds)
    %% Plan Path Using RRT* Planner
    
    % Setup state space (SE(3))
    ss = stateSpaceSE3;
    ss.StateBounds = [bounds(1:2); bounds(3:4); bounds(5:6); -1 1; -1 1; -1 1; -1 1];
    
    % Setup validator
    sv = validatorOccupancyMap3D(ss);
    sv.Map = map;
    sv.ValidationDistance = 0.5;
    
    % Setup planner
    planner = plannerRRTStar(ss, sv, 'MaxConnectionDistance', 4.0, 'GoalBias', 0.2, 'MaxIterations', 5000);
    
    % Ensure start/goal are valid
    start_state = [start_pos.' 1 0 0 0];
    goal_state = [goal_pos.' 1 0 0 0];
    
    if ~isStateValid(sv, start_state)
        start_state = nudge_to_free_space(sv, start_state, bounds);
    end
    if ~isStateValid(sv, goal_state)
        goal_state = nudge_to_free_space(sv, goal_state, bounds);
    end
    
    % Plan path
    [pathObj, ~] = plan(planner, start_state, goal_state);
    path = pathObj.States(:, 1:3);  % Extract position components
    
    % Shortcut path
    path = shortcut_path(path, map);
end

function state = nudge_to_free_space(sv, state, bounds)
    %% Nudge state to free space if invalid
    for z = state(3):0.5:bounds(6)
        candidate = [state(1) state(2) z 1 0 0 0];
        if isStateValid(sv, candidate)
            state = candidate;
            return;
        end
    end
end

function path = shortcut_path(path, map)
    %% Shortcut path to remove unnecessary waypoints
    if size(path, 1) < 3
        return;
    end
    
    out = path(1, :);
    i = 1;
    n = size(path, 1);
    
    while i < n
        j = n;
        while j > i + 1
            if check_line_free(map, path(i, :), path(j, :))
                out(end+1, :) = path(j, :);
                i = j;
                break;
            end
            j = j - 1;
        end
        if j == i + 1
            out(end+1, :) = path(j, :);
            i = j;
        end
    end
    
    path = out;
end

function free = check_line_free(map, p1, p2)
    %% Check if line between two points is collision-free
    n = max(2, ceil(norm(p2 - p1) / map.Resolution));
    s = linspace(0, 1, n).';
    seg = p1 .* (1 - s) + p2 .* s;
    occ = getOccupancy(map, seg);
    free = all(occ < 0.5);
end

function [trajectory_time, trajectory_pos, trajectory_vel, trajectory_yaw] = smooth_trajectory(path)
    %% Smooth Path into Time-Parameterized Trajectory
    
    % Calculate path length
    s = [0; cumsum(vecnorm(diff(path), 2, 2))];
    total_length = s(end);
    
    % Time parameters
    v_nom = 3.0;  % m/s
    tf = max(2.0, total_length / v_nom);
    dt = 0.1;
    trajectory_time = 0:dt:tf;
    
    % Generate smooth trajectory
    trajectory_pos = cubicpolytraj(path.', s/s(end)*tf, trajectory_time);
    trajectory_vel = gradient(trajectory_pos, dt);
    
    % Calculate yaw from velocity direction
    trajectory_yaw = atan2(trajectory_vel(2, :), trajectory_vel(1, :));
    trajectory_yaw = unwrap(trajectory_yaw);
end

function guidance_systems = initialize_guidance_systems(params)
    %% Initialize Guidance Systems
    
    guidance_systems = struct();
    
    % Waypoint follower
    try
        guidance_systems.waypoint_follower = uavWaypointFollower(...
            'UAVType', 'multirotor', ...
            'TransitionRadius', 3.0, ...
            'MinLookaheadDistance', 2.0);
    catch
        guidance_systems.waypoint_follower = [];
    end
    
    % VFH controller for obstacle avoidance (less conservative to prevent detours)
    guidance_systems.vfh = controllerVFH(...
        'DistanceLimits', [0.5, 6.0], ...
        'RobotRadius', 0.35, ...
        'SafetyDistance', 0.5, ...
        'NumAngularSectors', 72);
    
    % Control gains
    guidance_systems.Kp_pos = [1.5; 1.5; 2.0];
    guidance_systems.Kd_vel = [0.8; 0.8; 1.2];
end

function [target_pos, target_vel, target_yaw] = get_trajectory_reference(current_time, trajectory_time, trajectory_pos, trajectory_vel, trajectory_yaw)
    %% Get Current Trajectory Reference
    
    % Interpolate position
    target_pos = interp1(trajectory_time, trajectory_pos.', current_time, 'linear', 'extrap').';
    
    % Interpolate velocity
    target_vel = interp1(trajectory_time, trajectory_vel.', current_time, 'linear', 'extrap').';
    
    % Interpolate yaw
    target_yaw = interp1(trajectory_time, trajectory_yaw, current_time, 'linear', 'extrap');
end

function [vel_cmd, yaw_rate_cmd] = generate_guidance_commands(true_state, target_pos, target_vel, target_yaw, guidance_systems, obstacles)
    % true_state, target_* are NED
    current_pos = true_state(1:3);
    current_vel = true_state(4:6);
    current_att = true_state(7:9);   % [roll; pitch; yaw] NED

    % --- Outer-loop PD in NED ---
    pos_error = target_pos - current_pos;
    vel_error = target_vel - current_vel;
    vel_cmd = guidance_systems.Kp_pos .* pos_error + guidance_systems.Kd_vel .* vel_error;

    % --- VFH avoidance in ENU (scan) with limited deflection ---
    pos_enu = ned2enu_vec(current_pos);
    yaw_enu = ned2enu_yaw(current_att(3));        % proper yaw mapping
    [ranges, angles] = generate_laser_scan(pos_enu, yaw_enu, obstacles);

    desired_heading_ned = atan2( vel_cmd(2), vel_cmd(1) );
    desired_heading_enu = ned2enu_yaw(desired_heading_ned); % map NED heading to ENU azimuth
    steer_dir = guidance_systems.vfh(ranges, angles, desired_heading_enu);

    if ~isnan(steer_dir)
        % Blend minimally toward avoidance to keep direct path preference
        alpha = 0.3;
        blended = alpha*steer_dir + (1-alpha)*desired_heading_enu;
        % Limit deflection from desired heading to avoid turn-arounds
        max_deflect = deg2rad(30);
        dpsi = atan2(sin(blended - desired_heading_enu), cos(blended - desired_heading_enu));
        dpsi = max(min(dpsi, max_deflect), -max_deflect);
        final_heading_enu = desired_heading_enu + dpsi;
        % speed in horizontal plane (use NED mag, reuse as ENU)
        speed_xy = norm(vel_cmd(1:2));
        vxy_enu  = speed_xy * [cos(final_heading_enu); sin(final_heading_enu)];
        % convert ENU vxy -> NED vxy and keep vertical from existing vel_cmd
        vxy_ned  = enu2ned_vec([vxy_enu; 0]);
        vel_cmd(1:2) = vxy_ned(1:2);
    end

    % --- Soft repulsion (NED) to smooth away from edges (reduced weight) ---
    drone_radius = 0.35;
    vel_cmd = vel_cmd + 0.3 * obstacle_repulsion_velocity(current_pos, obstacles, drone_radius);

    % --- Limit velocity and set yaw-rate ---
    max_vel = 5.0;
    vel_cmd = max(min(vel_cmd, max_vel), -max_vel);

    yaw_error = atan2(sin(target_yaw - current_att(3)), cos(target_yaw - current_att(3)));
    yaw_rate_cmd = 2.0 * yaw_error;
    yaw_rate_cmd = max(min(yaw_rate_cmd, deg2rad(45)), -deg2rad(45));
end

function [ranges, angles] = generate_laser_scan(pos_enu, yaw_enu, obstacles)
    angles = linspace(-pi, pi, 72);
    ranges = zeros(size(angles));
    max_range = 12.0;

    for i = 1:numel(angles)
        dir2 = [cos(angles(i)+yaw_enu); sin(angles(i)+yaw_enu)];
        r = max_range;
        for j = 1:numel(obstacles)
            obs = obstacles(j);
            % Intersect 2D ray with cylinder cap (ignore z thickness here)
            % We only consider if drone z is within obstacle vertical span:
            if pos_enu(3) >= obs.baseZ-0.5 && pos_enu(3) <= obs.baseZ+obs.height+0.5
                % Solve ||(pos + t*dir) - center|| = R
                p0 = pos_enu(1:2) - obs.center(:);
                a = 1; b = 2*(dir2.'*p0); c = (p0.'*p0) - (obs.radius+0.5)^2; % +0.5 safety
                disc = b*b - 4*a*c;
                if disc >= 0
                    t1 = (-b - sqrt(disc)) / (2*a);
                    t2 = (-b + sqrt(disc)) / (2*a);
                    t_hit = min([t1 t2 t1(t1>0) t2(t2>0)], [], 'omitnan');
                    if ~isempty(t_hit) && t_hit > 0
                        r = min(r, t_hit);
                    end
                end
            end
        end
        ranges(i) = max(0.2, min(r, max_range));
    end
end

function [fig, ax, drone, path_line, true_line, est_line, hud] = setup_animation(bounds, path_enu, obstacles)
    fig = figure('Position', [100, 100, 1400, 800]);

    % Main 3D plot
    ax = subplot(2,3,[1,2,4,5]);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    xlim(ax,[bounds(1), bounds(2)]); ylim(ax,[bounds(3), bounds(4)]); zlim(ax,[bounds(5), bounds(6)]);
    xlabel(ax,'X_E (m)'); ylabel(ax,'Y_N (m)'); zlabel(ax,'Z_U (m)'); title(ax,'Animated Obstacle Avoidance with EKF');
    view(ax,45,30);

    % Planned + trails
    path_line = plot3(ax, path_enu(:,1), path_enu(:,2), path_enu(:,3), 'k--','LineWidth',2);
    true_line = plot3(ax, NaN, NaN, NaN, 'g-','LineWidth',1.5);  % GREEN = True Path
    est_line  = plot3(ax, NaN, NaN, NaN, 'b-','LineWidth',1.5);  % BLUE  = EKF Estimated Path

    % Obstacles
    plot_obstacles(ax, obstacles);

    % Drone model (hgtransform based)
    drone = create_drone(ax);
    set_drone_pose(drone, [0;0;0], [0;0;0]);  % ENU pose

    legend([path_line, true_line, est_line, drone.body], ...
           {'Planned Path','True Path','EKF Estimated Path','Drone'}, 'Location','best');

    % ===== HUD =====
    hud.ax  = subplot(2,3,3); cla(hud.ax);
    axis(hud.ax,'off');
    hud.txt = text(hud.ax, 0, 1, '', 'Units','normalized', ...
                   'HorizontalAlignment','left', 'VerticalAlignment','top', ...
                   'FontName','Consolas', 'FontSize', 11);

    % Status panel (optional)
    hud.ax2  = subplot(2,3,6); cla(hud.ax2); axis(hud.ax2,'off');
    hud.txt2 = text(hud.ax2, 0, 1, '', 'Units','normalized', ...
                    'HorizontalAlignment','left', 'VerticalAlignment','top', ...
                    'FontName','Consolas', 'FontSize', 11);
end


function drone = create_drone(ax)
    % Create a drone model (sphere body + 4 arms) under a single hgtransform
    t = hgtransform('Parent', ax);

    % Body
    body_radius = 0.1;
    [Xb,Yb,Zb] = sphere(12);
    body = surf('Parent', t, Xb*body_radius, Yb*body_radius, Zb*body_radius, ...
                'FaceColor',[0.2 0.2 0.8], 'EdgeColor','none');

    % Arms (draw along ±X and ±Y in body frame, z=0)
    arm_length = 0.3;
    arms = gobjects(4,1);
    arms(1) = plot3(t, [0 +arm_length], [0 0], [0 0], 'k-', 'LineWidth', 3);
    arms(2) = plot3(t, [0 -arm_length], [0 0], [0 0], 'k-', 'LineWidth', 3);
    arms(3) = plot3(t, [0 0], [0 +arm_length], [0 0], 'k-', 'LineWidth', 3);
    arms(4) = plot3(t, [0 0], [0 -arm_length], [0 0], 'k-', 'LineWidth', 3);

    drone = struct('hT', t, 'body', body, 'arms', arms);
end

function set_drone_pose(drone, pos, att)
    % Update the hgtransform with ZYX (yaw-pitch-roll) then translate
    yaw = att(3); pitch = att(2); roll = att(1);
    T = makehgtform('zrotate', yaw, 'yrotate', pitch, 'xrotate', roll, ...
                    'translate', pos(:)');
    set(drone.hT, 'Matrix', T);
end



function plot_obstacles(ax, obstacles)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        [X,Y,Z] = cylinder(obs.radius, 40);
        Z = Z*obs.height + obs.baseZ;
        X = X + obs.center(1);  Y = Y + obs.center(2);
        surf(ax, X, Y, Z, 'FaceColor',[0.8,0.2,0.2], 'FaceAlpha',0.6, 'EdgeColor','none');
        fill3(ax, X(1,:),Y(1,:),Z(1,:), [0.8,0.2,0.2], 'FaceAlpha',0.6);
        fill3(ax, X(2,:),Y(2,:),Z(2,:), [0.8,0.2,0.2], 'FaceAlpha',0.6);
    end
end

function log_data = initialize_logging(N)
    %% Initialize Data Logging
    
    log_data = struct();
    log_data.true_pos = zeros(3, N);
    log_data.true_vel = zeros(3, N);
    log_data.true_att = zeros(3, N);
    log_data.est_pos = zeros(3, N);
    log_data.est_vel = zeros(3, N);
    log_data.est_att = zeros(3, N);
    log_data.uncertainty = zeros(9, N);
    log_data.target_pos = zeros(3, N);
    log_data.target_vel = zeros(3, N);
    log_data.time = zeros(1, N);
    log_data.estimation_error = zeros(9, N);
    % Sensor logs
    log_data.meas_gps = NaN(3, N);
    log_data.meas_baro = NaN(1, N);
    log_data.meas_mag = NaN(1, N);
    % Derived true measurements (what ideal sensors would report)
    log_data.true_gps = NaN(3, N);
    log_data.true_baro = NaN(1, N);
    log_data.true_mag = NaN(1, N);
end

function log_data = update_logging(log_data, k, true_state, ekf_state, ekf_covariance, target_pos, target_vel, current_time, measurements, dt)
    %% Update Logging Data
    
    log_data.true_pos(:, k) = true_state(1:3);
    log_data.true_vel(:, k) = true_state(4:6);
    log_data.true_att(:, k) = true_state(7:9);
    
    log_data.est_pos(:, k) = ekf_state(1:3);
    log_data.est_vel(:, k) = ekf_state(4:6);
    log_data.est_att(:, k) = ekf_state(7:9);
    
    log_data.uncertainty(:, k) = sqrt(builtin('diag', ekf_covariance));
    log_data.target_pos(:, k) = target_pos;
    log_data.target_vel(:, k) = target_vel;
    log_data.time(k) = current_time;
    
    % Calculate estimation error
    error = true_state - ekf_state;
    error(7:9) = atan2(sin(error(7:9)), cos(error(7:9)));  % Wrap angle errors
    log_data.estimation_error(:, k) = error;

    % Log sensor measurements
    if isfield(measurements,'gps') && ~isempty(measurements.gps)
        log_data.meas_gps(:,k) = measurements.gps;
        log_data.true_gps(:,k) = true_state(1:3);
    end
    if isfield(measurements,'baro') && ~isempty(measurements.baro)
        log_data.meas_baro(1,k) = measurements.baro;
        log_data.true_baro(1,k) = -true_state(3); % altitude = -z (NED)
    end
    if isfield(measurements,'mag') && ~isempty(measurements.mag)
        log_data.meas_mag(1,k) = measurements.mag;
        log_data.true_mag(1,k) = true_state(9); % yaw
    end
end

function update_animation(fig, ax, drone, path_line, true_line, est_line, hud, obstacles, true_state, ekf_state, log_data, k)
    if ~ishandle(ax) || ~isvalid(ax), return; end

    % Convert NED -> ENU for drawing
    pos_true_enu = ned2enu_vec(true_state(1:3));
    vel_true_enu = ned2enu_vec(true_state(4:6));
    att_true_enu = ned2enu_eul(true_state(7:9));     % [roll; pitch; yaw] in ENU

    pos_est_enu  = ned2enu_vec(ekf_state(1:3));

    % Pose
    set_drone_pose(drone, pos_true_enu, att_true_enu);

    % Trails (True=green, EKF=blue)
    if k > 1
        true_line.XData(end+1) = pos_true_enu(1);  true_line.YData(end+1) = pos_true_enu(2);  true_line.ZData(end+1) = pos_true_enu(3);
        est_line.XData(end+1)  = pos_est_enu(1);   est_line.YData(end+1)  = pos_est_enu(2);   est_line.ZData(end+1)  = pos_est_enu(3);
    end

    % Calculate distance to goal
    goal_ned = evalin('base', 'goal_ned');
    dist_to_goal = norm(true_state(1:3) - goal_ned);
    
    % HUD (show ENU components to match axes; angles in deg)
    rpy_deg = rad2deg(att_true_enu(:)');
    hud.txt.String = sprintf([ ...
        'Position (ENU) [m]\n' ...
        '  X_E = %7.2f   Y_N = %7.2f   Z_U = %7.2f\n' ...
        '\nVelocity (ENU) [m/s]\n' ...
        '  VxE = %7.2f   VyN = %7.2f   VzU = %7.2f\n' ...
        '\nEuler Angles (Body Frame) [deg]\n' ...
        '  Roll  = %7.2f   Pitch = %7.2f   Yaw   = %7.2f\n' ...
        '\nDistance to Goal: %.2f m' ], ...
        pos_true_enu(1), pos_true_enu(2), pos_true_enu(3), ...
        vel_true_enu(1), vel_true_enu(2), vel_true_enu(3), ...
        rpy_deg(1),      rpy_deg(2),      rpy_deg(3), ...
        dist_to_goal);

    % Status line (pos error etc., optional)
    pos_err = norm(true_state(1:3) - ekf_state(1:3));
    hud.txt2.String = sprintf('Time: %.2f s   |EKF Error|: %.2f m   |Speed|: %.2f m/s', ...
        log_data.time(k), pos_err, norm(vel_true_enu));

    % Title
    title(ax, sprintf('Animated Obstacle Avoidance (t=%.1fs)', log_data.time(k)));
end



function generate_analysis_plots(log_data, t, params)
    %% Generate Analysis Plots
    
    figure('Position', [200, 200, 1400, 1000]);
    
    % Trim to last valid time to avoid flatlining beyond early termination
    last_idx = find(log_data.time > 0, 1, 'last');
    if ~isempty(last_idx)
        t = log_data.time(1:last_idx);
        fields = {'true_pos','true_vel','true_att','est_pos','est_vel','est_att','uncertainty', ...
                  'target_pos','target_vel','estimation_error','meas_gps','true_gps','meas_baro','true_baro','meas_mag','true_mag'};
        for ii = 1:numel(fields)
            f = fields{ii};
            if isfield(log_data, f)
                A = log_data.(f);
                if ~isempty(A)
                    if size(A,2) >= last_idx
                        log_data.(f) = A(:,1:last_idx);
                    end
                end
            end
        end
    end

    % Position tracking
    subplot(3, 3, 1);
    plot(t, log_data.true_pos(1,:), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(t, log_data.true_pos(2,:), 'g-', 'LineWidth', 1.5);
    plot(t, log_data.true_pos(3,:), 'r-', 'LineWidth', 1.5);
    plot(t, log_data.est_pos(1,:), 'b--', 'LineWidth', 1);
    plot(t, log_data.est_pos(2,:), 'g--', 'LineWidth', 1);
    plot(t, log_data.est_pos(3,:), 'r--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Position (m)');
    title('Position Tracking');
    legend('True X', 'True Y', 'True Z', 'Est X', 'Est Y', 'Est Z', 'Location', 'best');
    grid on;
    
    % Velocity tracking
    subplot(3, 3, 2);
    plot(t, log_data.true_vel(1,:), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(t, log_data.true_vel(2,:), 'g-', 'LineWidth', 1.5);
    plot(t, log_data.true_vel(3,:), 'r-', 'LineWidth', 1.5);
    plot(t, log_data.est_vel(1,:), 'b--', 'LineWidth', 1);
    plot(t, log_data.est_vel(2,:), 'g--', 'LineWidth', 1);
    plot(t, log_data.est_vel(3,:), 'r--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    title('Velocity Tracking');
    legend('True Vx', 'True Vy', 'True Vz', 'Est Vx', 'Est Vy', 'Est Vz', 'Location', 'best');
    grid on;
    
    % Attitude tracking
    subplot(3, 3, 3);
    plot(t, rad2deg(log_data.true_att(1,:)), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(t, rad2deg(log_data.true_att(2,:)), 'g-', 'LineWidth', 1.5);
    plot(t, rad2deg(log_data.true_att(3,:)), 'r-', 'LineWidth', 1.5);
    plot(t, rad2deg(log_data.est_att(1,:)), 'b--', 'LineWidth', 1);
    plot(t, rad2deg(log_data.est_att(2,:)), 'g--', 'LineWidth', 1);
    plot(t, rad2deg(log_data.est_att(3,:)), 'r--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Attitude (deg)');
    title('Attitude Tracking');
    legend('True Roll', 'True Pitch', 'True Yaw', 'Est Roll', 'Est Pitch', 'Est Yaw', 'Location', 'best');
    grid on;
    
    % Estimation errors
    subplot(3, 3, 4);
    pos_error = vecnorm(log_data.estimation_error(1:3, :), 2, 1);
    plot(t, pos_error, 'r-', 'LineWidth', 1.5); hold on;
    % Overlay sensor position errors if available
    if any(~isnan(log_data.meas_gps(:)))
        gps_err = NaN(size(t));
        idx = find(~isnan(log_data.meas_gps(1,:)));
        if ~isempty(idx)
            gps_pos = log_data.meas_gps(:,idx);
            true_pos = log_data.true_gps(:,idx);
            gps_err(idx) = vecnorm(gps_pos - true_pos, 2, 1);
        end
        plot(t, gps_err, 'b--', 'LineWidth', 1);
        legend('EKF Pos Error','GPS Pos Error','Location','best');
    else
        legend('EKF Pos Error','Location','best');
    end
    xlabel('Time (s)'); ylabel('Position Error (m)');
    title('Position Estimation Error');
    grid on;
    
    subplot(3, 3, 5);
    vel_error = vecnorm(log_data.estimation_error(4:6, :), 2, 1);
    plot(t, vel_error, 'g-', 'LineWidth', 1.5); hold on;
    % There is no direct velocity sensor; leave placeholder for future IMU-derived proxy
    legend('EKF Vel Error','Location','best');
    xlabel('Time (s)'); ylabel('Velocity Error (m/s)');
    title('Velocity Estimation Error');
    grid on;
    
    subplot(3, 3, 6);
    att_error = rad2deg(vecnorm(log_data.estimation_error(7:9, :), 2, 1));
    plot(t, att_error, 'm-', 'LineWidth', 1.5); hold on;
    % Overlay yaw sensor error (mag) if available
    if any(~isnan(log_data.meas_mag(:)))
        mag_err = NaN(size(t));
        idx = find(~isnan(log_data.meas_mag(1,:)));
        if ~isempty(idx)
            mag_meas = log_data.meas_mag(1,idx);
            yaw_true = log_data.true_mag(1,idx);
            d = arrayfun(@(a,b) atan2(sin(a-b), cos(a-b)), mag_meas, yaw_true);
            mag_err(idx) = abs(rad2deg(d));
        end
        plot(t, mag_err, 'c--', 'LineWidth', 1);
        legend('EKF Att Error (norm)','Mag Yaw Error','Location','best');
    else
        legend('EKF Att Error (norm)','Location','best');
    end
    xlabel('Time (s)'); ylabel('Attitude Error (deg)');
    title('Attitude Estimation Error');
    grid on;
    
    % Uncertainty
    subplot(3, 3, 7);
    plot(t, log_data.uncertainty(1:3, :), 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Position Uncertainty (m)');
    title('Position Uncertainty');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;
    
    subplot(3, 3, 8);
    plot(t, log_data.uncertainty(4:6, :), 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Velocity Uncertainty (m/s)');
    title('Velocity Uncertainty');
    legend('Vx', 'Vy', 'Vz', 'Location', 'best');
    grid on;
    
    subplot(3, 3, 9);
    plot(t, rad2deg(log_data.uncertainty(7:9, :)), 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Attitude Uncertainty (deg)');
    title('Attitude Uncertainty');
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    grid on;
    
    sgtitle('EKF State Estimation Performance Analysis', 'FontSize', 16);
    
    % Save plot
    saveas(gcf, 'animated_obstacle_avoidance_analysis.png');
    fprintf('  Analysis plot saved: animated_obstacle_avoidance_analysis.png\n');
end

% REMOVED: initialize_parameters() function - now using parameters_animated_obstacle_avoidance()

function ekf_state = initialize_ekf_state(start_pos)
    %% Initialize EKF State
    
    % Initial state estimate [pos; vel; att]
    ekf_state = [start_pos; zeros(3,1); zeros(3,1)];
end

function measurements = generate_sensor_measurements(true_state, control_inputs, params)
    %% Generate Realistic Sensor Measurements
    
    measurements = struct();
    
    % IMU measurements (always available)
    measurements.imu = generate_imu_measurements(true_state, control_inputs, params);
    
    % GPS measurements (with some noise)
    measurements.gps = generate_gps_measurements(true_state, params);
    
    % Barometer measurements
    measurements.baro = generate_baro_measurements(true_state, params);
    
    % Magnetometer measurements
    measurements.mag = generate_mag_measurements(true_state, params);
end

function imu_meas = generate_imu_measurements(true_state, control_inputs, params)
    %% Generate IMU Measurements
    
    % Specific force based on applied control and drag, consistent with dynamics
    % World frame quantities (NED, +Z down)
    gravity_w = [0; 0; params.g(3)];
    att = true_state(7:9);
    vel = true_state(4:6);
    thrust = control_inputs(1);
    R_wb = eul2rotm([att(3), att(2), att(1)], 'ZYX');
    thrust_world = R_wb * [0; 0; -thrust];
    rho = 1.225;
    CdA = params.drag_coeff;
    vmag = max(norm(vel), 1e-6);
    drag_world = -0.5 * rho * CdA * vmag * vel;
    % a_world = gravity + thrust/mass + drag/mass
    a_world = gravity_w + thrust_world/params.mass + drag_world/params.mass;
    % specific force f_b = R_wb' * (a_world - gravity)
    f_b = R_wb' * (a_world - gravity_w);

    % Body rates from torque/inertia dynamics (consistent with simulate_drone_dynamics)
    persistent omega_b
    if isempty(omega_b), omega_b = zeros(3,1); end
    if isvector(params.I), I = builtin('diag', params.I); else, I = params.I; end
    tau = control_inputs(2:4);
    dtp = params.Ts.physics;
    omega_dot = I \ (tau(:) - cross(omega_b, I*omega_b));
    omega_b = omega_b + omega_dot * dtp;

    % Add noise
    accel_noise = params.IMU.accel_noise_density * randn(3,1);
    gyro_noise  = params.IMU.gyro_noise_density  * randn(3,1);

    imu_meas = [f_b + accel_noise; omega_b + gyro_noise];
end

function gps_meas = generate_gps_measurements(true_state, params)
    %% Generate GPS Measurements
    
    gps_noise = [
        params.GPS.sigma_xy * randn();
        params.GPS.sigma_xy * randn();
        params.GPS.sigma_z * randn()
    ];
    
    gps_meas = true_state(1:3) + gps_noise;
end

function baro_meas = generate_baro_measurements(true_state, params)
    %% Generate Barometer Measurements
    
    baro_noise = params.Baro.sigma_z * randn();
    baro_meas = -true_state(3) + baro_noise;  % Altitude = -z in NED
end

function mag_meas = generate_mag_measurements(true_state, params)
    %% Generate Magnetometer Measurements
    
    mag_noise = params.Mag.sigma_rad * randn();
    mag_meas = true_state(9) + mag_noise;  % Yaw angle
end

function [ekf_state, ekf_covariance] = update_ekf_improved(ekf_state, measurements, control_inputs, params, dt, ekf_covariance)
    %% Update EKF with Measurements (IMPROVED VERSION)
    
    % Add dimension assertions to catch any state/covariance shape issues
    assert(isequal(size(ekf_state),[9,1]), 'ekf_state must be 9x1');
    assert(isequal(size(ekf_covariance),[9,9]), 'ekf_covariance must be 9x9');
    
    % Initialize covariance if not provided (force built-in diag to avoid shadowing)
    if nargin < 6 || isempty(ekf_covariance)
        ekf_covariance = builtin('diag', [ ...
            1.0^2, 1.0^2, 1.0^2, ...
            0.5^2, 0.5^2, 0.5^2, ...
            deg2rad(5)^2, deg2rad(5)^2, deg2rad(10)^2 ]);
    end
    
    % Prediction step (using IMPROVED EKF)
    try
        [ekf_state, ekf_covariance] = ekf_animated_obstacle_avoidance_improved(...
            ekf_state, ekf_covariance, measurements.imu, [], params, dt, 'IMU');
    catch ME
        fprintf('EKF IMU step error: %s\n', ME.message);
        fprintf('ekf_state size: %s\n', mat2str(size(ekf_state)));
        fprintf('ekf_covariance size: %s\n', mat2str(size(ekf_covariance)));
        fprintf('IMU size: %s\n', mat2str(size(measurements.imu)));
        rethrow(ME);
    end
    
    % Update steps for each available sensor (using IMPROVED EKF)
    if ~isempty(measurements.gps)
        [ekf_state, ekf_covariance] = ekf_animated_obstacle_avoidance_improved(...
            ekf_state, ekf_covariance, measurements.imu, measurements.gps, params, 0, 'GPS');
    end
    
    if ~isempty(measurements.baro)
        [ekf_state, ekf_covariance] = ekf_animated_obstacle_avoidance_improved(...
            ekf_state, ekf_covariance, measurements.imu, measurements.baro, params, 0, 'Baro');
    end
    
    if ~isempty(measurements.mag)
        [ekf_state, ekf_covariance] = ekf_animated_obstacle_avoidance_improved(...
            ekf_state, ekf_covariance, measurements.imu, measurements.mag, params, 0, 'Mag');
    end
    
    % Update persistent variable
    ekf_covariance_persistent = ekf_covariance;
end

function true_state = simulate_drone_dynamics(current_state, control_inputs, dt, params)
    %% Simulate True Drone Dynamics
    
    % Extract control inputs
    thrust = control_inputs(1);
    torque = control_inputs(2:4);
    
    % Extract current state
    pos = current_state(1:3);
    vel = current_state(4:6);
    att = current_state(7:9);
    
    % Position integration
    new_pos = pos + vel * dt;
    
    % Gravity (NED, +Z down)
    gravity = [0; 0; params.g(3)];
    
    % Rotate body thrust to world frame
    R_wb = eul2rotm([att(3), att(2), att(1)], 'ZYX');
    thrust_world = R_wb * [0; 0; -thrust];
    
    % Total acceleration with drag
    accel = gravity + thrust_world / params.mass;
    rho = 1.225;
    CdA = params.drag_coeff;
    vmag = max(norm(vel), 1e-6);
    drag_world = -0.5 * rho * CdA * vmag * vel;
    accel = accel + drag_world / params.mass;
    
    new_vel = vel + accel * dt;
    
    % Attitude integration using body rates and quaternion
    persistent omega_b q
    if isempty(omega_b); omega_b = zeros(3,1); end
    if isempty(q); q = eul2quat(att(:).', 'ZYX'); end
    
    if isvector(params.I)
        I = builtin('diag', params.I);
    else
        I = params.I;
    end
    omega_dot = I \ (torque(:) - cross(omega_b, I*omega_b));
    omega_b = omega_b + omega_dot * dt;
    
    % Quaternion kinematics
    skew_omega = [0 -omega_b(3) omega_b(2); omega_b(3) 0 -omega_b(1); -omega_b(2) omega_b(1) 0];
    Omega = [ 0,          -omega_b';
              omega_b,    -skew_omega ];
    q = (q.' + 0.5 * Omega * q.' * dt).';
    q = q / norm(q);
    
    eulZyx = quat2eul(q, 'ZYX');
    new_att = [eulZyx(3); eulZyx(2); eulZyx(1)];
    new_att(3) = wrapToPi(new_att(3));
    
    true_state = [new_pos; new_vel; new_att];
end

function s = tern(cond, a, b)
    %% Ternary operator
    if cond, s = a; else, s = b; end
end

% === NED <-> ENU helpers ===
function p_enu = ned2enu_vec(p_ned), p_enu = [p_ned(2); p_ned(1); -p_ned(3)]; end
function p_ned = enu2ned_vec(p_enu), p_ned = [p_enu(2); p_enu(1); -p_enu(3)]; end

function yaw_enu = ned2enu_yaw(yaw_ned), yaw_enu = wrapToPi(pi/2 - yaw_ned); end
function yaw_ned = enu2ned_yaw(yaw_enu), yaw_ned = wrapToPi(pi/2 - yaw_enu); end

function att_enu = ned2enu_eul(att_ned)
    % Convert full attitude via rotation matrices: R_enu = T * R_ned * T'
    Rn = eul2rotm([att_ned(3), att_ned(2), att_ned(1)], 'ZYX'); % [yaw pitch roll]
    T  = [0 1 0; 1 0 0; 0 0 -1];
    Re = T * Rn * T';
    ez = rotm2eul(Re, 'ZYX');   % -> [yaw pitch roll]
    att_enu = [ez(3); ez(2); ez(1)];  % [roll; pitch; yaw]
end

function v_avoid = obstacle_repulsion_velocity(pos_ned, obstacles, drone_radius)
    pos_enu = ned2enu_vec(pos_ned);
    vxy = [0;0];
    d_safe = drone_radius + 0.8;   % start pushing within this buffer
    k = 1.5;                       % strength (m/s at contact)

    for j = 1:numel(obstacles)
        obs = obstacles(j);
        if pos_enu(3) >= obs.baseZ-0.5 && pos_enu(3) <= obs.baseZ+obs.height+0.5
            dvec = pos_enu(1:2) - obs.center(:);
            r = norm(dvec) + 1e-9;
            pen = (obs.radius + d_safe) - r;  % positive inside buffer
            if pen > 0
                n = dvec / r;
                vxy = vxy + k * (pen / d_safe) * n;  % smooth ramp (0..k)
            end
        end
    end
    v_avoid = enu2ned_vec([vxy; 0]);
end

function state_out = enforce_collision_constraints_ned(state_in, obstacles, drone_radius)
    p_enu = ned2enu_vec(state_in(1:3));
    v_enu = ned2enu_vec(state_in(4:6));   % sign flip on z ok

    bumped = false;
    for j = 1:numel(obstacles)
        obs = obstacles(j);
        % Only if within vertical span:
        if p_enu(3) >= obs.baseZ && p_enu(3) <= obs.baseZ + obs.height
            dvec = p_enu(1:2) - obs.center(:);
            r = norm(dvec);
            R = obs.radius + drone_radius;
            if r < R
                bumped = true;
                % Project to surface + tiny epsilon
                if r < 1e-6, n = [1;0]; else, n = dvec / r; end
                p_enu(1:2) = obs.center(:) + n * (R + 1e-3);
                % Zero inward radial velocity
                v_rad = dot(v_enu(1:2), n);
                if v_rad < 0, v_enu(1:2) = v_enu(1:2) - v_rad*n; end
            end
        end
    end

    state_out = state_in;
    if bumped
        state_out(1:3) = enu2ned_vec(p_enu);
        state_out(4:6) = enu2ned_vec(v_enu);
    end
end

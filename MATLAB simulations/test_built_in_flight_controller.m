%% test_built_in_flight_controller.m - Built-in Toolbox Flight Control Demo (R2023b)
% PURPOSE
% End-to-end path planning and trajectory following using MATLAB toolboxes only.
% - Checks installed toolboxes
% - Builds a simple 3D occupancy map (columns)
% - Plans with RRT* (Navigation/Robotics)
% - Smooths with cubicpolytraj (Robotics System Toolbox)
% - Follows trajectory with uavTrajectoryFollower (UAV Toolbox) if available
% - Simulates simple point-mass dynamics with thrust rotation for validation plots

function test_built_in_flight_controller()
    clc; close all;
    fprintf('=== Built-in Flight Controller Demo (R2023b) ===\n');

    %% 0) Toolbox capability checks
    req = { ...
        'Robotics System Toolbox', 'Navigation Toolbox', 'UAV Toolbox' ...
    };
    v = ver; have = ismember(req, {v.Name});
    for i = 1:numel(req)
        fprintf('  %-28s : %s\n', req{i}, tern(have(i),'FOUND','MISSING'));
    end
    if ~all(have(1:2))
        error('Robotics System Toolbox and Navigation Toolbox are required.');
    end
    haveUAV = have(3);

    % Object/function existence
    haveFollower = haveUAV && exist('uavTrajectoryFollower','class')==8; % System object

    %% 1) Parameters
    params.mass = 0.5; params.I = diag([0.0023 0.0023 0.004]);
    params.g = [0;0;9.81]; params.drag_coeff = 0.05; params.Ts = 0.01;
    T_end = 40; t = 0:params.Ts:T_end; N = numel(t);

    startXYZ = [0 0 10]; goalXYZ = [35 25 12];
    bounds = [-10 50 -10 40 0 20];

    %% 2) Build simple 3D occupancy map (columns)
    try
        map = occupancyMap3D(0.5);
        % Mark the workspace volume as free (unknown treated as occupied by some validators)
        fx = bounds(1):1.0:bounds(2); fy = bounds(3):1.0:bounds(4); fz = bounds(5):1.0:bounds(6);
        [Xf,Yf,Zf] = ndgrid(fx,fy,fz);
        setOccupancy(map, [Xf(:) Yf(:) Zf(:)], 0);

        % Add cylindrical column obstacles
        cols = [struct('center',[10 10],'radius',2.5,'height',20,'baseZ',0), ...
                struct('center',[20 18],'radius',2.0,'height',20,'baseZ',0), ...
                struct('center',[28 8],'radius',2.0,'height',20,'baseZ',0)];
        for k = 1:numel(cols)
            c = cols(k);
            th = linspace(0,2*pi,48); R = c.radius;
            for z = 0:0.5:c.height
                X = c.center(1) + R*cos(th); Y = c.center(2) + R*sin(th); Z = c.baseZ + z*ones(size(th));
                setOccupancy(map, [X.' Y.' Z.'], 1);
            end
        end
        inflate(map, 0.4);
    catch
        error('occupancyMap3D not available in this installation.');
    end

    %% 3) Plan with RRT* (SE(3))
    ss = stateSpaceSE3;
    % StateBounds is 7x2: [x;y;z; qw; qx; qy; qz]
    ss.StateBounds = [bounds(1:2); bounds(3:4); bounds(5:6); -1 1; -1 1; -1 1; -1 1];
    sv = validatorOccupancyMap3D(ss);
    sv.Map = map; sv.ValidationDistance = 0.5;
    planner = plannerRRTStar(ss, sv, 'MaxConnectionDistance', 3.0, 'GoalBias', 0.15, 'MaxIterations', 4000);
    % Ensure start/goal states are valid; if not, nudge Z upward within bounds
    startSE3 = [startXYZ 1 0 0 0]; goalSE3 = [goalXYZ 1 0 0 0];
    if ~isStateValid(sv, startSE3)
        for z = startXYZ(3):0.5:bounds(6)
            cand = [startXYZ(1) startXYZ(2) z 1 0 0 0];
            if isStateValid(sv, cand), startSE3 = cand; break; end
        end
    end
    if ~isStateValid(sv, goalSE3)
        for z = goalXYZ(3):0.5:bounds(6)
            cand = [goalXYZ(1) goalXYZ(2) z 1 0 0 0];
            if isStateValid(sv, cand), goalSE3 = cand; break; end
        end
    end

    [pthObj, ~] = plan(planner, startSE3, goalSE3);
    raw = pthObj.States(:,1:3);
    path = shortcut3D(map, raw);
    fprintf('  Planned path with %d waypoints (post-shortcut).\n', size(path,1));

    %% 4) Smooth trajectory (cubicpolytraj) and velocity feed-forward
    s = [0; cumsum(vecnorm(diff(path),2,2))]; total = s(end);
    v_nom = 2.0; tf = max(1.0, total / v_nom);
    tvec = linspace(0, tf, max(25, ceil(total)));
    Q = cubicpolytraj(path.', s/s(end)*tf, tvec); % 3xM
    dt = tvec(2)-tvec(1);
    Qd = gradient(Q, dt);

    %% 5) Follower selection
    if haveFollower
        follower = uavTrajectoryFollower('SampleTime', params.Ts, 'MaxRollAngle', deg2rad(35), 'MaxPitchAngle', deg2rad(35));
        follower.TrajectoryTimes = tvec;
        follower.TrajectoryPositions = Q.';
        follower.TrajectoryVelocities = Qd.';
        useFollower = true;
        fprintf('  Using uavTrajectoryFollower for tracking.\n');
    else
        useFollower = false;
        fprintf('  uavTrajectoryFollower not found. Using uavWaypointFollower + VFH for guidance.\n');
        % Guidance layer: uavWaypointFollower for course/speed + controllerVFH for reactive heading
        try
            wpf = uavWaypointFollower('UAVType','multirotor', 'TransitionRadius', 4.0, 'MinLookaheadDistance', 2.0);
        catch
            wpf = []; % Not fatal; we will do pure pursuit manually on the smoothed path
        end
        vfh = controllerVFH('DistanceLimits',[0.2 6.0], 'RobotRadius',0.3, 'SafetyDistance',0.4, 'NumAngularSectors',72);
    end

    %% 6) Simulation state and logs
    x = [startXYZ 0 0 0 0 0 0].';  % [pos(3); vel(3); att(3)]
    log_pos = zeros(3,N); log_ref = zeros(3,N); log_u = zeros(4,N);

    for k = 1:N
        tk = t(k);
        % Interpolate desired ref from trajectory
        idx = min(numel(tvec), max(1, round(tk/dt)+1));
        p_ref = Q(:,idx); v_ref = Qd(:,idx);
        log_ref(:,k) = p_ref;

        if useFollower
            % Get attitude and thrust commands from follower
            [attCmd, thrustCmd] = follower(x(1:3).', x(4:6).', x(7:9).', tk);
            u = [thrustCmd; attCmd(:)];
        else
            % Guidance from waypoint follower (course/speed) and VFH obstacle avoidance
            pos = x(1:3); att = x(7:9);
            % Build a small local polar histogram from occupancy map samples along bearing (simple proxy)
            % Use desired path direction from smoothed path
            dir_vec = (p_ref - pos); dir_vec(3) = 0; dir_norm = norm(dir_vec(1:2));
            if dir_norm > 1e-6
                desiredHeading = atan2(dir_vec(2), dir_vec(1));
            else
                desiredHeading = att(3);
            end
            % Fake ranges: none available here, so bias VFH toward desiredHeading only
            scanAngles = linspace(-pi, pi, 72);
            ranges = 10*ones(size(scanAngles));
            steerDir = vfh(ranges, scanAngles, desiredHeading);
            if isnan(steerDir)
                steerDir = desiredHeading;
            end
            speedCmd = 2.0; v_ref_xy = speedCmd * [cos(steerDir); sin(steerDir)];
            v_ref_full = [v_ref_xy; 0];
            % Vertical reference from trajectory
            v_ref_full(3) = v_ref(3);
            % PD in position/velocity -> accel
            Kp = [1.2;1.2;2.0]; Kd = [0.6;0.6;1.2];
            pos_err = p_ref - pos; vel_err = v_ref_full - x(4:6);
            a_cmd = Kp.*pos_err + Kd.*vel_err;
            % Map to attitude + thrust
            [attCmd, thrustCmd] = accelToAttThrust(a_cmd, att, params);
            u = [thrustCmd; attCmd(:)];
        end

        % Simulate simple dynamics with thrust rotation
        x = stepPointMass(x, u, params);
        log_pos(:,k) = x(1:3);
        log_u(:,k) = u;
    end

    %% 7) Plots
    figure('Position',[100 100 1200 500]);
    subplot(1,2,1); plot3(path(:,1), path(:,2), path(:,3),'k.-'); hold on;
    plot3(log_pos(1,:), log_pos(2,:), log_pos(3,:), 'b'); grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Path vs Actual'); legend('Planned','Actual');
    subplot(1,2,2); plot(t, log_pos.' - interp1(tvec, Q.', min(t, tvec(end)))); grid on;
    xlabel('Time (s)'); ylabel('Position Error (m)'); legend('ex','ey','ez'); title('Tracking Error');

    fprintf('Done.\n');
end

function path = shortcut3D(map, pts)
    if size(pts,1) < 3, path = pts; return; end
    out = pts(1,:); i = 1; n = size(pts,1);
    while i < n
        j = n;
        while j > i+1
            if lineFree(map, pts(i,:), pts(j,:))
                out(end+1,:) = pts(j,:); i = j; break; %#ok<AGROW>
            end
            j = j - 1;
        end
        if j == i+1
            out(end+1,:) = pts(j,:); i = j; %#ok<AGROW>
        end
    end
    path = out;
end

function ok = lineFree(map, p, q)
    n = max(2, ceil(norm(q-p)/map.Resolution));
    s = linspace(0,1,n).'; seg = p.*(1-s) + q.*s;
    occ = getOccupancy(map, seg);
    ok = all(occ < 0.5);
end

function [att_cmd, thrust_cmd] = accelToAttThrust(a_cmd, current_att, params)
    g = params.g; m = params.mass;
    FT_w = m * (a_cmd - g);
    FTn = max(norm(FT_w), 1e-6);
    thrust_cmd = FTn;
    z_b_des = -FT_w / FTn;
    yaw = current_att(3);
    x_c = [cos(yaw); sin(yaw); 0];
    y_b_des = cross(z_b_des, x_c); y_b_des = y_b_des / max(norm(y_b_des),1e-6);
    x_b_des = cross(y_b_des, z_b_des);
    R_des = [x_b_des y_b_des z_b_des];
    eulZyx = rotm2eul(R_des,'ZYX');
    att_cmd = [eulZyx(3); eulZyx(2); eulZyx(1)];
end

function x = stepPointMass(x, u, params)
    % x: [pos; vel; att], u=[thrust; att_des(3)]
    dt = params.Ts; g = params.g; m = params.mass; CdA = params.drag_coeff;
    pos = x(1:3); vel = x(4:6); att = x(7:9);
    att_des = u(2:4);

    % First-order lag on thrust and attitude
    if ~isfield(params,'tau_thrust'), params.tau_thrust = 0.08; end
    if ~isfield(params,'tau_att'), params.tau_att = 0.12; end
    persistent thrust_prev
    if isempty(thrust_prev), thrust_prev = u(1); end
    thrust = thrust_prev + (u(1) - thrust_prev) * dt/params.tau_thrust;
    thrust_prev = thrust;

    % Attitude lag with wrap on yaw
    yaw_err = atan2(sin(att_des(3)-att(3)), cos(att_des(3)-att(3)));
    att_err = [att_des(1)-att(1); att_des(2)-att(2); yaw_err];
    att = att + att_err * dt/params.tau_att;

    R_wb = eul2rotm([att(3) att(2) att(1)], 'ZYX');
    thrust_world = R_wb * [0;0;-thrust];
    vmag = max(norm(vel),1e-6); drag = -0.5*1.225*CdA*vmag*vel;
    acc = g + thrust_world/m + drag/m;
    vel = vel + acc*dt; pos = pos + vel*dt;
    x = [pos; vel; att];
end

function s = tern(cond, a, b)
    if cond, s=a; else, s=b; end
end



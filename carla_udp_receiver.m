function carla_udp_receiver(port)
% CARLA_UDP_RECEIVER - Main entry point for the CARLA diagnostics dashboard.
% This script sets up a modern UI, receives data from a Python CARLA client
% over UDP, processes it, and continuously updates the dashboard components.
% It uses a robust, state-aware fuzzy logic controller to manage driving modes.

    % --- Configuration ---
    if nargin < 1
        port = 10000;
    end
    HISTORY_LENGTH = 100; % Number of historical data points for plots
    FDIR_RATE_HZ = 5;     % How often the FDIR system checks for faults

    % --- Global Outputs for External Access ---
    global carla_outputs;
    carla_outputs = struct(); 

    is_running = true;
    fdirTimer = []; % Initialize timer handle

    % --- Setup UI and UDP ---
    [uiHandles] = setupDashboard(HISTORY_LENGTH, @onCloseRequest);
    logToDashboard(uiHandles, 'Dashboard initialized. Starting UDP receiver...');
    
    try
        u = udpport("IPV4", "LocalPort", port, "Timeout", 1, "EnablePortSharing", true);
        u.InputBufferSize = 2000000;
        logToDashboard(uiHandles, sprintf('UDP Receiver started on port %d.', port));
    catch ME
        logToDashboard(uiHandles, sprintf('[FATAL] UDP setup failed: %s', ME.message));
        errordlg(sprintf('UDP setup on port %d failed. Is it in use?', port), 'UDP Error');
        is_running = false; 
    end

    % --- SETUP FDIR SYSTEM (TIMER) ---
    try
        fdirTimer = timer(...
            'ExecutionMode', 'fixedRate', ...
            'Period', 1/FDIR_RATE_HZ, ...
            'TimerFcn', @(~,~) run_fdir_cycle(), ...
            'StartDelay', 2, ...
            'Tag', 'FDIR_Timer', ...
            'StopFcn', @(~,~) disp('FDIR Timer has been stopped.'));
        start(fdirTimer);
        logToDashboard(uiHandles, ...
            sprintf('FDIR background monitoring system started at %d Hz.', FDIR_RATE_HZ));
    catch ME
        logToDashboard(uiHandles, sprintf('[FATAL] FDIR Timer setup failed: %s', ME.message));
        is_running = false;
    end
    
    % --- Data Storage and State Initialization ---
    trajectory = nan(HISTORY_LENGTH * 5, 2);
    gnss_track = nan(HISTORY_LENGTH * 5, 2);
    imu_history = struct('x', nan(1, HISTORY_LENGTH), 'y', nan(1, HISTORY_LENGTH), 'z', nan(1, HISTORY_LENGTH));
    
    buffer = '';
    last_message_time = tic;
    
    % --- Main Loop ---
    logToDashboard(uiHandles, 'Waiting for data from CARLA...');
    while is_running
        if ~ishandle(uiHandles.fig) 
            break;
        end
        
        latest_frame_to_render = [];
        
        if u.NumBytesAvailable > 0
            byteData = read(u, u.NumBytesAvailable, "uint8");
            newData = native2unicode(byteData, 'UTF-8');
            buffer = [buffer, newData];
            last_message_time = tic;
            
            delimiter = '|||JSON_DELIMITER|||';
            sanitizedBuffer = strrep(buffer, '}{', ['}' delimiter '{']);
            jsonObjects = strsplit(sanitizedBuffer, delimiter);
            
            if ~endsWith(jsonObjects{end}, '}')
                buffer = jsonObjects{end};
                jsonObjects = jsonObjects(1:end-1);
            else
                buffer = '';
            end
            
            for i = 1:numel(jsonObjects)
                jsonStr = jsonObjects{i};
                if isempty(jsonStr), continue; end
                
                try
                    packet = jsondecode(jsonStr);
                    processed_frame = processPacket(packet); 
                    if ~isempty(processed_frame)
                        latest_frame_to_render = processed_frame;
                    end
                catch ME
                    logToDashboard(uiHandles, sprintf('[WARN] JSON decode failed: %s', ME.message));
                end
            end
        end
        
        if ~isempty(latest_frame_to_render)
            frame_data = latest_frame_to_render;
            [trajectory, gnss_track, imu_history] = updateDataHistories(frame_data, ...
                trajectory, gnss_track, imu_history);
            processAndAnalyzeFrame(frame_data, toc(last_message_time), uiHandles);
            updateDashboard(uiHandles, frame_data, trajectory, ...
                gnss_track, imu_history, carla_outputs);
        end
        
        pause(0.01); 
    end
    
    logToDashboard(uiHandles, 'Shutdown sequence initiated...');
    
    % --- CLEANUP ---
    if ~isempty(fdirTimer) && isvalid(fdirTimer)
        stop(fdirTimer);
        delete(fdirTimer);
        disp('FDIR Timer cleaned up.');
    end
    if exist('u','var') && isvalid(u), delete(u); end
    if ishandle(uiHandles.fig), delete(uiHandles.fig); end
    disp('CARLA UDP Receiver has shut down gracefully.');

    function onCloseRequest(~, ~)
        is_running = false;
    end
end


%% =======================================================================
%                      UI SETUP
% ========================================================================
function [uiHandles] = setupDashboard(historyLength, closeCallback)
    fig = uifigure('Name', 'CARLA Final Diagnostics Dashboard', 'Position', [50 50, 1600, 950]);
    fig.CloseRequestFcn = closeCallback;
    uiHandles.fig = fig;
    
    gl = uigridlayout(fig, [3, 4]);
    gl.RowHeight = {250, '2x', 200};
    gl.ColumnWidth = {'1x', '1x', '1x', '1x'};
    
    % Speed Gauge
    p_speed = uipanel(gl, 'Title', 'Speed (km/h)', 'FontWeight', 'bold');
    p_speed.Layout.Row = 1; p_speed.Layout.Column = 1;
    speed_grid = uigridlayout(p_speed, [1 1]);
    uiHandles.speedGauge = uigauge(speed_grid, ...
        'ScaleColors', {[0 .8 .4], [.9 .8 0], [1 .2 .2]}, ...
        'ScaleColorLimits', [0 60; 60 100; 100 160]);
    
    % IMU Plot
    p_imu = uipanel(gl, 'Title', 'IMU - Accelerometer', 'FontWeight', 'bold');
    p_imu.Layout.Row = 1; p_imu.Layout.Column = 2;
    uiHandles.imuAx = uiaxes(p_imu);
    hold(uiHandles.imuAx, 'on'); grid(uiHandles.imuAx, 'on');
    uiHandles.accelXPlot = plot(uiHandles.imuAx, NaN(1, historyLength), 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
    uiHandles.accelYPlot = plot(uiHandles.imuAx, NaN(1, historyLength), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
    uiHandles.accelZPlot = plot(uiHandles.imuAx, NaN(1, historyLength), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
    xlabel(uiHandles.imuAx, 'Frames Ago'); ylabel(uiHandles.imuAx, 'm/s^2'); legend(uiHandles.imuAx);
    
    % GNSS Plot
    p_gnss = uipanel(gl, 'Title', 'GNSS Trajectory', 'FontWeight', 'bold');
    p_gnss.Layout.Row = 1; p_gnss.Layout.Column = 3;
    uiHandles.gnssAx = uiaxes(p_gnss);
    hold(uiHandles.gnssAx, 'on'); grid(uiHandles.gnssAx, 'on'); axis(uiHandles.gnssAx, 'equal');
    xlabel(uiHandles.gnssAx, 'Longitude'); ylabel(uiHandles.gnssAx, 'Latitude');
    uiHandles.gnssPlot = plot(uiHandles.gnssAx, NaN, NaN, '-m', 'LineWidth', 1.5);
    
    % Diagnostics Panel
    p_diag = uipanel(gl, 'Title', 'Diagnostics', 'FontWeight', 'bold');
    p_diag.Layout.Row = 1; p_diag.Layout.Column = 4;
    g_diag = uigridlayout(p_diag, [1 1]);
    p_diag_metrics = uipanel(g_diag, 'Title', '');
    g_diag_metrics = uigridlayout(p_diag_metrics, [10, 2], 'ColumnWidth', {'fit', '1x'});
    uilabel(g_diag_metrics, 'Text', 'Frame:'); uiHandles.frameLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold');
    uilabel(g_diag_metrics, 'Text', 'Net Latency:'); uiHandles.latencyLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold');
    uilabel(g_diag_metrics, 'Text', 'Mode:'); uiHandles.modeLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold');
    uilabel(g_diag_metrics, 'Text', 'Map:'); uiHandles.mapLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold');
    uilabel(g_diag_metrics, 'Text', 'Weather:'); uiHandles.weatherLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold');
    uilabel(g_diag_metrics, 'Text', 'Sensor Health:'); uiHandles.healthLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold');
    uilabel(g_diag_metrics, 'Text', 'Fallback Status:'); uiHandles.fallbackLabel = uilabel(g_diag_metrics, 'Text', 'IDLE', 'FontWeight', 'bold', 'FontColor', 'g');
    uilabel(g_diag_metrics, 'Text', 'Traffic Light:'); uiHandles.trafficLightLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold');
    uilabel(g_diag_metrics, 'Text', 'Sys. Confidence:'); uiHandles.confidenceLabel = uilabel(g_diag_metrics, 'Text', 'NOMINAL', 'FontWeight', 'bold');
    uilabel(g_diag_metrics, 'Text', 'Fuzzy Decision:', 'FontWeight', 'bold'); uiHandles.fuzzyDecisionLabel = uilabel(g_diag_metrics, 'Text', 'STARTING...', 'FontWeight', 'bold');
    
    % Camera Feeds
    p_cameras = uipanel(gl, 'Title', 'Camera Feeds', 'FontWeight', 'bold');
    p_cameras.Layout.Row = 2; p_cameras.Layout.Column = [1 2];
    g_cameras = uigridlayout(p_cameras, [2 3]);
    uiHandles.camLeftAx = uiaxes(g_cameras, 'XTick', [], 'YTick', []); title(uiHandles.camLeftAx, 'Left');
    uiHandles.camFrontAx = uiaxes(g_cameras, 'XTick', [], 'YTick', []); title(uiHandles.camFrontAx, 'Front');
    uiHandles.camRightAx = uiaxes(g_cameras, 'XTick', [], 'YTick', []); title(uiHandles.camRightAx, 'Right');
    uiHandles.camInteriorAx = uiaxes(g_cameras, 'XTick', [], 'YTick', []); title(uiHandles.camInteriorAx, 'Interior');
    uiHandles.camRearAx = uiaxes(g_cameras, 'XTick', [], 'YTick', []); title(uiHandles.camRearAx, 'Back');
    
    % Vehicle Trajectory
    p_traj = uipanel(gl, 'Title', 'Vehicle Trajectory (Fused)', 'FontWeight', 'bold');
    p_traj.Layout.Row = 2; p_traj.Layout.Column = 3;
    uiHandles.trajAx = uiaxes(p_traj);
    hold(uiHandles.trajAx, 'on'); grid(uiHandles.trajAx, 'on'); axis(uiHandles.trajAx, 'equal');
    xlabel(uiHandles.trajAx, 'X (m)'); ylabel(uiHandles.trajAx, 'Y (m)');
    uiHandles.trajPlot = plot(uiHandles.trajAx, NaN, NaN, '-c', 'LineWidth', 2, 'DisplayName', 'Ego Path'); 
    uiHandles.currentPosPlot = plot(uiHandles.trajAx, NaN, NaN, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Ego Vehicle'); 
    uiHandles.v2vPlot = plot(uiHandles.trajAx, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5, 'DisplayName', 'Other Vehicles');
    legend(uiHandles.trajAx);
    
    % LiDAR Plot
    p_lidar = uipanel(gl, 'Title', '3D LiDAR & Object Detections', 'FontWeight', 'bold');
    p_lidar.Layout.Row = 2; p_lidar.Layout.Column = 4;
    uiHandles.lidarAx = uiaxes(p_lidar); 
    hold(uiHandles.lidarAx, 'on'); grid(uiHandles.lidarAx, 'on'); axis(uiHandles.lidarAx, 'equal');
    xlabel(uiHandles.lidarAx, 'X (m)'); ylabel(uiHandles.lidarAx, 'Y (m)'); zlabel(uiHandles.lidarAx, 'Z (m)');
    view(uiHandles.lidarAx, -45, 30);
    uiHandles.lidarPlot = scatter3(uiHandles.lidarAx, NaN, NaN, NaN, 10, NaN, 'filled', 'DisplayName', 'LiDAR Points');
    uiHandles.obstaclePlot = scatter3(uiHandles.lidarAx, NaN, NaN, NaN, 100, 'bo', 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Obstacles');
    legend(uiHandles.lidarAx);
    
    % System Log
    p_log = uipanel(gl, 'Title', 'System Log', 'FontWeight', 'bold');
    p_log.Layout.Row = 3; p_log.Layout.Column = [1 4];
    log_grid = uigridlayout(p_log, [1 1]);
    uiHandles.logArea = uitextarea(log_grid, 'Value', {''}, 'Editable', 'off', ...
        'FontName', 'Monospaced', 'BackgroundColor', [0.1 0.1 0.1], 'FontColor', [0.9 0.9 0.9]);
end


%% =======================================================================
%                      DATA PROCESSING AND UPDATING
% ========================================================================
function updateDashboard(uiHandles, data, trajectory, gnss_track, imu_history, outputs)
    if ~isvalid(uiHandles.fig), return; end
    
    % Update gauges and plots
    if isfield(data, 'speed'), uiHandles.speedGauge.Value = data.speed; end
    set(uiHandles.accelXPlot, 'YData', imu_history.x);
    set(uiHandles.accelYPlot, 'YData', imu_history.y);
    set(uiHandles.accelZPlot, 'YData', imu_history.z);
    set(uiHandles.gnssPlot, 'XData', gnss_track(:,2), 'YData', gnss_track(:,1));
    set(uiHandles.trajPlot, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
    set(uiHandles.currentPosPlot, 'XData', trajectory(end,1), 'YData', trajectory(end,2));
    
    % Update text labels
    if isfield(data, 'frame'), uiHandles.frameLabel.Text = num2str(data.frame); end
    if isfield(data, 'mode'), uiHandles.modeLabel.Text = data.mode; end
    if isfield(data, 'map'), uiHandles.mapLabel.Text = data.map; end
    if isfield(data, 'weather'), uiHandles.weatherLabel.Text = data.weather; end
    
    % Update diagnostic labels from `carla_outputs`
    if isfield(outputs, 'network_status')
        uiHandles.latencyLabel.Text = sprintf('%.2f ms (%.1f FPS)', ...
            outputs.network_status.latency * 1000, outputs.network_status.data_rate);
    end
    if isfield(outputs, 'sensor_fusion_status') && isfield(outputs.sensor_fusion_status, 'health_score')
        uiHandles.healthLabel.Text = sprintf('%.0f %% OK', outputs.sensor_fusion_status.health_score * 100);
    end
    if isfield(outputs, 'fallback_initiation')
        if outputs.fallback_initiation
            uiHandles.fallbackLabel.Text = 'TRIGGERED';
            uiHandles.fallbackLabel.FontColor = 'r';
        else
            uiHandles.fallbackLabel.Text = 'IDLE';
            uiHandles.fallbackLabel.FontColor = 'g';
        end
    end
    if isfield(outputs, 'system_confidence')
        conf = outputs.system_confidence;
        uiHandles.confidenceLabel.Text = conf.status_text;
        if conf.score > 0.8, uiHandles.confidenceLabel.FontColor = [0.2 1 0.2]; % Green
        elseif conf.score > 0.5, uiHandles.confidenceLabel.FontColor = [1 0.9 0]; % Yellow
        else, uiHandles.confidenceLabel.FontColor = [1 0.2 0.2]; % Red
        end
    end
    
    % Update camera feeds
    updateCamera(uiHandles.camFrontAx, data, 'image_front');
    updateCamera(uiHandles.camRearAx, data, 'image_back');
    updateCamera(uiHandles.camLeftAx, data, 'image_left');
    updateCamera(uiHandles.camRightAx, data, 'image_right');
    updateCamera(uiHandles.camInteriorAx, data, 'image_interior');
    
    % Update LiDAR point cloud
    if isfield(data, 'lidar_roof') && ~isempty(data.lidar_roof)
        try
            points = typecast(base64decode(data.lidar_roof), 'single');
            if mod(numel(points), 4) == 0
                points = reshape(points, 4, [])';
                xyz = points(1:20:end, 1:3); % Subsample for performance
                set(uiHandles.lidarPlot, 'XData', xyz(:,1), 'YData', xyz(:,2), ...
                    'ZData', xyz(:,3), 'CData', xyz(:,3));
            end
        catch
            % Silently fail if LiDAR data is corrupt
        end
    end

    % Update obstacle plot
    obstacles = get_safe(data, 'Detected_Obstacles', []);
    if ~isempty(obstacles)
        num_obstacles = numel(obstacles);
        obs_x = nan(num_obstacles, 1);
        obs_y = nan(num_obstacles, 1);
        obs_z = nan(num_obstacles, 1);
        for i = 1:num_obstacles
            obs_x(i) = obstacles(i).position.x;
            obs_y(i) = obstacles(i).position.y;
            obs_z(i) = obstacles(i).position.z;
        end
        set(uiHandles.obstaclePlot, 'XData', obs_x, 'YData', obs_y, 'ZData', obs_z);
    else
        set(uiHandles.obstaclePlot, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    end

    % Update V2V plot
    v2v_data = get_safe(data, 'V2V_Data', []);
    if ~isempty(v2v_data)
        num_vehicles = numel(v2v_data);
        v2v_x = nan(num_vehicles, 1); v2v_y = nan(num_vehicles, 1);
        for i = 1:num_vehicles
            v2v_x(i) = v2v_data(i).position.x; v2v_y(i) = v2v_data(i).position.y;
        end
        set(uiHandles.v2vPlot, 'XData', v2v_x, 'YData', v2v_y);
    else
        set(uiHandles.v2vPlot, 'XData', NaN, 'YData', NaN);
    end

    % Update traffic light status
    v2i_data = get_safe(data, 'V2I_Data', struct());
    if isfield(v2i_data, 'traffic_light_state') && ~isempty(v2i_data.traffic_light_state)
        state = v2i_data.traffic_light_state;
        uiHandles.trafficLightLabel.Text = state;
        switch upper(state)
            case 'RED', uiHandles.trafficLightLabel.FontColor = [1, 0.2, 0.2];
            case 'YELLOW', uiHandles.trafficLightLabel.FontColor = [1, 0.9, 0];
            case 'GREEN', uiHandles.trafficLightLabel.FontColor = [0.2, 1, 0.2];
            otherwise, uiHandles.trafficLightLabel.FontColor = [0.8, 0.8, 0.8];
        end
    else
        uiHandles.trafficLightLabel.Text = 'N/A';
        uiHandles.trafficLightLabel.FontColor = [0.9, 0.9, 0.9];
    end
    
    % Update the Fuzzy Logic Decision label
    if isfield(outputs, 'fuzzy_decision')
        decision_text = sprintf('%s (%.1f)', outputs.fuzzy_decision, outputs.fuzzy_desirability_score);
        uiHandles.fuzzyDecisionLabel.Text = strrep(decision_text, '_', ' ');
        
        switch outputs.fuzzy_decision
            case 'AUTOPILOT',        uiHandles.fuzzyDecisionLabel.FontColor = [0.2, 1, 0.2]; % Green
            case 'MANUAL',           uiHandles.fuzzyDecisionLabel.FontColor = [1, 0.9, 0];   % Yellow
            case 'REQUEST_HANDOVER', uiHandles.fuzzyDecisionLabel.FontColor = [1, 0.2, 0.2]; % Red
            otherwise,               uiHandles.fuzzyDecisionLabel.FontColor = [0.9, 0.9, 0.9];
        end
    end
end

function updateCamera(ax, data, fieldName)
    if isfield(data, fieldName) && ~isempty(data.(fieldName))
        try
            img_bytes = base64decode(data.(fieldName));
            temp_file = [tempname '.jpg'];
            fid = fopen(temp_file, 'wb');
            fwrite(fid, img_bytes, 'uint8');
            fclose(fid);
            img = imread(temp_file);
            imshow(img, 'Parent', ax);
            delete(temp_file);
        catch
            % Silently fail on image decode error
        end
    end
end

function [trajectory, gnss_track, imu_history] = updateDataHistories(data, trajectory, gnss_track, imu_history)
    % Update fused trajectory
    fused_state = get_safe(data, 'fused_state', struct('x', NaN, 'y', NaN));
    if isstruct(fused_state) && ~isnan(fused_state.x)
        new_pos = [fused_state.x, fused_state.y];
        trajectory = [trajectory(2:end,:); new_pos];
    end
    
    % Update GNSS track
    gnss_data = get_safe(data, 'gnss', []);
    if isstruct(gnss_data)
        new_gnss = [gnss_data.lat, gnss_data.lon];
        gnss_track = [gnss_track(2:end,:); new_gnss];
    end
    
    % Update IMU history
    imu_b64 = get_safe(data, 'imu', '');
    if ~isempty(imu_b64)
        try
            imu_vals = typecast(base64decode(imu_b64), 'single');
            if numel(imu_vals) >= 3
                imu_history.x = [imu_history.x(2:end), imu_vals(1)];
                imu_history.y = [imu_history.y(2:end), imu_vals(2)];
                imu_history.z = [imu_history.z(2:end), imu_vals(3)];
            end
        catch
            % Silently fail
        end
    end
end

function processAndAnalyzeFrame(frame, latency, uiHandles)
    global carla_outputs;
    persistent last_call_timer last_yaw last_collisions last_lane_invasions current_control_mode;
    
    if isempty(current_control_mode)
        current_control_mode = 'MANUAL';
        logToDashboard(uiHandles, sprintf('Control system initialized. Starting in %s mode.', current_control_mode));
    end
    
    if isempty(last_call_timer)
        time_since_last = 1/20; % Assume 20 Hz on first frame
        last_call_timer = tic;
    else
        time_since_last = toc(last_call_timer);
        last_call_timer = tic;
    end
    
    % --- 1. Network Status ---
    network_status = struct(...
        'latency', latency, ...
        'data_rate', 1 / max(time_since_last, 0.001));

    % --- 2. Sensor Fusion & Health Status ---
    [health_score, sensor_health_data] = computeSensorHealth(frame);
    covariance_trace = computeEkfUncertainty(frame);
    sensor_fusion_status = struct(...
        'fused_state', get_safe(frame, 'fused_state', struct()), ...
        'position_uncertainty', covariance_trace, ...
        'health_score', health_score, ...
        'raw_health', sensor_health_data);
    
    % --- 3. Processed Sensor Data ---
    processed_sensor_data = extractRawSensorData(frame);
    processed_sensor_data.Weather_Severity = computeWeatherSeverity(frame);
    processed_sensor_data.Obstacle_Density = computeObstacleDensity(frame);
    
    % Calculate yaw rate
    rotation_data = get_safe(frame, 'rotation', struct('yaw', 0));
    current_yaw = rotation_data.yaw;
    if isempty(last_yaw), last_yaw = current_yaw; end
    yaw_delta = current_yaw - last_yaw;
    if yaw_delta > 180, yaw_delta = yaw_delta - 360; end
    if yaw_delta < -180, yaw_delta = yaw_delta + 360; end
    processed_sensor_data.yaw_rate = yaw_delta / max(time_since_last, 0.001);
    last_yaw = current_yaw;

    % Detect collision/lane invasion events
    current_collisions = get_safe(frame, 'collisions', 0);
    if isempty(last_collisions), last_collisions = current_collisions; end
    processed_sensor_data.is_collision_event = (current_collisions > last_collisions);
    last_collisions = current_collisions;
    current_lane_invasions = get_safe(frame, 'lane_invasions', 0);
    if isempty(last_lane_invasions), last_lane_invasions = current_lane_invasions; end
    processed_sensor_data.is_lane_invasion_event = (current_lane_invasions > last_lane_invasions);
    last_lane_invasions = current_lane_invasions;
    
    % --- 4. Driver State ---
    driver_attention = computeDriverAttention(frame);
    driver_readiness = computeDriverReadiness(frame);

    % --- 5. FDIR Fallback Logic ---
    threat_data = computeThreatAssessment(frame);
    system_confidence = computeSystemConfidence(frame);
    [fallback, reason] = checkForFallback(latency, health_score, threat_data, ...
        driver_attention, system_confidence, frame);
    if fallback
        logToDashboard(uiHandles, sprintf('[FALLBACK] Reasons: %s', strjoin(reason, ', ')));
    end

    % --- 6. Assemble Final Output Struct ---
    carla_outputs = struct(...
        'network_status', network_status, ...
        'sensor_fusion_status', sensor_fusion_status, ...
        'processed_sensor_data', processed_sensor_data, ...
        'fallback_initiation', fallback, ...
        'driver_attention', driver_attention, ...
        'driver_readiness', driver_readiness, ...
        'system_confidence', system_confidence);

    % --- 7. Call State-Aware Fuzzy Logic Decider ---
    [new_decision, score] = fuzzy_bbna(carla_outputs, current_control_mode);
    
    % Update the persistent state for the next cycle
    current_control_mode = new_decision;
    
    % Store the fuzzy logic output back into the main struct for display
    carla_outputs.fuzzy_decision = current_control_mode;
    carla_outputs.fuzzy_desirability_score = score;
end


%% =======================================================================
%                        NETWORK & UTILITY FUNCTIONS
% ========================================================================
function full_data = processPacket(packet)
    persistent chunkBuffer;
    if isempty(chunkBuffer)
        chunkBuffer = containers.Map('KeyType', 'double', 'ValueType', 'any');
    end
    
    full_data = [];
    if isfield(packet, 'chunk')
        frameId = packet.frame;
        if ~isKey(chunkBuffer, frameId)
            chunkBuffer(frameId) = struct(...
                'chunks', {cell(1, packet.total_chunks)}, 'received_chunks', 0);
        end
        
        chunkData = chunkBuffer(frameId);
        if isempty(chunkData.chunks{packet.chunk + 1})
            chunkData.chunks{packet.chunk + 1} = packet.data;
            chunkData.received_chunks = chunkData.received_chunks + 1;
            chunkBuffer(frameId) = chunkData;
        end
        
        currentChunkData = chunkBuffer(frameId);
        if currentChunkData.received_chunks == packet.total_chunks
            full_json_string = strjoin(currentChunkData.chunks, '');
            try
                full_data = jsondecode(full_json_string);
            catch ME
                disp(['JSON decode failed for frame ' num2str(frameId) ': ' ME.message]);
                full_data = [];
            end
            remove(chunkBuffer, frameId);
        end
    else
        % Packet is not chunked
        full_data = packet;
    end
end

function logToDashboard(uiHandles, message)
    if ~isvalid(uiHandles.fig), return; end
    
    timestamp = datestr(now, 'HH:MM:SS');
    newMessage = sprintf('%s - %s', timestamp, message);
    
    currentLog = uiHandles.logArea.Value;
    if numel(currentLog) > 150
        currentLog = currentLog(end-149:end);
    end
    
    uiHandles.logArea.Value = [currentLog; {newMessage}];
    scroll(uiHandles.logArea, 'bottom');
    drawnow;
end

function decoded = base64decode(str)
    try
        import java.util.Base64;
        decoder = Base64.getDecoder();
        % Remove newlines that can corrupt base64 strings
        clean_str = strrep(str, newline, '');
        decoded = typecast(decoder.decode(uint8(clean_str)), 'uint8');
    catch
        decoded = [];
    end
end

function threat = computeThreatAssessment(frame)
    threat.min_front_distance = inf; 
    threat.closing_velocity = 0;
    
    min_dist_radar = inf;
    radar_data = get_safe(frame, 'radar_front', []);
    if isstruct(radar_data)
        for i=1:numel(radar_data)
            % Check if radar point is roughly in front (azimuth < ~11.5 degrees)
            if abs(radar_data(i).az) < 0.2 && radar_data(i).depth < min_dist_radar
                min_dist_radar = radar_data(i).depth; 
                threat.closing_velocity = radar_data(i).vel;
            end
        end
    end
    
    [min_dist_v2v, ~] = findClosestV2VThreat(frame);
    threat.min_front_distance = min(min_dist_radar, min_dist_v2v);
end

function confidence = computeSystemConfidence(frame)
    confidence.score = 1.0;
    confidence.status_text = 'NOMINAL';
    
    [min_dist_v2v, ~] = findClosestV2VThreat(frame);
    
    min_dist_radar = inf;
    radar_data = get_safe(frame, 'radar_front', []);
    if isstruct(radar_data)
        for i=1:numel(radar_data)
            if abs(radar_data(i).az) < 0.2 && radar_data(i).depth < min_dist_radar
                min_dist_radar = radar_data(i).depth; 
            end
        end
    end
    
    DANGER_THRESHOLD = 20;  % meters
    CLEAR_THRESHOLD = 50;   % meters
    
    % V2V sees a close vehicle, but radar sees nothing. Potential radar failure.
    if min_dist_v2v < DANGER_THRESHOLD && min_dist_radar > CLEAR_THRESHOLD
        confidence.score = 0.2;
        confidence.status_text = 'V2V/Radar Mismatch (Radar Blind)';
        return;
    end
    
    % Radar sees a close vehicle, but V2V sees nothing. Potential V2V failure.
    if min_dist_radar < DANGER_THRESHOLD && min_dist_v2v > CLEAR_THRESHOLD
        confidence.score = 0.4;
        confidence.status_text = 'V2V/Radar Mismatch (V2V Missing)';
        return;
    end
end

function [closest_dist, closing_vel] = findClosestV2VThreat(frame)
    closest_dist = inf;
    closing_vel = 0;
    
    v2v_data = get_safe(frame, 'V2V_Data', []);
    ego_pos = get_safe(frame, 'position', []);
    ego_rot = get_safe(frame, 'rotation', []);
    
    if isempty(v2v_data) || isempty(ego_pos) || isempty(ego_rot), return; end
    
    ego_yaw_rad = deg2rad(ego_rot.yaw);
    forward_vec = [cos(ego_yaw_rad), sin(ego_yaw_rad)];
    
    for i = 1:numel(v2v_data)
        v = v2v_data(i);
        rel_pos_vec = [v.position.x - ego_pos.x, v.position.y - ego_pos.y];
        
        % Check if vehicle is in the forward hemisphere
        dot_product = dot(rel_pos_vec, forward_vec);
        if dot_product > 0 
            dist = norm(rel_pos_vec);
            % Check if vehicle is directly in front (cosine similarity > 0.98)
            if dot_product / dist > 0.98 
                if dist < closest_dist
                    closest_dist = dist;
                end
            end
        end
    end
end

function score = computeDriverAttention(frame)
    persistent time_of_last_input;
    if isempty(time_of_last_input), time_of_last_input = tic; end
    
    score = 1.0;
    controls = get_safe(frame, 'control', struct('steer',0,'throttle',0,'brake',0));
    
    % Only degrade attention score if in manual mode with no input
    if strcmp(get_safe(frame, 'mode', 'autopilot'), 'manual')
        if abs(controls.steer) > 0.01 || controls.throttle > 0.01 || controls.brake > 0.01
            time_of_last_input = tic;
            score = 1.0;
        else
            % Score degrades linearly over 5 seconds of inactivity
            score = max(0, min(1, 1.0 - (toc(time_of_last_input) / 5.0)));
        end
    else
        % In autopilot, assume driver is attentive until proven otherwise
        time_of_last_input = tic;
        score = 1.0;
    end
end

function score = computeDriverReadiness(frame)
    score = 1.0;
    controls = get_safe(frame, 'control', struct('throttle',0,'brake',0,'hand_brake',false));
    
    % Penalize for conflicting inputs (e.g., pressing gas and brake)
    if controls.throttle > 0.1 && controls.brake > 0.1
        score = 0.1;
        return;
    end
    
    % Penalize for trying to drive with handbrake on
    if controls.hand_brake && controls.throttle > 0.1
        score = 0.3;
        return;
    end
end

function value = get_safe(s, field, default_value)
    if isfield(s, field)
        value = s.(field);
    else
        value = default_value;
    end
end

function severity = computeWeatherSeverity(frame)
    weather_str = get_safe(frame, 'weather', 'ClearNoon');
    switch weather_str
        case 'ClearNoon', severity = 0.1;
        case 'CloudyNoon', severity = 0.3;
        case 'WetNoon', severity = 0.6;
        case 'HardRainNoon', severity = 0.9;
        otherwise, severity = 0.1;
    end
end

function density = computeObstacleDensity(frame)
    density = 0;
    obstacles = get_safe(frame, 'Detected_Obstacles', []);
    ego_pos = get_safe(frame, 'position', struct('x', 0, 'y', 0, 'z', 0));
    if isempty(obstacles), return; end
    
    for i = 1:numel(obstacles)
        dist = sqrt((obstacles(i).position.x - ego_pos.x)^2 + (obstacles(i).position.y - ego_pos.y)^2);
        % Count obstacles within a 50m radius
        if dist < 50
            density = density + 1;
        end
    end
end

function [score, raw_health] = computeSensorHealth(frame)
    raw_health = get_safe(frame, 'sensor_health', []);
    if ~isstruct(raw_health)
        score = 0; % No health data, assume worst case
        return;
    end
    
    all_groups = struct2cell(raw_health);
    total_ok = 0;
    total_sensors = 0;
    for i = 1:numel(all_groups)
        status_list = all_groups{i};
        total_sensors = total_sensors + numel(status_list);
        total_ok = total_ok + sum(strcmp(status_list, 'OK'));
    end
    
    if total_sensors > 0
        score = total_ok / total_sensors;
    else
        score = 1; % No sensors reported, assume OK
    end
end

function trace_val = computeEkfUncertainty(frame)
    ekf_cov_data = get_safe(frame, 'ekf_covariance', []);
    if iscell(ekf_cov_data)
        try
            covariance_matrix = cell2mat(cellfun(@cell2mat, ekf_cov_data, 'UniformOutput', false));
            % Trace of the position submatrix (X, Y)
            trace_val = trace(covariance_matrix(1:2, 1:2));
        catch
            trace_val = inf;
        end
    else
        trace_val = inf;
    end
end

function data_out = extractRawSensorData(frame)
    data_out = struct();
    control_data = get_safe(frame, 'control', struct());
    
    all_fields = fieldnames(frame);
    prefixes_to_copy = {'image_', 'lidar_', 'radar_'};
    single_fields_to_copy = {'gnss', 'imu', 'position', 'rotation', ...
        'Detected_Obstacles', 'V2V_Data', 'V2I_Data'};
    
    for i = 1:numel(all_fields)
        field = all_fields{i};
        copy_this_field = false;
        for j = 1:numel(prefixes_to_copy)
            if startsWith(field, prefixes_to_copy{j})
                copy_this_field = true;
                break;
            end
        end
        if ~copy_this_field && ismember(field, single_fields_to_copy)
            copy_this_field = true;
        end
        if copy_this_field
            data_out.(field) = frame.(field);
        end
    end
    
    data_out.speed = get_safe(frame, 'speed', 0);
    data_out.ultrasonic_front = get_safe(frame, 'ultrasonic_front', inf);
    data_out.ultrasonic_back = get_safe(frame, 'ultrasonic_back', inf);
    data_out.throttle_input = get_safe(control_data, 'throttle', 0);
    data_out.brake_input = get_safe(control_data, 'brake', 0);
    data_out.steering_input = get_safe(control_data, 'steer', 0);
end

function [fallback, reason] = checkForFallback(latency, health, threat, attention, confidence, frame)
    fallback = false;
    reason = {};
    
    if latency > 0.2
        fallback = true; reason{end+1} = 'High Latency (>200ms)';
    end
    if health < 0.75
        fallback = true; reason{end+1} = 'Low Sensor Health';
    end
    if threat.min_front_distance < 10 && threat.closing_velocity < -5
        fallback = true; reason{end+1} = 'Imminent Collision Risk';
    end
    if attention < 0.2
        fallback = true; reason{end+1} = 'Low Driver Attention';
    end
    if confidence.score < 0.5
        fallback = true; reason{end+1} = ['Sensor Discrepancy: ' confidence.status_text];
    end
    
    v2i_data = get_safe(frame, 'V2I_Data', struct());
    if isfield(v2i_data, 'traffic_light_state') && ...
            strcmp(v2i_data.traffic_light_state, 'RED') && frame.speed > 5
        fallback = true; reason{end+1} = 'V2I Conflict: Moving on Red Light';
    end
end
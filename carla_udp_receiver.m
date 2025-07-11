function carla_udp_receiver(port)
% CARLA_UDP_RECEIVER - Finalized supervisory controller for CARLA with KPI monitoring.

    % --- Configuration ---
    if nargin < 1
        port = 10000;
    end
    COMMAND_PORT = 10001;
    HISTORY_LENGTH = 100;
    FDIR_RATE_HZ = 20;

    % --- Global Outputs and State ---
    global carla_outputs;
    carla_outputs = struct();

    persistent matlab_control_mode handover_requested_time handoff_confirmed;
    if isempty(matlab_control_mode)
        matlab_control_mode = 'MANUAL'; % States: MANUAL, AWAITING_CONFIRMATION, AUTOPILOT
        handover_requested_time = tic;
        handoff_confirmed = false;
    end

    is_running = true;
    fdirTimer = [];
    command_sender_udp = [];

    % --- Setup UI and UDP ---
    [uiHandles] = setupDashboard(HISTORY_LENGTH, @onCloseRequest);
    logToDashboard(uiHandles, 'Dashboard initialized. Starting UDP receiver...');

    try
        u = udpport("IPV4", "LocalPort", port, "Timeout", 1, "EnablePortSharing", true);
        u.InputBufferSize = 2000000;
        logToDashboard(uiHandles, sprintf('UDP Receiver started on port %d.', port));

        command_sender_udp = udpport("datagram");
        logToDashboard(uiHandles, sprintf('UDP Command Sender targeting 127.0.0.1:%d.', COMMAND_PORT));
    catch ME
        logToDashboard(uiHandles, sprintf('[FATAL] UDP setup failed: %s', ME.message));
        is_running = false;
    end

    % --- SETUP FDIR SYSTEM (TIMER) ---
    try
        fdirTimer = timer(...
            'ExecutionMode', 'fixedRate', ...
            'Period', 1/FDIR_RATE_HZ, ...
            'TimerFcn', @(~,~) run_fdir_cycle(command_sender_udp), ...
            'StartDelay', 2, ...
            'Tag', 'FDIR_Timer', ...
            'StopFcn', @(~,~) disp('FDIR Timer has been stopped.'));
        start(fdirTimer);
        logToDashboard(uiHandles, sprintf('FDIR background monitoring system started at %d Hz.', FDIR_RATE_HZ));
    catch ME
        logToDashboard(uiHandles, sprintf('[FATAL] FDIR Timer setup failed: %s', ME.message));
        is_running = false;
    end

    % --- Data Storage and Initialization ---
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
            else, buffer = ''; end

            for i = 1:numel(jsonObjects)
                jsonStr = jsonObjects{i};
                if isempty(jsonStr), continue; end
                try
                    [packet, handoff_confirmed] = jsondecode_wrapper(jsonStr, handoff_confirmed);
                    if isempty(packet), continue; end
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
            [trajectory, gnss_track, imu_history] = updateDataHistories(frame_data, trajectory, gnss_track, imu_history);

            dt = processAndAnalyzeFrame(frame_data, toc(last_message_time), imu_history);

            if exist('fuzzy_bbna.m', 'file')
                [fuzzy_decision, score] = fuzzy_bbna(carla_outputs, matlab_control_mode);
            else, fuzzy_decision = 'MANUAL'; score = 0; end

            if exist('ModeController.m', 'file')
                [desired_mode, vehicle_command] = ModeController(fuzzy_decision, matlab_control_mode, carla_outputs, dt);
            else, desired_mode = 'MANUAL'; vehicle_command = struct('steer',0,'throttle',0,'brake',1); end

            switch matlab_control_mode
                case 'MANUAL'
                    if strcmp(desired_mode, 'AUTOPILOT')
                        logToDashboard(uiHandles, '[STATE] Conditions met for handover. Requesting confirmation from driver...');
                        matlab_control_mode = 'AWAITING_CONFIRMATION';
                        handover_requested_time = tic;
                        send_simple_command_to_carla(command_sender_udp, 'REQUEST_REMOTE_HANDOVER', 'All checks passed', COMMAND_PORT);
                    end

                case 'AWAITING_CONFIRMATION'
                    if handoff_confirmed
                        logToDashboard(uiHandles, '[STATE] Handover confirmed by driver! Engaging AUTOPILOT.');
                        matlab_control_mode = 'AUTOPILOT';
                        carla_outputs.kpi_transition_speed = toc(handover_requested_time) * 1000;
                        handoff_confirmed = false;
                    elseif toc(handover_requested_time) > 10
                        logToDashboard(uiHandles, '[STATE-WARN] Handover request timed out. Reverting to MANUAL.');
                        matlab_control_mode = 'MANUAL';
                    elseif strcmp(desired_mode, 'MANUAL')
                        logToDashboard(uiHandles, '[STATE] Handover cancelled. Reverting to MANUAL.');
                        matlab_control_mode = 'MANUAL';
                    end

                case 'AUTOPILOT'
                    if strcmp(desired_mode, 'MANUAL')
                        logToDashboard(uiHandles, '[STATE] Conditions no longer met for AUTOPILOT. Disengaging.');
                        matlab_control_mode = 'MANUAL';
                        send_simple_command_to_carla(command_sender_udp, 'DISENGAGED_AUTOPILOT', 'Reverting to manual due to safety/logic.', COMMAND_PORT);
                    end
            end

            if strcmp(matlab_control_mode, 'AUTOPILOT')
                send_control_to_carla(command_sender_udp, vehicle_command, COMMAND_PORT);
            end

            carla_outputs.fuzzy_decision = fuzzy_decision;
            carla_outputs.fuzzy_desirability_score = score;
            carla_outputs.matlab_commanded_mode = matlab_control_mode;

            updateDashboard(uiHandles, frame_data, trajectory, gnss_track, imu_history, carla_outputs);
        end
        pause(0.01);
    end

    % --- CLEANUP ---
    logToDashboard(uiHandles, 'Shutdown sequence initiated...');
    if ~isempty(fdirTimer) && isvalid(fdirTimer), stop(fdirTimer); delete(fdirTimer); end
    if exist('u','var') && isvalid(u), delete(u); end
    if ~isempty(command_sender_udp) && isvalid(command_sender_udp), delete(command_sender_udp); end
    if ishandle(uiHandles.fig), delete(uiHandles.fig); end
    clearvars -global carla_outputs;

    function onCloseRequest(~, ~)
        is_running = false;
    end
end


% =======================================================================
%                      UI
% ========================================================================

function [uiHandles] = setupDashboard(historyLength, closeCallback)
    fig = uifigure('Name', 'CARLA Final Diagnostics Dashboard', 'Position', [50 50, 1600, 950]);
    fig.CloseRequestFcn = closeCallback;
    uiHandles.fig = fig;
    gl = uigridlayout(fig, [3, 4]);
    gl.RowHeight = {250, '2x', 200}; gl.ColumnWidth = {'1x', '1x', '1x', '1x'};
    
    p_speed = uipanel(gl, 'Title', 'Speed (km/h)', 'FontWeight', 'bold'); p_speed.Layout.Row = 1; p_speed.Layout.Column = 1;
    speed_grid = uigridlayout(p_speed, [1 1]);
    uiHandles.speedGauge = uigauge(speed_grid, 'ScaleColors', {[0 .8 .4], [.9 .8 0], [1 .2 .2]}, 'ScaleColorLimits', [0 60; 60 100; 100 160]);
    
    p_imu = uipanel(gl, 'Title', 'IMU - Accelerometer', 'FontWeight', 'bold'); p_imu.Layout.Row = 1; p_imu.Layout.Column = 2;
    uiHandles.imuAx = uiaxes(p_imu); hold(uiHandles.imuAx, 'on'); grid(uiHandles.imuAx, 'on');
    uiHandles.accelXPlot = plot(uiHandles.imuAx, NaN(1, historyLength), 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
    uiHandles.accelYPlot = plot(uiHandles.imuAx, NaN(1, historyLength), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
    uiHandles.accelZPlot = plot(uiHandles.imuAx, NaN(1, historyLength), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
    xlabel(uiHandles.imuAx, 'Frames Ago'); ylabel(uiHandles.imuAx, 'm/s^2'); legend(uiHandles.imuAx);
    
    p_gnss = uipanel(gl, 'Title', 'GNSS Trajectory', 'FontWeight', 'bold'); p_gnss.Layout.Row = 1; p_gnss.Layout.Column = 3;
    uiHandles.gnssAx = uiaxes(p_gnss); hold(uiHandles.gnssAx, 'on'); grid(uiHandles.gnssAx, 'on'); axis(uiHandles.gnssAx, 'equal');
    xlabel(uiHandles.gnssAx, 'Longitude'); ylabel(uiHandles.gnssAx, 'Latitude');
    uiHandles.gnssPlot = plot(uiHandles.gnssAx, NaN, NaN, '-m', 'LineWidth', 1.5);
    
    p_diag = uipanel(gl, 'Title', 'Diagnostics', 'FontWeight', 'bold'); p_diag.Layout.Row = 1; p_diag.Layout.Column = 4;
    g_diag_metrics = uigridlayout(p_diag, [11, 2], 'ColumnWidth', {'fit', '1x'}); g_diag_metrics.RowSpacing = 2; g_diag_metrics.Padding = [5 5 5 5];
    uilabel(g_diag_metrics, 'Text', 'Frame:', 'FontSize', 10); uiHandles.frameLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold', 'FontSize', 10);
    uilabel(g_diag_metrics, 'Text', 'Latency:', 'FontSize', 10); uiHandles.latencyLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold', 'FontSize', 10);
    uilabel(g_diag_metrics, 'Text', 'Vehicle Mode:', 'FontSize', 10); uiHandles.modeLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold', 'FontSize', 10);
    uilabel(g_diag_metrics, 'Text', 'MATLAB Mode:', 'FontSize', 10); uiHandles.matlabModeLabel = uilabel(g_diag_metrics, 'Text', 'STARTING', 'FontWeight', 'bold', 'FontSize', 10);
    uilabel(g_diag_metrics, 'Text', 'Map:', 'FontSize', 10); uiHandles.mapLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold', 'FontSize', 10);
    uilabel(g_diag_metrics, 'Text', 'Weather:', 'FontSize', 10); uiHandles.weatherLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold', 'FontSize', 10);
    uilabel(g_diag_metrics, 'Text', 'Health:', 'FontSize', 10); uiHandles.healthLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold', 'FontSize', 10);
    uilabel(g_diag_metrics, 'Text', 'Fallback:', 'FontSize', 10); uiHandles.fallbackLabel = uilabel(g_diag_metrics, 'Text', 'IDLE', 'FontWeight', 'bold', 'FontColor', 'g', 'FontSize', 10);
    uilabel(g_diag_metrics, 'Text', 'Traffic Light:', 'FontSize', 10); uiHandles.trafficLightLabel = uilabel(g_diag_metrics, 'Text', 'N/A', 'FontWeight', 'bold', 'FontSize', 10);
    uilabel(g_diag_metrics, 'Text', 'Sys. Conf:', 'FontSize', 10); uiHandles.confidenceLabel = uilabel(g_diag_metrics, 'Text', 'NOMINAL', 'FontWeight', 'bold', 'FontSize', 10);
    uilabel(g_diag_metrics, 'Text', 'Fuzzy Desire:', 'FontWeight', 'bold', 'FontSize', 10); uiHandles.fuzzyDecisionLabel = uilabel(g_diag_metrics, 'Text', 'STARTING...', 'FontWeight', 'bold', 'FontSize', 10);
    
    p_cameras = uipanel(gl, 'Title', 'Camera Feeds', 'FontWeight', 'bold'); p_cameras.Layout.Row = 2; p_cameras.Layout.Column = [1 2];
    g_cameras = uigridlayout(p_cameras, [2 3]);
    uiHandles.camLeftAx = uiaxes(g_cameras, 'XTick', [], 'YTick', []); title(uiHandles.camLeftAx, 'Left'); uiHandles.camFrontAx = uiaxes(g_cameras, 'XTick', [], 'YTick', []); title(uiHandles.camFrontAx, 'Front'); uiHandles.camRightAx = uiaxes(g_cameras, 'XTick', [], 'YTick', []); title(uiHandles.camRightAx, 'Right'); uiHandles.camInteriorAx = uiaxes(g_cameras, 'XTick', [], 'YTick', []); title(uiHandles.camInteriorAx, 'Interior'); uiHandles.camRearAx = uiaxes(g_cameras, 'XTick', [], 'YTick', []); title(uiHandles.camRearAx, 'Back');
    
    p_traj = uipanel(gl, 'Title', 'Vehicle Trajectory (Fused)', 'FontWeight', 'bold'); p_traj.Layout.Row = 2; p_traj.Layout.Column = 3;
    uiHandles.trajAx = uiaxes(p_traj); hold(uiHandles.trajAx, 'on'); grid(uiHandles.trajAx, 'on'); axis(uiHandles.trajAx, 'equal'); xlabel(uiHandles.trajAx, 'X (m)'); ylabel(uiHandles.trajAx, 'Y (m)');
    uiHandles.trajPlot = plot(uiHandles.trajAx, NaN, NaN, '-c', 'LineWidth', 2, 'DisplayName', 'Ego Path'); uiHandles.currentPosPlot = plot(uiHandles.trajAx, NaN, NaN, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Ego Vehicle'); uiHandles.v2vPlot = plot(uiHandles.trajAx, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5, 'DisplayName', 'Other Vehicles'); legend(uiHandles.trajAx);
    
    p_lidar = uipanel(gl, 'Title', '3D LiDAR & Object Detections', 'FontWeight', 'bold'); p_lidar.Layout.Row = 2; p_lidar.Layout.Column = 4;
    uiHandles.lidarAx = uiaxes(p_lidar); hold(uiHandles.lidarAx, 'on'); grid(uiHandles.lidarAx, 'on'); axis(uiHandles.lidarAx, 'equal'); xlabel(uiHandles.lidarAx, 'X (m)'); ylabel(uiHandles.lidarAx, 'Y (m)'); zlabel(uiHandles.lidarAx, 'Z (m)'); view(uiHandles.lidarAx, -45, 30);
    uiHandles.lidarPlot = scatter3(uiHandles.lidarAx, NaN, NaN, NaN, 10, NaN, 'filled', 'DisplayName', 'LiDAR Points'); uiHandles.obstaclePlot = scatter3(uiHandles.lidarAx, NaN, NaN, NaN, 100, 'bo', 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Obstacles'); legend(uiHandles.lidarAx);
    
    p_log = uipanel(gl, 'Title', 'System Log', 'FontWeight', 'bold'); p_log.Layout.Row = 3; p_log.Layout.Column = [3 4];
    log_grid = uigridlayout(p_log, [1 1]);
    uiHandles.logArea = uitextarea(log_grid, 'Value', {''}, 'Editable', 'off', 'FontName', 'Monospaced', 'BackgroundColor', [0.1 0.1 0.1], 'FontColor', [0.9 0.9 0.9]);
    
    p_kpi = uipanel(gl, 'Title', 'Key Performance Indicators (KPIs)', 'FontWeight', 'bold'); p_kpi.Layout.Row = 3; p_kpi.Layout.Column = [1 2];
    g_kpi = uigridlayout(p_kpi, [6, 2], 'ColumnWidth', {'fit', '1x'}, 'Padding', [10 10 10 10]);
    uilabel(g_kpi, 'Text', 'Transition Speed (ms):', 'FontWeight', 'bold'); uiHandles.kpiSpeedLabel = uilabel(g_kpi, 'Text', 'N/A');
    uilabel(g_kpi, 'Text', 'Safety (Peak Lat Accel):', 'FontWeight', 'bold'); uiHandles.kpiSafetyLabel = uilabel(g_kpi, 'Text', 'N/A');
    uilabel(g_kpi, 'Text', 'Stability (Steer StDev):', 'FontWeight', 'bold'); uiHandles.kpiStabilityLabel = uilabel(g_kpi, 'Text', 'N/A');
    uilabel(g_kpi, 'Text', 'Precision (Pos Error m):', 'FontWeight', 'bold'); uiHandles.kpiPrecisionLabel = uilabel(g_kpi, 'Text', 'N/A');
    uilabel(g_kpi, 'Text', 'Adaptability (Speed Error m/s):', 'FontWeight', 'bold'); uiHandles.kpiAdaptabilityLabel = uilabel(g_kpi, 'Text', 'N/A');
    uilabel(g_kpi, 'Text', 'Comp. Efficiency (Cycle ms):', 'FontWeight', 'bold'); uiHandles.kpiEfficiencyLabel = uilabel(g_kpi, 'Text', 'N/A');
end

%% =======================================================================
%                      DATA PROCESSING AND UPDATING
% ========================================================================
function updateDashboard(uiHandles, data, trajectory, gnss_track, imu_history, outputs)
    if ~isvalid(uiHandles.fig), return; end
    if isfield(data, 'speed'), uiHandles.speedGauge.Value = data.speed; end
    set(uiHandles.accelXPlot, 'YData', imu_history.x); set(uiHandles.accelYPlot, 'YData', imu_history.y); set(uiHandles.accelZPlot, 'YData', imu_history.z);
    set(uiHandles.gnssPlot, 'XData', gnss_track(:,2), 'YData', gnss_track(:,1));
    set(uiHandles.trajPlot, 'XData', trajectory(:,1), 'YData', trajectory(:,2)); set(uiHandles.currentPosPlot, 'XData', trajectory(end,1), 'YData', trajectory(end,2));
    if isfield(data, 'frame'), uiHandles.frameLabel.Text = num2str(data.frame); end
    if isfield(data, 'mode'), uiHandles.modeLabel.Text = strrep(data.mode, '_', ' '); end
    if isfield(data, 'map'), uiHandles.mapLabel.Text = data.map; end
    if isfield(data, 'weather'), uiHandles.weatherLabel.Text = data.weather; end
    if isfield(outputs, 'network_status'), uiHandles.latencyLabel.Text = sprintf('%.1f ms', outputs.network_status.latency * 1000); end
    if isfield(outputs, 'sensor_fusion_status'), uiHandles.healthLabel.Text = sprintf('%.0f%%', outputs.sensor_fusion_status.health_score * 100); end
    if isfield(outputs, 'fallback_initiation'), if outputs.fallback_initiation, uiHandles.fallbackLabel.Text = 'TRIGGERED'; uiHandles.fallbackLabel.FontColor = 'r'; else, uiHandles.fallbackLabel.Text = 'IDLE'; uiHandles.fallbackLabel.FontColor = 'g'; end; end
    if isfield(outputs, 'system_confidence'), conf = outputs.system_confidence; uiHandles.confidenceLabel.Text = strrep(conf.status_text, '_', ' '); if conf.score > 0.8, uiHandles.confidenceLabel.FontColor = [0.2 1 0.2]; elseif conf.score > 0.5, uiHandles.confidenceLabel.FontColor = [1 0.9 0]; else, uiHandles.confidenceLabel.FontColor = [1 0.2 0.2]; end; end
    updateCamera(uiHandles.camFrontAx, data, 'image_front'); updateCamera(uiHandles.camRearAx, data, 'image_back'); updateCamera(uiHandles.camLeftAx, data, 'image_left'); updateCamera(uiHandles.camRightAx, data, 'image_right'); updateCamera(uiHandles.camInteriorAx, data, 'image_interior');
    if isfield(data, 'lidar_roof') && ~isempty(data.lidar_roof), try, points = typecast(base64decode(data.lidar_roof), 'single'); if mod(numel(points), 4) == 0, points = reshape(points, 4, [])'; xyz = points(1:20:end, 1:3); set(uiHandles.lidarPlot, 'XData', xyz(:,1), 'YData', xyz(:,2), 'ZData', xyz(:,3), 'CData', xyz(:,3)); end; catch; end; end
    obstacles = get_safe(data, 'Detected_Obstacles', []); if ~isempty(obstacles), num_obstacles = numel(obstacles); obs_x = nan(num_obstacles, 1); obs_y = nan(num_obstacles, 1); obs_z = nan(num_obstacles, 1); for i = 1:num_obstacles, obs_x(i) = obstacles(i).position.x; obs_y(i) = obstacles(i).position.y; obs_z(i) = obstacles(i).position.z; end; set(uiHandles.obstaclePlot, 'XData', obs_x, 'YData', obs_y, 'ZData', obs_z); else, set(uiHandles.obstaclePlot, 'XData', NaN, 'YData', NaN, 'ZData', NaN); end
    v2v_data = get_safe(data, 'V2V_Data', []); if ~isempty(v2v_data), num_vehicles = numel(v2v_data); v2v_x = nan(num_vehicles, 1); v2v_y = nan(num_vehicles, 1); for i = 1:num_vehicles, v2v_x(i) = v2v_data(i).position.x; v2v_y(i) = v2v_data(i).position.y; end; set(uiHandles.v2vPlot, 'XData', v2v_x, 'YData', v2v_y); else, set(uiHandles.v2vPlot, 'XData', NaN, 'YData', NaN); end
    v2i_data = get_safe(data, 'V2I_Data', struct()); if isfield(v2i_data, 'traffic_light_state') && ~isempty(v2i_data.traffic_light_state), state = v2i_data.traffic_light_state; uiHandles.trafficLightLabel.Text = state; switch upper(state), case 'RED', uiHandles.trafficLightLabel.FontColor = [1, 0.2, 0.2]; case 'YELLOW', uiHandles.trafficLightLabel.FontColor = [1, 0.9, 0]; case 'GREEN', uiHandles.trafficLightLabel.FontColor = [0.2, 1, 0.2]; otherwise, uiHandles.trafficLightLabel.FontColor = [0.8, 0.8, 0.8]; end; else, uiHandles.trafficLightLabel.Text = 'N/A'; uiHandles.trafficLightLabel.FontColor = [0.9, 0.9, 0.9]; end
    if isfield(outputs, 'fuzzy_decision'), decision_text = sprintf('%s (%.1f)', outputs.fuzzy_decision, outputs.fuzzy_desirability_score); uiHandles.fuzzyDecisionLabel.Text = strrep(decision_text, '_', ' '); switch outputs.fuzzy_decision, case 'AUTOPILOT', uiHandles.fuzzyDecisionLabel.FontColor = [0.2, 1, 0.2]; case 'MANUAL', uiHandles.fuzzyDecisionLabel.FontColor = [1, 0.2, 0.2]; otherwise, uiHandles.fuzzyDecisionLabel.FontColor = [0.9, 0.9, 0.9]; end; end
    if isfield(outputs, 'matlab_commanded_mode'), mode_text = outputs.matlab_commanded_mode; uiHandles.matlabModeLabel.Text = strrep(mode_text, '_', ' '); switch mode_text, case 'AUTOPILOT', uiHandles.matlabModeLabel.FontColor = [0.2, 1, 0.2]; case 'MANUAL', uiHandles.matlabModeLabel.FontColor = [1, 0.9, 0]; case 'AWAITING_CONFIRMATION', uiHandles.matlabModeLabel.FontColor = [0.2, 0.8, 1]; otherwise, uiHandles.matlabModeLabel.FontColor = [0.9, 0.9, 0.9]; end; end
    if isfield(outputs, 'kpi_transition_speed'), uiHandles.kpiSpeedLabel.Text = sprintf('%.1f', outputs.kpi_transition_speed); end
    if isfield(outputs, 'kpi_safety_lat_accel'), uiHandles.kpiSafetyLabel.Text = sprintf('%.3f', outputs.kpi_safety_lat_accel); end
    if isfield(outputs, 'kpi_stability_steer_std'), uiHandles.kpiStabilityLabel.Text = sprintf('%.4f', outputs.kpi_stability_steer_std); end
    if isfield(outputs, 'kpi_precision_error'), uiHandles.kpiPrecisionLabel.Text = sprintf('%.3f', outputs.kpi_precision_error); end
    if isfield(outputs, 'kpi_adaptability_speed_error'), uiHandles.kpiAdaptabilityLabel.Text = sprintf('%.3f', outputs.kpi_adaptability_speed_error); end
    if isfield(outputs, 'kpi_efficiency_cycle_time'), uiHandles.kpiEfficiencyLabel.Text = sprintf('%.1f', outputs.kpi_efficiency_cycle_time); end
end

function [trajectory, gnss_track, imu_history] = updateDataHistories(data, trajectory, gnss_track, imu_history)
    fused_state = get_safe(data, 'fused_state', struct('x', NaN, 'y', NaN)); if isstruct(fused_state) && ~isnan(fused_state.x), trajectory = [trajectory(2:end,:); [fused_state.x, fused_state.y]]; end
    gnss_data = get_safe(data, 'gnss', []); if isstruct(gnss_data), gnss_track = [gnss_track(2:end,:); [gnss_data.lat, gnss_data.lon]]; end
    imu_b64 = get_safe(data, 'imu', ''); if ~isempty(imu_b64), try, imu_vals = typecast(base64decode(imu_b64), 'single'); if numel(imu_vals) >= 3, imu_history.x = [imu_history.x(2:end), imu_vals(1)]; imu_history.y = [imu_history.y(2:end), imu_vals(2)]; imu_history.z = [imu_history.z(2:end), imu_vals(3)]; end; catch; end; end
end

function [time_since_last] = processAndAnalyzeFrame(frame, latency, imu_history)
    global carla_outputs;
    persistent kpi_efficiency_timer;
    if isempty(kpi_efficiency_timer), kpi_efficiency_timer = tic; end
    carla_outputs.kpi_efficiency_cycle_time = toc(kpi_efficiency_timer) * 1000;
    kpi_efficiency_timer = tic;
    persistent last_call_timer last_yaw last_collisions last_lane_invasions;
    if isempty(last_call_timer), time_since_last = 1/20; last_call_timer = tic; else, time_since_last = toc(last_call_timer); last_call_timer = tic; end
    network_status = struct('latency', latency, 'data_rate', 1 / max(time_since_last, 0.001));
    [health_score, sensor_health_data] = computeSensorHealth(frame);
    covariance_trace = computeEkfUncertainty(frame);
    sensor_fusion_status = struct('fused_state', get_safe(frame, 'fused_state', struct()), 'position_uncertainty', covariance_trace, 'health_score', health_score, 'raw_health', sensor_health_data);
    processed_sensor_data = extractRawSensorData(frame);
    weather_severity = computeWeatherSeverity(frame);
    threat_level = computeThreatLevel(frame);
    simple_density = computeSimpleObstacleDensity(frame);
    actual_speed_ms = get_safe(frame, 'speed', 0) / 3.6;
    v2i_data = get_safe(frame, 'V2I_Data', struct());
    SPEED_LIMIT_MS = 15;
    weather_factor = 1.0 - (weather_severity * 0.7);
    density_factor = 1.0 - (min(simple_density, 5) * 0.1);
    recommended_speed_ms = SPEED_LIMIT_MS * weather_factor * density_factor;
    speed_error = abs(actual_speed_ms - recommended_speed_ms);
    persistent speed_error_history;
    if isempty(speed_error_history), speed_error_history = nan(1,100); end
    speed_error_history = [speed_error_history(2:end), speed_error];
    carla_outputs.kpi_adaptability_speed_error = mean(speed_error_history, 'omitnan');
    system_confidence = computeSystemConfidence(health_score, covariance_trace, threat_level, weather_severity);
    processed_sensor_data.Weather_Severity = weather_severity;
    processed_sensor_data.Obstacle_Threat = threat_level;
    processed_sensor_data.Obstacle_Density = simple_density;
    rotation_data = get_safe(frame, 'rotation', struct('yaw', 0)); current_yaw = rotation_data.yaw; if isempty(last_yaw), last_yaw = current_yaw; end
    yaw_delta = current_yaw - last_yaw; if yaw_delta > 180, yaw_delta = yaw_delta - 360; end; if yaw_delta < -180, yaw_delta = yaw_delta + 360; end
    processed_sensor_data.yaw_rate = yaw_delta / max(time_since_last, 0.001); last_yaw = current_yaw;
    current_collisions = get_safe(frame, 'collisions', 0); if isempty(last_collisions), last_collisions = current_collisions; end
    processed_sensor_data.is_collision_event = (current_collisions > last_collisions); last_collisions = current_collisions;
    current_lane_invasions = get_safe(frame, 'lane_invasions', 0); if isempty(last_lane_invasions), last_lane_invasions = current_lane_invasions; end
    is_lane_invasion = (current_lane_invasions > last_lane_invasions);
    processed_sensor_data.is_lane_invasion_event = is_lane_invasion; last_lane_invasions = current_lane_invasions;
    driver_attention = computeDriverAttention(frame, is_lane_invasion);
    driver_readiness = computeDriverReadiness(frame, driver_attention, threat_level);
    ego_pos = get_safe(frame, 'fused_state', struct('x',0,'y',0));
    ego_rot = get_safe(frame, 'rotation', struct('yaw',0));
    real_lane_data = get_safe(frame, 'lane_waypoints', []);
    [planned_waypoints, intelligent_target_speed] = simulate_path_with_toolbox(ego_pos, ego_rot, actual_speed_ms, real_lane_data, v2i_data, threat_level);
    processed_sensor_data.lane_waypoints = planned_waypoints;
    carla_outputs.network_status = network_status;
    carla_outputs.sensor_fusion_status = sensor_fusion_status;
    carla_outputs.processed_sensor_data = processed_sensor_data;
    carla_outputs.driver_attention = driver_attention;
    carla_outputs.driver_readiness = driver_readiness;
    carla_outputs.system_confidence = system_confidence;
    carla_outputs.intelligent_target_speed_ms = intelligent_target_speed;
    carla_outputs.control = get_safe(frame, 'control', struct());
    carla_outputs.speed = get_safe(frame, 'speed', 0);
    carla_outputs.rotation = get_safe(frame, 'rotation', struct());
    carla_outputs.kpi_safety_lat_accel = max(abs(imu_history.y));
    persistent steer_history;
    if isempty(steer_history), steer_history = nan(1, 50); end
    steer_history = [steer_history(2:end), get_safe(processed_sensor_data, 'steering_input', 0)];
    carla_outputs.kpi_stability_steer_std = std(steer_history, 'omitnan');
    fused_pos = get_safe(sensor_fusion_status, 'fused_state', []);
    true_pos = get_safe(frame, 'position', []);
    if ~isempty(fused_pos) && isfield(fused_pos, 'x') && ~isempty(true_pos), carla_outputs.kpi_precision_error = sqrt((fused_pos.x - true_pos.x)^2 + (fused_pos.y - true_pos.y)^2); end
    carla_outputs.sensor_health = get_safe(frame, 'sensor_health', struct());
    carla_outputs.active_sensor_indices = get_safe(frame, 'active_sensor_indices', struct());
end

%% =======================================================================
%                        NETWORK & UTILITY FUNCTIONS
% ========================================================================
function send_control_to_carla(udp_sender, command, port)
    persistent command_id;
    if isempty(command_id), command_id = 0; end
    if ~isempty(udp_sender) && isvalid(udp_sender)
        packet = struct('command', 'SET_VEHICLE_CONTROL', 'control', command, 'command_id', command_id, 'api_key', 'SECRET_CARLA_KEY_123');
        json_packet = jsonencode(packet);
        write(udp_sender, json_packet, "char", "127.0.0.1", port);
        command_id = command_id + 1;
    end
end

function send_simple_command_to_carla(udp_sender, command_type, reason, port)
    if ~isempty(udp_sender) && isvalid(udp_sender)
        packet = struct('command', command_type, 'reason', reason, 'api_key', 'SECRET_CARLA_KEY_123');
        json_packet = jsonencode(packet);
        write(udp_sender, json_packet, "char", "127.0.0.1", port);
    end
end

function full_data = processPacket(packet)
    persistent chunkBuffer;
    if isempty(chunkBuffer), chunkBuffer = containers.Map('KeyType', 'double', 'ValueType', 'any'); end
    full_data = [];
    PACKET_TIMEOUT_S = 1.0;
    keys_to_remove = {};
    all_keys = keys(chunkBuffer);
    for i = 1:length(all_keys), key = all_keys{i}; if toc(chunkBuffer(key).timestamp) > PACKET_TIMEOUT_S, keys_to_remove{end+1} = key; end; end
    if ~isempty(keys_to_remove), remove(chunkBuffer, keys_to_remove); end
    if isfield(packet, 'chunk')
        frameId = packet.frame;
        if ~isKey(chunkBuffer, frameId), chunkBuffer(frameId) = struct('chunks', {cell(1, packet.total_chunks)}, 'received_chunks', 0, 'timestamp', tic); end
        frame_buffer = chunkBuffer(frameId);
        if isempty(frame_buffer.chunks{packet.chunk + 1})
            frame_buffer.chunks{packet.chunk + 1} = packet.data;
            frame_buffer.received_chunks = frame_buffer.received_chunks + 1;
            chunkBuffer(frameId) = frame_buffer;
        end
        if frame_buffer.received_chunks == packet.total_chunks, full_json_string = strjoin(frame_buffer.chunks, ''); try, full_data = jsondecode(full_json_string); catch, full_data = []; end; remove(chunkBuffer, frameId); end
    else, full_data = packet; end
end

function [s, handoff_confirmed_out] = jsondecode_wrapper(json_str, handoff_confirmed_in)
    s = jsondecode(json_str);
    handoff_confirmed_out = handoff_confirmed_in;
    if isfield(s, 'type') && strcmp(s.type, 'ack'), s = []; return; end
    if isfield(s, 'command') && strcmp(s.command, 'CONFIRM_REMOTE_HANDOVER')
        if isfield(s, 'api_key') && strcmp(s.api_key, 'SECRET_CARLA_KEY_123'), handoff_confirmed_out = true; end
        s = []; return;
    end
end

function logToDashboard(uiHandles, message)
    if ~isvalid(uiHandles.fig), return; end
    timestamp = datestr(now, 'HH:MM:SS'); newMessage = sprintf('%s - %s', timestamp, message);
    currentLog = uiHandles.logArea.Value; if numel(currentLog) > 150, currentLog = currentLog(end-149:end); end
    uiHandles.logArea.Value = [currentLog; {newMessage}]; scroll(uiHandles.logArea, 'bottom'); drawnow;
end

function decoded = base64decode(str)
    try, import java.util.Base64; decoder = Base64.getDecoder(); clean_str = strrep(str, newline, ''); decoded = typecast(decoder.decode(uint8(clean_str)), 'uint8'); catch, decoded = []; end
end

function updateCamera(ax, data, fieldName)
    if isfield(data, fieldName) && ~isempty(data.(fieldName))
        try, img_bytes = base64decode(data.(fieldName)); temp_file = [tempname '.jpg']; fid = fopen(temp_file, 'wb'); fwrite(fid, img_bytes, 'uint8'); fclose(fid); img = imread(temp_file); imshow(img, 'Parent', ax); delete(temp_file); catch; end
    end
end

function val = get_safe(s, field, default)
    if isfield(s, field) && ~isempty(s.(field)), val = s.(field); else, val = default; end
end

%% =======================================================================
%                    ANALYSIS, PERCEPTION, & UTILITY FUNCTIONS
% ========================================================================
function density = computeSimpleObstacleDensity(frame)
    density = 0; obstacles = get_safe(frame, 'Detected_Obstacles', []); ego_pos = get_safe(frame, 'position', struct('x', 0, 'y', 0, 'z', 0));
    if isempty(obstacles) || ~isfield(ego_pos, 'x'), return; end
    for i = 1:numel(obstacles), dist = sqrt((obstacles(i).position.x - ego_pos.x)^2 + (obstacles(i).position.y - ego_pos.y)^2); if dist < 50, density = density + 1; end; end
end
function threat = computeThreatLevel(frame)
    threat = 0; obstacles = get_safe(frame, 'Detected_Obstacles', []); ego_pos = get_safe(frame, 'fused_state', []); ego_rot = get_safe(frame, 'rotation', []);
    if isempty(obstacles) || isempty(ego_pos) || isempty(ego_rot), return; end
    ego_yaw_rad = deg2rad(ego_rot.yaw); forward_vec = [cos(ego_yaw_rad), sin(ego_yaw_rad)];
    for i = 1:numel(obstacles)
        obs = obstacles(i); rel_pos_vec = [obs.position.x - ego_pos.x, obs.position.y - ego_pos.y]; dist = norm(rel_pos_vec);
        if dist > 50 || dist < 0.1, continue; end
        rel_pos_dir = rel_pos_vec / dist; alignment = dot(forward_vec, rel_pos_dir);
        if alignment > 0.5, distance_threat = (1 / dist) * 10; alignment_threat = alignment^2; threat = threat + (distance_threat * alignment_threat); end
    end
end
function confidence = computeSystemConfidence(health_score, uncertainty, threat_level, weather_severity)
    score = health_score; status_text = 'NOMINAL';
    if uncertainty > 1.0, score = score * 0.7; status_text = 'HIGH_UNCERTAINTY'; end
    if threat_level > 10.0, score = score * (1 - min(threat_level / 50, 0.8)); status_text = 'HIGH_THREAT_ENV'; end
    if weather_severity > 0.7, score = score * 0.8; if strcmp(status_text, 'NOMINAL'), status_text = 'POOR_WEATHER'; end; end
    confidence.score = max(0, score); confidence.status_text = status_text;
end
function score = computeDriverAttention(frame, is_lane_invasion)
    persistent last_image_hash time_head_away;
    if isempty(last_image_hash), last_image_hash = 0; time_head_away = tic; end
    score = 1.0; img_b64 = get_safe(frame, 'image_interior', ''); head_is_forward = true;
    if ~isempty(img_b64), try, img_bytes = base64decode(img_b64); current_hash = sum(img_bytes(1:10:end)); if abs(current_hash - last_image_hash) > numel(img_bytes) * 0.1 && last_image_hash ~= 0, head_is_forward = false; end; last_image_hash = current_hash; catch, head_is_forward = true; end; end
    if head_is_forward, time_head_away = tic; else, score = score - min(1.0, toc(time_head_away) / 3.0); end
    if is_lane_invasion, score = score * 0.2; end; score = max(0, score);
end
function score = computeDriverReadiness(frame, attention_score, threat_level)
    score = 1.0; controls = get_safe(frame, 'control', struct('throttle',0,'brake',0,'hand_brake',false)); v2i_data = get_safe(frame, 'V2I_Data', struct());
    if controls.throttle > 0.1 && controls.brake > 0.1, score = 0.1; end
    if controls.hand_brake && controls.throttle > 0.1, score = 0.3; end
    traffic_light_state = get_safe(v2i_data, 'traffic_light_state', 'GREEN');
    if strcmpi(traffic_light_state, 'RED') && controls.throttle > 0.1, score = min(score, 0.2); end
    if threat_level > 10.0 && controls.brake < 0.1, score = min(score, 0.4); end
    score = min(score, attention_score);
end
function [waypoints, target_speed_ms] = simulate_path_with_toolbox(ego_pos, ego_rot, ego_speed_ms, real_lane_waypoints, v2i_data, threat_level)
    if ~isempty(real_lane_waypoints) && ismatrix(real_lane_waypoints) && size(real_lane_waypoints, 2) == 2 && size(real_lane_waypoints, 1) > 1, ref_wps = real_lane_waypoints; else, ref_wps = [ ego_pos.x - 1, ego_pos.y; ego_pos.x + 100, ego_pos.y]; end
    refPath = referencePathFrenet(ref_wps); ego_state_global = [ego_pos.x, ego_pos.y, deg2rad(ego_rot.yaw), 0, 0, ego_speed_ms]; frenet_state_current = global2frenet(refPath, ego_state_global);
    target_speed_ms = ego_speed_ms; traffic_light_state = get_safe(v2i_data, 'traffic_light_state', 'GREEN');
    if strcmpi(traffic_light_state, 'RED'), target_speed_ms = 0; end
    if threat_level > 15.0, target_speed_ms = 0; elseif threat_level > 8.0, target_speed_ms = min(target_speed_ms, 5.0); end
    lookahead_dist = 20.0; frenet_state_target = [frenet_state_current(1) + lookahead_dist, 0, 0, target_speed_ms, 0, 0];
    timeSpan = lookahead_dist / max(ego_speed_ms, 5); trajGen = trajectoryGeneratorFrenet(refPath);
    [~, trajectory] = connect(trajGen, frenet_state_current, frenet_state_target, timeSpan);
    if isempty(trajectory.Trajectory), waypoints = ref_wps; else, waypoints = trajectory.Trajectory(:, 1:2); end
end
function severity = computeWeatherSeverity(frame)
    weather_str = get_safe(frame, 'weather', 'ClearNoon');
    switch weather_str, case 'ClearNoon', severity = 0.1; case 'CloudyNoon', severity = 0.3; case 'WetNoon', severity = 0.6; case 'HardRainNoon', severity = 0.9; otherwise, severity = 0.1; end
end
function [score, raw_health] = computeSensorHealth(frame)
    raw_health = get_safe(frame, 'sensor_health', []); if ~isstruct(raw_health), score = 0; return; end
    all_groups = struct2cell(raw_health); total_ok = 0; total_sensors = 0;
    for i = 1:numel(all_groups), status_list = all_groups{i}; total_sensors = total_sensors + numel(status_list); total_ok = total_ok + sum(strcmp(status_list, 'OK')); end
    if total_sensors > 0, score = total_ok / total_sensors; else, score = 1; end
end
function trace_val = computeEkfUncertainty(frame)
    ekf_cov_data = get_safe(frame, 'ekf_covariance', []);
    if iscell(ekf_cov_data), try, covariance_matrix = cell2mat(cellfun(@cell2mat, ekf_cov_data, 'UniformOutput', false)); trace_val = trace(covariance_matrix(1:2, 1:2)); catch, trace_val = inf; end; else, trace_val = inf; end
end
function data_out = extractRawSensorData(frame)
    data_out = struct(); control_data = get_safe(frame, 'control', struct()); all_fields = fieldnames(frame); prefixes = {'image_', 'lidar_', 'radar_'}; singles = {'gnss', 'imu', 'position', 'rotation', 'Detected_Obstacles', 'V2V_Data', 'V2I_Data'};
    for i = 1:numel(all_fields), field = all_fields{i}; copy = false; for j = 1:numel(prefixes), if startsWith(field, prefixes{j}), copy = true; break; end; end
        if ~copy && ismember(field, singles), copy = true; end; if copy, data_out.(field) = frame.(field); end; end
    data_out.speed = get_safe(frame, 'speed', 0); data_out.ultrasonic_front = get_safe(frame, 'ultrasonic_front', inf); data_out.ultrasonic_back = get_safe(frame, 'ultrasonic_back', inf);
    data_out.throttle_input = get_safe(control_data, 'throttle', 0); data_out.brake_input = get_safe(control_data, 'brake', 0); data_out.steering_input = get_safe(control_data, 'steer', 0);
end

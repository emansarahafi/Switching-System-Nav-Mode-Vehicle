function carla_udp_receiver(port)
% CARLA_UDP_RECEIVER - Main entry point for the CARLA diagnostics dashboard.
% This script sets up a modern UI, receives data from a Python CARLA client
% over UDP, processes it, and continuously updates the dashboard components.
% It also computes a final 'carla_outputs' struct suitable for a fuzzy logic
% controller to manage autonomous driving state transitions.

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
            'ExecutionMode', 'fixedRate', ...         % Run repeatedly
            'Period', 1/FDIR_RATE_HZ, ...              % Time between checks
            'TimerFcn', @(~,~) run_fdir_cycle(), ...   % Function to call
            'StartDelay', 2, ...                      % Wait 2s for first data
            'Tag', 'FDIR_Timer', ...                  % Easy to find later
            'StopFcn', @(~,~) disp('FDIR Timer has been stopped.'));
        start(fdirTimer);
        logToDashboard(uiHandles, sprintf('FDIR background monitoring system started at %d Hz.', FDIR_RATE_HZ));
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
            [trajectory, gnss_track, imu_history] = updateDataHistories(frame_data, trajectory, gnss_track, imu_history);
            processAndAnalyzeFrame(frame_data, toc(last_message_time), uiHandles);
            updateDashboard(uiHandles, frame_data, trajectory, gnss_track, imu_history, carla_outputs);
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

% ... (rest of the functions: setupDashboard, updateDashboard, etc. remain unchanged) ...
% ... (PASTING THE REST OF THE FUNCTIONS FROM CODE 1 FOR COMPLETENESS) ...

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
    p_speed = uipanel(gl,'Title','Speed (km/h)','FontWeight','bold'); p_speed.Layout.Row=1; p_speed.Layout.Column=1;
    uiHandles.speedGauge = uigauge(uigridlayout(p_speed,[1 1]),'ScaleColors',{[0 .8 .4],[.9 .8 0],[1 .2 .2]},'ScaleColorLimits',[0 60;60 100;100 160]);
    p_imu = uipanel(gl,'Title','IMU - Accelerometer','FontWeight','bold'); p_imu.Layout.Row=1; p_imu.Layout.Column=2;
    uiHandles.imuAx = uiaxes(p_imu);
    hold(uiHandles.imuAx,'on'); grid(uiHandles.imuAx,'on');
    uiHandles.accelXPlot = plot(uiHandles.imuAx,NaN(1,historyLength),'r-','LineWidth',1.5,'DisplayName','X');
    uiHandles.accelYPlot = plot(uiHandles.imuAx,NaN(1,historyLength),'g-','LineWidth',1.5,'DisplayName','Y');
    uiHandles.accelZPlot = plot(uiHandles.imuAx,NaN(1,historyLength),'b-','LineWidth',1.5,'DisplayName','Z');
    xlabel(uiHandles.imuAx,'Frames Ago'); ylabel(uiHandles.imuAx,'m/s^2'); legend(uiHandles.imuAx);
    p_gnss = uipanel(gl,'Title','GNSS Trajectory','FontWeight','bold'); p_gnss.Layout.Row=1; p_gnss.Layout.Column=3;
    uiHandles.gnssAx = uiaxes(p_gnss);
    hold(uiHandles.gnssAx,'on'); grid(uiHandles.gnssAx,'on'); axis(uiHandles.gnssAx,'equal');
    xlabel(uiHandles.gnssAx,'Longitude'); ylabel(uiHandles.gnssAx,'Latitude');
    uiHandles.gnssPlot = plot(uiHandles.gnssAx,NaN,NaN,'-m','LineWidth',1.5);
    p_diag = uipanel(gl,'Title','Diagnostics','FontWeight','bold'); p_diag.Layout.Row=1; p_diag.Layout.Column=4;
    g_diag = uigridlayout(p_diag,[1 1]);
    p_diag_metrics = uipanel(g_diag,'Title','');
    g_diag_metrics = uigridlayout(p_diag_metrics,[7,2],'ColumnWidth',{'fit','1x'});
    uilabel(g_diag_metrics,'Text','Frame:', 'FontSize', 10); 
    uiHandles.frameLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold', 'FontSize', 10);
    uilabel(g_diag_metrics,'Text','Net Latency:', 'FontSize', 10); 
    uiHandles.latencyLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold', 'FontSize', 10);
    uilabel(g_diag_metrics,'Text','Mode:', 'FontSize', 10); 
    uiHandles.modeLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold', 'HorizontalAlignment', 'left', 'FontSize', 10);
    uilabel(g_diag_metrics,'Text','Map:', 'FontSize', 10); 
    uiHandles.mapLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold', 'HorizontalAlignment', 'left', 'FontSize', 10);
    uilabel(g_diag_metrics,'Text','Weather:', 'FontSize', 10); 
    uiHandles.weatherLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold', 'HorizontalAlignment', 'left', 'FontSize', 10);
    uilabel(g_diag_metrics,'Text','Sensor Health:', 'FontSize', 10); 
    uiHandles.healthLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold', 'HorizontalAlignment', 'left', 'FontSize', 10);
    uilabel(g_diag_metrics,'Text','Fallback Status:', 'FontSize', 10); 
    uiHandles.fallbackLabel=uilabel(g_diag_metrics,'Text','IDLE','FontWeight','bold','FontColor','g', 'HorizontalAlignment', 'left', 'FontSize', 10);
    p_cameras = uipanel(gl,'Title','Camera Feeds','FontWeight','bold'); p_cameras.Layout.Row=2; p_cameras.Layout.Column=[1 2];
    g_cameras = uigridlayout(p_cameras,[2 3]);
    uiHandles.camLeftAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); title(uiHandles.camLeftAx,'Left');
    uiHandles.camFrontAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); title(uiHandles.camFrontAx,'Front');
    uiHandles.camRightAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); title(uiHandles.camRightAx,'Right');
    uiHandles.camInteriorAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); title(uiHandles.camInteriorAx,'Interior');
    uiHandles.camRearAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); title(uiHandles.camRearAx,'Back');
    p_traj = uipanel(gl,'Title','Vehicle Trajectory (Fused)','FontWeight','bold'); p_traj.Layout.Row=2; p_traj.Layout.Column=3;
    uiHandles.trajAx=uiaxes(p_traj); hold(uiHandles.trajAx,'on'); grid(uiHandles.trajAx,'on'); axis(uiHandles.trajAx,'equal'); xlabel(uiHandles.trajAx,'X (m)'); ylabel(uiHandles.trajAx,'Y (m)');
    uiHandles.trajPlot=plot(uiHandles.trajAx,NaN,NaN,'-c','LineWidth',2,'DisplayName','Path'); 
    uiHandles.currentPosPlot=plot(uiHandles.trajAx,NaN,NaN,'bo','MarkerFaceColor','b','MarkerSize',8,'DisplayName','Current'); 
    legend(uiHandles.trajAx);
    p_lidar = uipanel(gl,'Title','3D LiDAR & Object Detections','FontWeight','bold'); p_lidar.Layout.Row=2; p_lidar.Layout.Column=4;
    uiHandles.lidarAx=uiaxes(p_lidar); 
    hold(uiHandles.lidarAx,'on'); grid(uiHandles.lidarAx,'on'); axis(uiHandles.lidarAx,'equal'); xlabel(uiHandles.lidarAx,'X (m)'); ylabel(uiHandles.lidarAx,'Y (m)'); zlabel(uiHandles.lidarAx,'Z (m)'); view(uiHandles.lidarAx, -45, 30);
    uiHandles.lidarPlot = scatter3(uiHandles.lidarAx, NaN, NaN, NaN, 10, NaN, 'filled', 'DisplayName', 'LiDAR Points');
    uiHandles.obstaclePlot = scatter3(uiHandles.lidarAx, NaN, NaN, NaN, 100, 'bo', 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Obstacles');
    legend(uiHandles.lidarAx);
    p_log = uipanel(gl,'Title','System Log','FontWeight','bold'); p_log.Layout.Row=3; p_log.Layout.Column=[1 4];
    uiHandles.logArea = uitextarea(uigridlayout(p_log,[1 1]), 'Value',{''}, 'Editable','off', 'FontName', 'Monospaced', 'BackgroundColor', [0.1 0.1 0.1], 'FontColor', [0.9 0.9 0.9]);
end

%% =======================================================================
%                      DATA PROCESSING AND UPDATING
% ========================================================================
function updateDashboard(uiHandles, data, trajectory, gnss_track, imu_history, outputs)
    if ~isvalid(uiHandles.fig), return; end
    if isfield(data, 'speed'), uiHandles.speedGauge.Value = data.speed; end
    set(uiHandles.accelXPlot, 'YData', imu_history.x); set(uiHandles.accelYPlot, 'YData', imu_history.y); set(uiHandles.accelZPlot, 'YData', imu_history.z);
    set(uiHandles.gnssPlot, 'XData', gnss_track(:,2), 'YData', gnss_track(:,1));
    set(uiHandles.trajPlot, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
    set(uiHandles.currentPosPlot, 'XData', trajectory(end,1), 'YData', trajectory(end,2));
    if isfield(data, 'frame'), uiHandles.frameLabel.Text = num2str(data.frame); end
    if isfield(outputs, 'network_status'), uiHandles.latencyLabel.Text = sprintf('%.2f ms (%.1f FPS)', outputs.network_status.latency * 1000, outputs.network_status.data_rate); end
    if isfield(outputs, 'sensor_fusion_status') && isfield(outputs.sensor_fusion_status, 'health_score'), uiHandles.healthLabel.Text = sprintf('%.0f %% OK', outputs.sensor_fusion_status.health_score * 100); end
    if isfield(outputs, 'fallback_initiation'), if outputs.fallback_initiation, uiHandles.fallbackLabel.Text = 'TRIGGERED'; uiHandles.fallbackLabel.FontColor = 'r'; else, uiHandles.fallbackLabel.Text = 'IDLE'; uiHandles.fallbackLabel.FontColor = 'g'; end, end
    if isfield(data, 'mode'), uiHandles.modeLabel.Text = data.mode; end
    if isfield(data, 'map'), uiHandles.mapLabel.Text = data.map; end
    if isfield(data, 'weather'), uiHandles.weatherLabel.Text = data.weather; end
    updateCamera(uiHandles.camFrontAx, data, 'image_front'); updateCamera(uiHandles.camRearAx, data, 'image_back'); updateCamera(uiHandles.camLeftAx, data, 'image_left'); updateCamera(uiHandles.camRightAx, data, 'image_right'); updateCamera(uiHandles.camInteriorAx, data, 'image_interior');
    if isfield(data, 'lidar_roof') && ~isempty(data.lidar_roof), try, points = typecast(base64decode(data.lidar_roof), 'single'); if mod(numel(points), 4) == 0, points = reshape(points, 4, [])'; xyz = points(1:20:end, 1:3); set(uiHandles.lidarPlot, 'XData', xyz(:,1), 'YData', xyz(:,2), 'ZData', xyz(:,3), 'CData', xyz(:,3)); end; catch, end; end

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
end

function updateCamera(ax, data, fieldName)
    if isfield(data, fieldName) && ~isempty(data.(fieldName)), try, img_bytes = base64decode(data.(fieldName)); temp_file = [tempname '.jpg']; fid = fopen(temp_file, 'wb'); fwrite(fid, img_bytes, 'uint8'); fclose(fid); img = imread(temp_file); imshow(img, 'Parent', ax); delete(temp_file); catch, end; end
end

function [trajectory, gnss_track, imu_history] = updateDataHistories(data, trajectory, gnss_track, imu_history)
    fused_state = get_safe(data, 'fused_state', struct('x', NaN, 'y', NaN));
    if isstruct(fused_state) && ~isnan(fused_state.x), new_pos = [fused_state.x, fused_state.y]; trajectory = [trajectory(2:end,:); new_pos]; end
    gnss_data = get_safe(data, 'gnss', []);
    if isstruct(gnss_data), new_gnss = [gnss_data.lat, gnss_data.lon]; gnss_track = [gnss_track(2:end,:); new_gnss]; end
    imu_b64 = get_safe(data, 'imu', '');
    if ~isempty(imu_b64), try, imu_vals = typecast(base64decode(imu_b64), 'single'); if numel(imu_vals) >= 3, imu_history.x = [imu_history.x(2:end), imu_vals(1)]; imu_history.y = [imu_history.y(2:end), imu_vals(2)]; imu_history.z = [imu_history.z(2:end), imu_vals(3)]; end; catch, end; end
end

function processAndAnalyzeFrame(frame, latency, uiHandles)
    global carla_outputs; 
    persistent last_call_timer last_yaw last_collisions last_lane_invasions;
    if isempty(last_call_timer)
        time_since_last = 1/20; 
        last_call_timer = tic; 
    else
        time_since_last = toc(last_call_timer);
        last_call_timer = tic;
    end
    data_rate = 1 / max(time_since_last, 0.001); 
    network_status = struct('latency', latency, 'data_rate', data_rate);
    health_score = 0;
    covariance_trace = inf;
    sensor_health_data = get_safe(frame, 'sensor_health', []);
    if isstruct(sensor_health_data)
        all_groups = struct2cell(sensor_health_data); 
        total_ok = 0; total_sensors = 0;
        for i = 1:numel(all_groups), status_list = all_groups{i}; total_sensors = total_sensors + numel(status_list); total_ok = total_ok + sum(strcmp(status_list, 'OK')); end
        if total_sensors > 0, health_score = total_ok / total_sensors; end
    end
    ekf_cov_data = get_safe(frame, 'ekf_covariance', []);
    if iscell(ekf_cov_data)
        try
            rows = cellfun(@cell2mat, ekf_cov_data, 'UniformOutput', false); 
            covariance_matrix = cell2mat(rows);
            covariance_trace = trace(covariance_matrix(1:2, 1:2));
        catch
            covariance_trace = inf;
        end
    end
    fused_state_data = get_safe(frame, 'fused_state', struct('x', NaN, 'y', NaN, 'vx', NaN, 'vy', NaN));
    sensor_fusion_status = struct('fused_state', fused_state_data, 'position_uncertainty', covariance_trace, 'health_score', health_score, 'raw_health', sensor_health_data);
    processed_sensor_data = struct();
    all_fields = fieldnames(frame);
    prefixes_to_copy = {'image_', 'lidar_', 'radar_'}; 
    single_fields_to_copy = {'gnss', 'imu', 'position', 'rotation', 'Detected_Obstacles'};
    for i = 1:numel(all_fields)
        field = all_fields{i};
        if isfield(frame, field)
            copy_this_field = false;
            for j = 1:numel(prefixes_to_copy), if startsWith(field, prefixes_to_copy{j}), copy_this_field = true; break; end, end
            if ~copy_this_field && ismember(field, single_fields_to_copy), copy_this_field = true; end
            if copy_this_field, processed_sensor_data.(field) = frame.(field); end
        end
    end
    default_control = struct('throttle',0,'brake',0,'steer',0,'hand_brake',false,'reverse',false);
    control_data = get_safe(frame, 'control', default_control);
    processed_sensor_data.speed = get_safe(frame, 'speed', 0);
    processed_sensor_data.ultrasonic_front = get_safe(frame, 'ultrasonic_front', inf);
    processed_sensor_data.ultrasonic_back = get_safe(frame, 'ultrasonic_back', inf);
    processed_sensor_data.throttle_input = control_data.throttle;
    processed_sensor_data.brake_input = control_data.brake;
    processed_sensor_data.steering_input = control_data.steer;
    rotation_data = get_safe(frame, 'rotation', struct('yaw', 0));
    yaw_rate = 0;
    if isfield(rotation_data, 'yaw')
        current_yaw = rotation_data.yaw;
        if isempty(last_yaw), last_yaw = current_yaw; end
        yaw_delta = current_yaw - last_yaw;
        if yaw_delta > 180, yaw_delta = yaw_delta - 360; end
        if yaw_delta < -180, yaw_delta = yaw_delta + 360; end
        yaw_rate = yaw_delta / max(time_since_last, 0.001);
        last_yaw = current_yaw;
    end
    processed_sensor_data.yaw_rate = yaw_rate;
    current_collisions = get_safe(frame, 'collisions', 0);
    if isempty(last_collisions), last_collisions = current_collisions; end
    is_collision = (current_collisions > last_collisions);
    last_collisions = current_collisions;
    processed_sensor_data.is_collision_event = is_collision;
    current_lane_invasions = get_safe(frame, 'lane_invasions', 0);
    if isempty(last_lane_invasions), last_lane_invasions = current_lane_invasions; end
    is_lane_invasion = (current_lane_invasions > last_lane_invasions);
    last_lane_invasions = current_lane_invasions;
    processed_sensor_data.is_lane_invasion_event = is_lane_invasion;
    
    processed_sensor_data.Weather_Severity = computeWeatherSeverity(frame);
    processed_sensor_data.Obstacle_Density = computeObstacleDensity(frame);

    driver_attention = computeDriverAttention(frame);
    driver_readiness = computeDriverReadiness(frame);
    threat_data = computeThreatAssessment(frame);
    fallback = false; reason = {};
    if latency > 0.2, fallback = true; reason{end+1} = 'High Latency (>200ms)'; end
    if health_score < 0.75, fallback = true; reason{end+1} = 'Low Sensor Health'; end
    if threat_data.min_front_distance < 10 && threat_data.closing_velocity < -5, fallback = true; reason{end+1} = 'Imminent Collision Risk'; end
    if driver_attention < 0.2, fallback = true; reason{end+1} = 'Low Driver Attention'; end
    if driver_readiness < 0.5, fallback = true; reason{end+1} = 'Low Driver Readiness'; end
    if processed_sensor_data.is_collision_event, fallback = true; reason{end+1} = 'Collision Detected'; end
    if processed_sensor_data.is_lane_invasion_event, fallback = true; reason{end+1} = 'Lane Invasion Detected'; end
    
    if fallback, logToDashboard(uiHandles, sprintf('[FALLBACK] Reasons: %s', strjoin(reason, ', '))); end
    carla_outputs = struct('network_status', network_status, 'sensor_fusion_status', sensor_fusion_status, 'processed_sensor_data', processed_sensor_data, 'fallback_initiation', fallback, 'driver_attention', driver_attention, 'driver_readiness', driver_readiness);
end


%% =======================================================================
%                        NETWORK & UTILITY FUNCTIONS
% ========================================================================
function full_data = processPacket(packet)
    persistent chunkBuffer;
    if isempty(chunkBuffer), chunkBuffer = containers.Map('KeyType', 'double', 'ValueType', 'any'); end
    full_data = [];
    if isfield(packet, 'chunk')
        frameId = packet.frame;
        if ~isKey(chunkBuffer, frameId), chunkBuffer(frameId) = struct('chunks', {cell(1, packet.total_chunks)}, 'received_chunks', 0); end
        chunkData = chunkBuffer(frameId);
        if isempty(chunkData.chunks{packet.chunk + 1}), chunkData.chunks{packet.chunk + 1} = packet.data; chunkData.received_chunks = chunkData.received_chunks + 1; chunkBuffer(frameId) = chunkData; end
        currentChunkData = chunkBuffer(frameId);
        if currentChunkData.received_chunks == packet.total_chunks
            full_json_string = strjoin(currentChunkData.chunks, '');
            try, full_data = jsondecode(full_json_string); catch ME, disp(['JSON decode failed for frame ' num2str(frameId) ': ' ME.message]); full_data = []; end
            remove(chunkBuffer, frameId);
        end
    else, full_data = packet; end
end

function [jsonStr, remaining] = extractJSON(buffer)
    jsonStr = ''; remaining = buffer; startIdx = find(buffer == '{', 1);
    if isempty(startIdx), return; end
    braceCount = 0; endIdx = 0; inString = false; escaped = false;
    for i = startIdx:length(buffer)
        char = buffer(i);
        if ~inString, if char == '{', braceCount = braceCount + 1; elseif char == '}', braceCount = braceCount - 1; if braceCount == 0, endIdx = i; break; end; elseif char == '"', inString = true; end
        else, if ~escaped, if char == '"', inString = false; elseif char == '\', escaped = true; end; else, escaped = false; end; end
    end
    if endIdx > 0, jsonStr = buffer(startIdx:endIdx); remaining = buffer(endIdx+1:end); end
end

function logToDashboard(uiHandles, message)
    if ~isvalid(uiHandles.fig), return; end
    timestamp = datestr(now, 'HH:MM:SS'); newMessage = sprintf('%s - %s', timestamp, message);
    currentLog = uiHandles.logArea.Value;
    if numel(currentLog) > 150, currentLog = currentLog(end-149:end); end
    uiHandles.logArea.Value = [currentLog; {newMessage}];
    scroll(uiHandles.logArea, 'bottom'); drawnow;
end

function decoded = base64decode(str)
    try, import java.util.Base64; decoder = Base64.getDecoder(); decoded = typecast(decoder.decode(uint8(strrep(str, newline, ''))), 'uint8');
    catch, decoded = []; end
end

function threat = computeThreatAssessment(frame)
    threat.min_front_distance = inf;
    threat.closing_velocity = 0;
    radar_data = get_safe(frame, 'radar_front', []);
    if isstruct(radar_data)
        detections = radar_data;
        for i = 1:numel(detections)
            if abs(detections(i).az) < 0.2 
                if detections(i).depth < threat.min_front_distance
                    threat.min_front_distance = detections(i).depth;
                    threat.closing_velocity = detections(i).vel;
                end
            end
        end
    end
end

function score = computeDriverAttention(frame)
    persistent time_of_last_input;
    if isempty(time_of_last_input), time_of_last_input = tic; end
    attention_decay_period = 5.0; 
    score = 1.0; 
    mode = get_safe(frame, 'mode', 'autopilot');
    default_control = struct('steer',0,'throttle',0,'brake',0);
    controls = get_safe(frame, 'control', default_control);
    if strcmp(mode, 'manual')
        if abs(controls.steer) > 0.01 || controls.throttle > 0.01 || controls.brake > 0.01
            time_of_last_input = tic;
            score = 1.0;
        else
            time_since_input = toc(time_of_last_input);
            score = 1.0 - (time_since_input / attention_decay_period);
            score = max(0, min(1, score));
        end
    else
        time_of_last_input = tic;
        score = 1.0;
    end
end

function score = computeDriverReadiness(frame)
    score = 1.0; 
    default_control = struct('throttle',0,'brake',0,'hand_brake',false);
    controls = get_safe(frame, 'control', default_control);
    if controls.throttle > 0.1 && controls.brake > 0.1
        score = 0.1; 
        return;
    end
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
        case 'ClearNoon'
            severity = 0.1;
        case 'CloudyNoon'
            severity = 0.3;
        case 'WetNoon'
            severity = 0.6;
        case 'HardRainNoon'
            severity = 0.9;
        otherwise
            severity = 0.1; % Default to benign
    end
end

function density = computeObstacleDensity(frame)
    density = 0;
    detection_radius = 50; % meters
    
    obstacles = get_safe(frame, 'Detected_Obstacles', []);
    ego_pos = get_safe(frame, 'position', struct('x', 0, 'y', 0, 'z', 0));
    
    if isempty(obstacles), return; end
    
    for i = 1:numel(obstacles)
        obs_pos = obstacles(i).position;
        distance = sqrt((obs_pos.x - ego_pos.x)^2 + (obs_pos.y - ego_pos.y)^2);
        if distance < detection_radius
            density = density + 1;
        end
    end
end
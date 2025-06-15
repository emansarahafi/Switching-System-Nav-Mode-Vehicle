function carla_udp_receiver(port)
% CARLA_UDP_RECEIVER - Main entry point for the CARLA diagnostics dashboard.
% This script sets up a modern UI, receives data from a Python CARLA client
% over UDP, processes it, and continuously updates the dashboard components.

    % --- Configuration ---
    if nargin < 1
        port = 10000;
    end
    HISTORY_LENGTH = 100; % Number of historical data points for plots

    % --- Global Outputs for External Access ---
    global carla_outputs;
    carla_outputs = struct();

    % --- Setup UI and UDP ---
    [uiHandles] = setupDashboard(HISTORY_LENGTH);
    logToDashboard(uiHandles, 'Dashboard initialized. Starting UDP receiver...');
    
    try
        u = udpport("IPV4", "LocalPort", port, "Timeout", 1, "EnablePortSharing", true);
        u.InputBufferSize = 2000000;
        logToDashboard(uiHandles, sprintf('UDP Receiver started on port %d.', port));
    catch ME
        logToDashboard(uiHandles, sprintf('[FATAL] UDP setup failed: %s', ME.message));
        errordlg(sprintf('UDP setup on port %d failed. Is it in use?', port), 'UDP Error');
        return;
    end

    % --- Data Storage and State Initialization ---
    trajectory = nan(HISTORY_LENGTH * 5, 2);
    gnss_track = nan(HISTORY_LENGTH * 5, 2);
    imu_history = struct('x', nan(1, HISTORY_LENGTH), 'y', nan(1, HISTORY_LENGTH), 'z', nan(1, HISTORY_LENGTH));
    
    buffer = '';
    frame_data = struct();
    last_message_time = tic;
    
    % --- Main Loop ---
    logToDashboard(uiHandles, 'Waiting for data from CARLA...');
    while ishandle(uiHandles.fig)
        if uiHandles.isPaused
            pause(0.2);
            continue;
        end

        % Read and process data from UDP
        if u.NumBytesAvailable > 0
            byteData = read(u, u.NumBytesAvailable, "uint8");
            newData = native2unicode(byteData, 'UTF-8');
            buffer = [buffer, newData];
            last_message_time = tic;
            
            while ~isempty(buffer)
                [jsonStr, buffer] = extractJSON(buffer);
                if isempty(jsonStr), break; end
                
                try
                    packet = jsondecode(jsonStr);
                    processed_frame = processPacket(packet); 
                    if ~isempty(processed_frame)
                        frame_data = processed_frame;
                    end
                catch ME
                    logToDashboard(uiHandles, sprintf('[ERROR] JSON processing: %s', ME.message));
                end
            end
        end
        
        % Update data histories and dashboard if new frame arrived
        if ~isempty(frame_data)
            [trajectory, gnss_track, imu_history] = updateDataHistories(frame_data, trajectory, gnss_track, imu_history);
            carla_outputs = processAndAnalyzeFrame(frame_data, toc(last_message_time), uiHandles);
            updateDashboard(uiHandles, frame_data, trajectory, gnss_track, imu_history, carla_outputs);
            frame_data = [];
        end
        
        pause(0.01);
    end
    
    if isvalid(u), delete(u); end
end

%% =======================================================================
%                      UI SETUP AND CALLBACKS
% ========================================================================

function [uiHandles] = setupDashboard(historyLength)
    fig = uifigure('Name', 'CARLA Final Diagnostics Dashboard', 'Position', [50 50, 1600, 950]);
    uiHandles.fig = fig;
    uiHandles.isPaused = false;
    
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

    p_diag = uipanel(gl,'Title','Diagnostics & Controls','FontWeight','bold'); p_diag.Layout.Row=1; p_diag.Layout.Column=4;
    g_diag = uigridlayout(p_diag,[2 1],'RowHeight',{'2x','fit'});
    p_diag_metrics = uipanel(g_diag,'Title',''); p_diag_metrics.Layout.Row=1;
    g_diag_metrics = uigridlayout(p_diag_metrics,[5,2],'ColumnWidth',{'fit','1x'});
    uilabel(g_diag_metrics,'Text','Frame:'); uiHandles.frameLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold');
    uilabel(g_diag_metrics,'Text','Net Latency:'); uiHandles.latencyLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold');
    uilabel(g_diag_metrics,'Text','Sensor Health:'); uiHandles.healthLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold');
    uilabel(g_diag_metrics,'Text','Fallback Status:'); uiHandles.fallbackLabel=uilabel(g_diag_metrics,'Text','IDLE','FontWeight','bold','FontColor','g');
    uilabel(g_diag_metrics,'Text','Environment:'); uiHandles.envLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold');
    p_ctrl = uipanel(g_diag,'Title',''); p_ctrl.Layout.Row=2;
    g_ctrl = uigridlayout(p_ctrl,[1 2]);
    uiHandles.pauseButton=uibutton(g_ctrl,'Text','Pause', 'ButtonPushedFcn',@(s,e)pauseCallback(uiHandles));
    uiHandles.saveButton=uibutton(g_ctrl,'Text','Save Snapshot', 'ButtonPushedFcn',@(s,e)saveCallback(uiHandles));

    p_cameras = uipanel(gl,'Title','Camera Feeds','FontWeight','bold'); p_cameras.Layout.Row=2; p_cameras.Layout.Column=[1 2];
    g_cameras = uigridlayout(p_cameras,[2 3]);
    uiHandles.camLeftAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); title(uiHandles.camLeftAx,'Left');
    uiHandles.camFrontAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); title(uiHandles.camFrontAx,'Front');
    uiHandles.camRightAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); title(uiHandles.camRightAx,'Right');
    uiHandles.camInteriorAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); title(uiHandles.camInteriorAx,'Interior');
    uiHandles.camRearAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); title(uiHandles.camRearAx,'Back');

    p_traj = uipanel(gl,'Title','Vehicle Trajectory (Fused)','FontWeight','bold'); p_traj.Layout.Row=2; p_traj.Layout.Column=3;
    uiHandles.trajAx=uiaxes(p_traj); hold(uiHandles.trajAx,'on'); grid(uiHandles.trajAx,'on'); axis(uiHandles.trajAx,'equal'); xlabel(uiHandles.trajAx,'X (m)'); ylabel(uiHandles.trajAx,'Y (m)');
    uiHandles.trajPlot=plot(uiHandles.trajAx,NaN,NaN,'-c','LineWidth',2); uiHandles.currentPosPlot=plot(uiHandles.trajAx,NaN,NaN,'bo','MarkerFaceColor','b','MarkerSize',8); legend(uiHandles.trajAx,'Path','Current','Location','northwest');

    p_lidar = uipanel(gl,'Title','3D LiDAR Point Cloud','FontWeight','bold'); p_lidar.Layout.Row=2; p_lidar.Layout.Column=4;
    uiHandles.lidarAx=uiaxes(p_lidar); hold(uiHandles.lidarAx,'on'); grid(uiHandles.lidarAx,'on'); axis(uiHandles.lidarAx,'equal'); xlabel(uiHandles.lidarAx,'X (m)'); ylabel(uiHandles.lidarAx,'Y (m)'); zlabel(uiHandles.lidarAx,'Z (m)'); view(uiHandles.lidarAx, -45, 30);
    uiHandles.lidarPlot = scatter3(uiHandles.lidarAx, NaN, NaN, NaN, 10, NaN, 'filled');

    p_log = uipanel(gl,'Title','System Log','FontWeight','bold'); p_log.Layout.Row=3; p_log.Layout.Column=[1 4];
    uiHandles.logArea = uitextarea(uigridlayout(p_log,[1 1]), 'Value',{''}, 'Editable','off', 'FontName', 'Monospaced', 'BackgroundColor', [0.1 0.1 0.1], 'FontColor', [0.9 0.9 0.9]);
end

function pauseCallback(uiHandles), uiHandles.isPaused = ~uiHandles.isPaused; if uiHandles.isPaused, uiHandles.pauseButton.Text = 'Resume'; else, uiHandles.pauseButton.Text = 'Pause'; end, end
function saveCallback(uiHandles), filename = ['CARLA_Snapshot_' datestr(now, 'yyyymmdd_HHMMSS') '.png']; exportgraphics(uiHandles.fig, filename); logToDashboard(uiHandles, sprintf('Dashboard saved to %s', filename)); end

%% =======================================================================
%                      DATA PROCESSING AND UPDATING
% ========================================================================

function updateDashboard(uiHandles, data, trajectory, gnss_track, imu_history, outputs)
    if isfield(data, 'speed'), uiHandles.speedGauge.Value = data.speed; end
    set(uiHandles.accelXPlot, 'YData', imu_history.x); set(uiHandles.accelYPlot, 'YData', imu_history.y); set(uiHandles.accelZPlot, 'YData', imu_history.z);
    set(uiHandles.gnssPlot, 'XData', gnss_track(:,2), 'YData', gnss_track(:,1));
    % --- This is the CORRECTED line ---
    set(uiHandles.trajPlot, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
    set(uiHandles.currentPosPlot, 'XData', trajectory(end,1), 'YData', trajectory(end,2));
    if isfield(data, 'frame'), uiHandles.frameLabel.Text = num2str(data.frame); end
    uiHandles.latencyLabel.Text = sprintf('%.2f ms', outputs.network_status.latency * 1000);
    
    health_struct = outputs.sensor_fusion_status.health;
    if isstruct(health_struct)
        all_groups = struct2cell(health_struct); total_ok = 0; total_sensors = 0;
        for i = 1:numel(all_groups), status_list = all_groups{i}; total_sensors = total_sensors + numel(status_list); total_ok = total_ok + sum(strcmp(status_list, 'OK')); end
        uiHandles.healthLabel.Text = sprintf('%d / %d OK', total_ok, total_sensors);
    end
    
    if outputs.fallback_initiation, uiHandles.fallbackLabel.Text = 'TRIGGERED'; uiHandles.fallbackLabel.FontColor = 'r'; else, uiHandles.fallbackLabel.Text = 'IDLE'; uiHandles.fallbackLabel.FontColor = 'g'; end
    if isfield(outputs.processed_sensor_data.environment, 'lighting'), uiHandles.envLabel.Text = sprintf('%s, %s', outputs.processed_sensor_data.environment.lighting, outputs.processed_sensor_data.environment.weather); end
    
    updateCamera(uiHandles.camFrontAx, data, 'image_front'); updateCamera(uiHandles.camRearAx, data, 'image_back'); updateCamera(uiHandles.camLeftAx, data, 'image_left'); updateCamera(uiHandles.camRightAx, data, 'image_right'); updateCamera(uiHandles.camInteriorAx, data, 'image_interior');
    if isfield(data, 'lidar_roof') && ~isempty(data.lidar_roof), try, points = typecast(base64decode(data.lidar_roof), 'single'); if mod(numel(points), 4) == 0, points = reshape(points, 4, [])'; xyz = points(1:20:end, 1:3); set(uiHandles.lidarPlot, 'XData', xyz(:,1), 'YData', xyz(:,2), 'ZData', xyz(:,3), 'CData', xyz(:,3)); end; catch, end; end
end

function updateCamera(ax, data, fieldName)
    if isfield(data, fieldName) && ~isempty(data.(fieldName)), try, img_bytes = base64decode(data.(fieldName)); temp_file = [tempname '.jpg']; fid = fopen(temp_file, 'wb'); fwrite(fid, img_bytes, 'uint8'); fclose(fid); img = imread(temp_file); imshow(img, 'Parent', ax); delete(temp_file); catch, end; end
end

function [trajectory, gnss_track, imu_history] = updateDataHistories(data, trajectory, gnss_track, imu_history)
    if isfield(data, 'fused_state') && isstruct(data.fused_state) && ~isempty(data.fused_state), new_pos = [data.fused_state.x, data.fused_state.y]; trajectory = [trajectory(2:end,:); new_pos]; end
    if isfield(data, 'gnss'), new_gnss = [data.gnss.lat, data.gnss.lon]; gnss_track = [gnss_track(2:end,:); new_gnss]; end
    if isfield(data, 'imu'), try, imu_vals = typecast(base64decode(data.imu), 'single'); if numel(imu_vals) >= 3, imu_history.x = [imu_history.x(2:end), imu_vals(1)]; imu_history.y = [imu_history.y(2:end), imu_vals(2)]; imu_history.z = [imu_history.z(2:end), imu_vals(3)]; end; catch, end; end
end

function outputs = processAndAnalyzeFrame(frame, latency, uiHandles)
    outputs = struct('network_status',[], 'sensor_fusion_status',[], 'processed_sensor_data',[], 'fallback_initiation',false);
    outputs.network_status = struct('latency', latency);
    health = struct(); if isfield(frame, 'sensor_health'), health = frame.sensor_health; end
    covariance = []; if isfield(frame, 'ekf_covariance') && iscell(frame.ekf_covariance), try, rows = cellfun(@cell2mat, frame.ekf_covariance, 'UniformOutput', false); covariance = cell2mat(rows); catch ME, logToDashboard(uiHandles, sprintf('[WARN] Covariance matrix invalid: %s', ME.message)); end; end
    env = struct(); if isfield(frame, 'environment'), env = detect_environment(frame.environment); end
    
    outputs.sensor_fusion_status = struct('fused_state', [], 'covariance', covariance, 'health', health);
    if isfield(frame, 'fused_state'), outputs.sensor_fusion_status.fused_state = frame.fused_state; end
    outputs.processed_sensor_data = struct('environment', env);
    
    fallback = false; reason = {};
    if latency > 2, fallback = true; reason{end+1} = 'High Latency'; end
    if isstruct(health), all_groups = struct2cell(health); total_fail = 0; total_sens = 0; for i=1:numel(all_groups), status_list=all_groups{i}; total_sens=total_sens+numel(status_list); total_fail=total_fail+sum(~strcmp(status_list,'OK')); end; if total_fail > total_sens/2, fallback=true; reason{end+1} = 'Critical Sensor Failure'; end; end
    if ~isempty(covariance) && any(diag(covariance(1:2,1:2)) > 10), fallback = true; reason{end+1} = 'High Position Uncertainty'; end
    outputs.fallback_initiation = fallback;
    if fallback, logToDashboard(uiHandles, sprintf('[FALLBACK] Reasons: %s', strjoin(reason, ', '))); end
    
    global carla_outputs; carla_outputs = outputs;
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
        if chunkData.received_chunks == packet.total_chunks, full_json = strjoin(chunkData.chunks, ''); full_data = jsondecode(full_json); remove(chunkBuffer, frameId); end
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

function env_cond = detect_environment(env_struct)
    if ~isfield(env_struct, 'weather'), env_cond = struct('lighting', 'unknown', 'weather', 'unknown'); return; end
    weather_data = env_struct.weather;
    if isfield(weather_data, 'sun_altitude_angle'), if weather_data.sun_altitude_angle > 0, lighting_cond = 'day'; else, lighting_cond = 'night'; end; else, lighting_cond = 'unknown'; end
    if isfield(weather_data, 'precipitation') && weather_data.precipitation > 70, weather_cond = 'heavy rain';
    elseif isfield(weather_data, 'precipitation') && weather_data.precipitation > 30, weather_cond = 'rain';
    else, weather_cond = 'clear'; end
    env_cond = struct('lighting', lighting_cond, 'weather', weather_cond);
end
function carla_udp_receiver(port)
    % Enhanced CARLA UDP receiver with Sensor Processing Block
    if nargin < 1
        port = 10000;
    end
    
    fprintf('[STATUS] Starting CARLA receiver on port %d...\n', port);
    
    % UDP setup with timeout handling
    try
        u = udpport("IPV4", "LocalHost", "127.0.0.1", "LocalPort", port, ...
                    "Timeout", 10, "EnablePortSharing", true);
        u.InputBufferSize = 1000000;
    catch ME
        fprintf('[ERROR] UDP setup failed: %s\n', ME.message);
        fprintf('>> Check port availability with: netstat -an | findstr :%d\n', port);
        fprintf('>> Try different port number if necessary\n');
        return;
    end
    
    % Create main figure with diagnostics panel
    fig = figure('Name', 'CARLA Diagnostics', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 1400, 900], 'DeleteFcn', @(src,event)close_receiver(u));
    
    % Create diagnostic console
    diag_panel = uipanel(fig, 'Title', 'Diagnostics', 'Position', [0.01 0.01 0.98 0.20]);
    diag_text = uicontrol(diag_panel, 'Style', 'edit', 'Max', 10, 'Min', 0, ...
                          'HorizontalAlignment', 'left', ...
                          'Position', [10, 10, 1350, 140], 'String', 'Initializing...', ...
                          'FontName', 'Consolas', 'FontSize', 9);
    
    % Create axes for trajectory plot
    ax_traj = subplot(3,3,1, 'Parent', fig);
    hold(ax_traj, 'on');
    grid(ax_traj, 'on');
    xlabel(ax_traj, 'X Position (m)');
    ylabel(ax_traj, 'Y Position (m)');
    title(ax_traj, 'Vehicle Trajectory');
    
    % Create axes for camera images
    ax_front = subplot(3,3,2, 'Parent', fig);
    title(ax_front, 'Front Camera');
    axis(ax_front, 'off');
    
    ax_back = subplot(3,3,3, 'Parent', fig);
    title(ax_back, 'Back Camera');
    axis(ax_back, 'off');
    
    ax_left = subplot(3,3,4, 'Parent', fig);
    title(ax_left, 'Left Camera');
    axis(ax_left, 'off');
    
    ax_right = subplot(3,3,5, 'Parent', fig);
    title(ax_right, 'Right Camera');
    axis(ax_right, 'off');
    
    % Create axes for LIDAR point cloud
    ax_lidar = subplot(3,3,6, 'Parent', fig);
    title(ax_lidar, 'LIDAR Point Cloud');
    axis(ax_lidar, 'equal');
    grid(ax_lidar, 'on');
    xlabel(ax_lidar, 'X (m)');
    ylabel(ax_lidar, 'Y (m)');
    zlabel(ax_lidar, 'Z (m)');
    
    % Create axes for sensor data plots
    ax_imu = subplot(3,3,7, 'Parent', fig);
    title(ax_imu, 'IMU Data');
    xlabel(ax_imu, 'Time');
    ylabel(ax_imu, 'Acceleration (m/s²)');
    grid(ax_imu, 'on');
    
    ax_gnss = subplot(3,3,8, 'Parent', fig);
    title(ax_gnss, 'GNSS Track');
    xlabel(ax_gnss, 'Longitude');
    ylabel(ax_gnss, 'Latitude');
    grid(ax_gnss, 'on');
    
    ax_speed = subplot(3,3,9, 'Parent', fig);
    title(ax_speed, 'Vehicle Speed');
    xlabel(ax_speed, 'Time (s)');
    ylabel(ax_speed, 'Speed (km/h)');
    grid(ax_speed, 'on');
    
    % Initialize data storage
    trajectory = [];
    gnss_track = [];
    imu_history = [];
    speed_history = [];
    time_history = [];
    last_plot_update = tic;
    last_diag_update = tic;
    buffer = '';
    frame_data = struct();
    connection_active = false;
    bytes_received = 0;
    messages_received = 0;
    last_message_time = tic;
    start_time = tic;
    
    % Data buffers for chunked transmission
    chunkBuffer = containers.Map('KeyType', 'double', 'ValueType', 'any');
    
    % Create data directories
    data_dir = 'matlab_data';
    frames_dir = fullfile(data_dir, 'frames');
    if ~exist(data_dir, 'dir')
        mkdir(data_dir);
        log_diag(diag_text, sprintf('Created data directory: %s', data_dir));
    end
    if ~exist(frames_dir, 'dir')
        mkdir(frames_dir);
        log_diag(diag_text, sprintf('Created frames directory: %s', frames_dir));
    end
    
    % Create CSV log file
    log_file = fullfile(data_dir, 'vehicle_data.csv');
    log_fid = fopen(log_file, 'w');
    if log_fid == -1
        log_diag(diag_text, sprintf('[ERROR] Failed to create log file: %s', log_file));
    else
        fprintf(log_fid, 'timestamp,frame,speed,x,y,z,yaw,pitch,roll,throttle,steer,brake,gnss_lat,gnss_lon,gnss_alt\n');
    end
    
    % Initialize Kalman Filter state
    ekf_state = zeros(6,1); % [x; y; z; vx; vy; vz]
    ekf_covariance = eye(6)*10;
    
    % Main processing loop
    try
        log_diag(diag_text, 'Waiting for CARLA data...');
        log_diag(diag_text, sprintf('Listening on 127.0.0.1:%d', port));
        log_diag(diag_text, '>> Check CARLA client configuration');
        log_diag(diag_text, '>> Verify matching port in Python script');
        
        while ishandle(fig)
            % Check for data
            if u.NumBytesAvailable > 0
                % Read as bytes
                byteData = read(u, u.NumBytesAvailable, "uint8");
                bytes_received = bytes_received + numel(byteData);
                
                % Convert to UTF-8 string
                try
                    newData = native2unicode(byteData, 'UTF-8');
                catch
                    newData = char(byteData);  % Fallback for encoding issues
                end
                
                buffer = [buffer, newData];
                connection_active = true;
                last_message_time = tic;
                
                % Process complete JSON messages
                while ~isempty(buffer)
                    [jsonStr, buffer] = extractJSON(buffer);
                    
                    if isempty(jsonStr)
                        break;
                    end
                    
                    try
                        packet = jsondecode(jsonStr);
                        messages_received = messages_received + 1;
                        
                        % Debug: Log the structure of received data
                        if messages_received <= 5  % Only log first few messages
                            log_diag(diag_text, sprintf('[DEBUG] Packet fields: %s', strjoin(fieldnames(packet), ', ')));
                            if length(jsonStr) <= 200
                                log_diag(diag_text, sprintf('[DEBUG] Raw JSON: %s', jsonStr));
                            else
                                log_diag(diag_text, sprintf('[DEBUG] Raw JSON preview: %s...', jsonStr(1:200)));
                            end
                        end
                        
                        % Handle chunked data
                        if isfield(packet, 'chunk')
                            % FIXED: Enhanced frame identifier handling
                            if isfield(packet, 'frame')  % Primary field from Python
                                frameId = packet.frame;
                            elseif isfield(packet, 'frame_id')  % Alternative field names
                                frameId = packet.frame_id;
                            elseif isfield(packet, 'frameId')
                                frameId = packet.frameId;
                            else
                                % Diagnostic log with available fields
                                availableFields = fieldnames(packet);
                                log_diag(diag_text, sprintf(...
                                    '[ERROR] Chunked packet missing frame identifier. Available fields: %s', ...
                                    strjoin(availableFields, ', ')...
                                ));
                                continue;
                            end
                            
                            % Initialize buffer for new frame
                            if ~isKey(chunkBuffer, frameId)
                                chunkBuffer(frameId) = struct(...
                                    'chunks', {cell(1, packet.total_chunks)}, ...
                                    'total_chunks', packet.total_chunks, ...
                                    'received_chunks', 0);
                            end
                            
                            % Store chunk (MATLAB uses 1-based indexing)
                            chunkData = chunkBuffer(frameId);
                            if isempty(chunkData.chunks{packet.chunk + 1})
                                chunkData.chunks{packet.chunk + 1} = packet.data;
                                chunkData.received_chunks = chunkData.received_chunks + 1;
                                chunkBuffer(frameId) = chunkData;
                            end
                            
                            % Check if all chunks received
                            if chunkData.received_chunks == packet.total_chunks
                                % Reassemble data
                                fullData = strjoin(chunkData.chunks, '');
                                try
                                    reconstructedData = jsondecode(fullData);
                                    processCompleteFrame(reconstructedData, log_fid, diag_text);
                                    frame_data = reconstructedData;
                                catch ME
                                    log_diag(diag_text, sprintf('[ERROR] Chunk assembly failed: %s', ME.message));
                                end
                                remove(chunkBuffer, frameId);
                            end
                        else
                            % Process complete frame directly - handle different field name variations
                            processCompleteFrame(packet, log_fid, diag_text);
                            frame_data = packet;
                        end
                        
                    catch ME
                        log_diag(diag_text, sprintf('[ERROR] JSON processing: %s', ME.message));
                        if length(jsonStr) > 100
                            log_diag(diag_text, sprintf('Message preview: %s...', jsonStr(1:100)));
                        else
                            log_diag(diag_text, sprintf('Message: %s', jsonStr));
                        end
                    end
                end
            end
            
            % Update data histories for plotting
            if ~isempty(frame_data)
                current_time = toc(start_time);
                
                % Update trajectory (handle different field name patterns)
                position_data = [];
                if isfield(frame_data, 'position')
                    position_data = frame_data.position;
                elseif isfield(frame_data, 'location')
                    position_data = frame_data.location;
                elseif isfield(frame_data, 'transform') && isfield(frame_data.transform, 'location')
                    position_data = frame_data.transform.location;
                end
                
                if ~isempty(position_data)
                    trajectory = [trajectory; [position_data.x, position_data.y]];
                    if size(trajectory, 1) > 1000  % Limit trajectory points
                        trajectory = trajectory(end-999:end, :);
                    end
                end
                
                % Update GNSS track
                if isfield(frame_data, 'gnss')
                    gnss_track = [gnss_track; [frame_data.gnss.lon, frame_data.gnss.lat]];
                    if size(gnss_track, 1) > 1000
                        gnss_track = gnss_track(end-999:end, :);
                    end
                end
                
                % Update speed history (handle different speed field patterns)
                speed_val = [];
                if isfield(frame_data, 'speed')
                    speed_val = frame_data.speed;
                elseif isfield(frame_data, 'velocity')
                    if isstruct(frame_data.velocity)
                        speed_val = sqrt(frame_data.velocity.x^2 + frame_data.velocity.y^2 + frame_data.velocity.z^2) * 3.6; % m/s to km/h
                    else
                        speed_val = frame_data.velocity;
                    end
                end
                
                if ~isempty(speed_val)
                    speed_history = [speed_history; speed_val];
                    time_history = [time_history; current_time];
                    if length(speed_history) > 1000
                        speed_history = speed_history(end-999:end);
                        time_history = time_history(end-999:end);
                    end
                end
            end
            
            % ==================== SENSOR PROCESSING BLOCK ====================
            if ~isempty(frame_data)
                [fused_data, health_status, attention, env_cond, ekf_state, ekf_covariance] = ...
                    sensorProcessingBlock(frame_data, ekf_state, ekf_covariance, diag_text);
                
                % Store processed outputs in frame_data
                frame_data.fused = fused_data;
                frame_data.health = health_status;
                frame_data.driver_attention = attention;
                frame_data.environment = env_cond;
                
                % Log sensor health status
                health_msg = 'Sensor Health: ';
                sensor_names = fieldnames(health_status);
                for i = 1:length(sensor_names)
                    health_msg = [health_msg sprintf('%s: %s, ', sensor_names{i}, health_status.(sensor_names{i}))];
                end
                log_diag(diag_text, health_msg(1:end-2));
            end
            % ================================================================
            
            % Update diagnostics panel
            if toc(last_diag_update) > 1.0
                status_msg = sprintf('PORT STATUS: %d\n', port);
                status_msg = [status_msg sprintf('Bytes received: %d\n', bytes_received)];
                status_msg = [status_msg sprintf('Messages received: %d\n', messages_received)];
                status_msg = [status_msg sprintf('Buffered chunks: %d\n', chunkBuffer.Count)];
                status_msg = [status_msg sprintf('Frames saved: %d\n', messages_received)];
                
                if connection_active
                    if toc(last_message_time) > 5
                        status_msg = [status_msg 'CONNECTION: ACTIVE (NO RECENT DATA)\n'];
                        status_msg = [status_msg '>> Check CARLA client is running\n'];
                        status_msg = [status_msg '>> Verify data streaming in CARLA\n'];
                    else
                        status_msg = [status_msg 'CONNECTION: ACTIVE\n'];
                    end
                else
                    status_msg = [status_msg 'CONNECTION: NO DATA RECEIVED\n'];
                    status_msg = [status_msg '>> Verify Python client configuration\n'];
                    status_msg = [status_msg '>> Check firewall settings\n'];
                end
                
                if isfield(frame_data, 'frame') || isfield(frame_data, 'frame_id') || isfield(frame_data, 'frameId')
                    frame_id = 0;
                    if isfield(frame_data, 'frame')
                        frame_id = frame_data.frame;
                    elseif isfield(frame_data, 'frame_id')
                        frame_id = frame_data.frame_id;
                    elseif isfield(frame_data, 'frameId')
                        frame_id = frame_data.frameId;
                    end
                    status_msg = [status_msg sprintf('Last frame: %d\n', frame_id)];
                end

                % Inside the diagnostic panel update section (where status_msg is built)
                if isfield(frame_data, 'environment')
                    status_msg = [status_msg sprintf('Environment:\n')];
                    status_msg = [status_msg sprintf('  Map: %s\n', frame_data.environment.map)];
                    status_msg = [status_msg sprintf('  Weather: %s\n', frame_data.environment.weather)];
                    status_msg = [status_msg sprintf('  Layer: %s\n', frame_data.environment.layer)];
                end
                
                % Display attention level if available
                if isfield(frame_data, 'driver_attention')
                    status_msg = [status_msg sprintf('Driver Attention: %.2f\n', frame_data.driver_attention)];
                end
                
                set(diag_text, 'String', status_msg);
                last_diag_update = tic;
            end
            
            % Update plots periodically
            if toc(last_plot_update) > 0.5
                try
                    % Update trajectory plot
                    if ~isempty(trajectory)
                        cla(ax_traj);
                        plot(ax_traj, trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 1.5);
                        plot(ax_traj, trajectory(end,1), trajectory(end,2), 'ro', ...
                             'MarkerSize', 8, 'MarkerFaceColor', 'r');
                        xlabel(ax_traj, 'X Position (m)');
                        ylabel(ax_traj, 'Y Position (m)');
                        title(ax_traj, 'Vehicle Trajectory');
                        grid(ax_traj, 'on');
                    end
                    
                    % Update GNSS track
                    if ~isempty(gnss_track)
                        cla(ax_gnss);
                        plot(ax_gnss, gnss_track(:,1), gnss_track(:,2), 'g-', 'LineWidth', 1.5);
                        plot(ax_gnss, gnss_track(end,1), gnss_track(end,2), 'go', ...
                             'MarkerSize', 8, 'MarkerFaceColor', 'g');
                        xlabel(ax_gnss, 'Longitude');
                        ylabel(ax_gnss, 'Latitude');
                        title(ax_gnss, 'GNSS Track');
                        grid(ax_gnss, 'on');
                    end
                    
                    % Update speed plot
                    if ~isempty(speed_history)
                        cla(ax_speed);
                        plot(ax_speed, time_history, speed_history, 'r-', 'LineWidth', 1.5);
                        xlabel(ax_speed, 'Time (s)');
                        ylabel(ax_speed, 'Speed (km/h)');
                        title(ax_speed, 'Vehicle Speed');
                        grid(ax_speed, 'on');
                    end
                    
                    drawnow limitrate;
                catch
                    % Silently handle plot errors
                end
                last_plot_update = tic;
            end
            
            % Update images and sensor data
            if ~isempty(frame_data)
                updateVisualizations(frame_data, ax_front, ax_back, ax_left, ax_right, ax_lidar, ax_imu, diag_text);
            end
            
            % Check for timeout
            if connection_active && toc(last_message_time) > 30
                log_diag(diag_text, '[WARNING] No data for 30 seconds. Connection may be lost.');
                connection_active = false;
            end
            
            pause(0.01);
        end
    catch ME
        log_diag(diag_text, sprintf('FATAL ERROR: %s', ME.message));
        for k = 1:length(ME.stack)
            log_diag(diag_text, sprintf('Line %d in %s', ME.stack(k).line, ME.stack(k).name));
        end
    end
    
    % Cleanup
    close_receiver(u, log_fid);
    log_diag(diag_text, 'Receiver shutdown complete');
end

%% ==================== SENSOR PROCESSING BLOCK ====================
function [fused, health, attention, env_cond, new_state, new_cov] = ...
         sensorProcessingBlock(frame, state, cov, diag_text)
    
    % Initialize with CARLA environment data if available
    env_cond = struct('map', '', 'weather', '', 'layer', '', 'lighting', 'day');
    if isfield(frame, 'environment')
        env_cond.map = frame.environment.map;
        env_cond.weather = frame.environment.weather;
        env_cond.layer = frame.environment.layer;
    end

    % Use front camera for lighting and fallback weather
    if isfield(frame, 'image_front') && ~isempty(frame.image_front)
        img_env = detect_environment(frame.image_front);
        env_cond.lighting = img_env.lighting;
        
        % Fallback for weather if not provided by CARLA
        if ~isfield(frame, 'environment') || isempty(env_cond.weather)
            env_cond.weather = img_env.weather;
        end
    end
    
    % Initialize outputs
    fused = struct();
    health = struct();
    attention = 1.0; % Default: fully attentive
    env_cond = struct('lighting', 'day', 'weather', 'clear');
    
    % 1. Sensor health monitoring with field mapping
    sensor_map = {
        'lidar',        'lidar';
        'imu',          'imu';
        'gnss',         'gnss';
        'camera_front', 'image_front';
        'camera_rear',  'image_back';
        'camera_left',  'image_left';
        'camera_right', 'image_right'
    };
    
    for i = 1:size(sensor_map, 1)
        display_name = sensor_map{i,1};
        field_name = sensor_map{i,2};
        
        if isfield(frame, field_name)
            health.(display_name) = monitor_sensor_health(frame.(field_name), display_name);
        else
            health.(display_name) = 'missing';
        end
    end
    
    % 2. Data synchronization and timestamping
    synced_data = synchronize_sensor_data(frame);
    
    % 3. Noise reduction
    filtered = apply_noise_reduction(synced_data);
    
    % 4. Sensor fusion with Kalman Filter
    [fused, new_state, new_cov] = kalman_sensor_fusion(filtered, state, cov);
    
    % 5. Environmental conditions detection (use front camera)
    if isfield(frame, 'image_front') && ~isempty(frame.image_front)
        env_cond = detect_environment(frame.image_front);
    end
    
    % 6. Driver attention estimation (if available)
    if isfield(frame, 'camera_interior') && ~isempty(frame.camera_interior)
        attention = estimate_attention(frame.camera_interior);
    end
end

%% ==================== UPDATED HEALTH MONITORING ====================
function health = monitor_sensor_health(data, sensor_type)
    % Check sensor data validity
    if isempty(data)
        health = 'missing';
        return;
    end
    
    % Extract base sensor type (remove camera position suffix)
    base_type = regexprep(sensor_type, '^(camera|image)_.*', '$1');
    
    switch base_type
        case 'lidar'
            try
                % Check point cloud density
                point_count = numel(typecast(base64decode(data), 'single'))/4;
                if point_count > 1000
                    health = 'ok';
                else
                    health = 'low_density';
                end
            catch
                health = 'decode_failed';
            end
            
        case 'imu'
            try
                % Check data range validity
                imu_vals = typecast(base64decode(data), 'single');
                if any(abs(imu_vals(1:3)) > 20) % Acceleration check
                    health = 'out_of_range';
                else
                    health = 'ok';
                end
            catch
                health = 'decode_failed';
            end
            
        case 'gnss'
            try
                % Handle both struct and numeric formats
                if isstruct(data)
                    % Structure format
                    lat = data.lat;
                    lon = data.lon;
                elseif isnumeric(data) && numel(data) >= 2
                    % Array format [lat, lon]
                    lat = data(1);
                    lon = data(2);
                else
                    health = 'invalid_format';
                    return;
                end
                
                % Validate coordinates
                if abs(lat) > 90 || abs(lon) > 180
                    health = 'invalid';
                else
                    health = 'ok';
                end
            catch
                health = 'invalid_structure';
            end
            
        case 'camera'
            try
                % Check image data integrity
                img = base64decode(data);
                if numel(img) > 1000
                    health = 'ok';
                else
                    health = 'corrupted';
                end
            catch
                health = 'decode_failed';
            end
            
        otherwise
            health = 'unknown_type';
    end
end

%% ==================== HELPER FUNCTIONS ====================
function synced = synchronize_sensor_data(frame)
    % Simple synchronization using frame timestamp
    synced = struct();
    if isfield(frame, 'timestamp')
        synced.timestamp = frame.timestamp;
    else
        synced.timestamp = now();
    end
    
    % IMU data
    if isfield(frame, 'imu')
        synced.imu = frame.imu;
    end
    
    % GPS data
    if isfield(frame, 'gnss')
        synced.gnss = [frame.gnss.lat, frame.gnss.lon, frame.gnss.alt];
    end
    
    % Vehicle dynamics
    if isfield(frame, 'speed')
        synced.speed = frame.speed;
    end
    if isfield(frame, 'position')
        synced.position = [frame.position.x, frame.position.y, frame.position.z];
    end
end

function filtered = apply_noise_reduction(data)
    % Apply noise reduction techniques
    filtered = data;
    
    % Low-pass filter for IMU
    if isfield(data, 'imu')
        persistent imu_filter_state;
        if isempty(imu_filter_state)
            imu_filter_state = zeros(6,1);
        end
        
        try
            imu_vals = typecast(base64decode(data.imu), 'single');
            filtered_vals = 0.8*imu_filter_state + 0.2*imu_vals;
            imu_filter_state = filtered_vals;
            filtered.imu = filtered_vals;
        catch
            % Keep original if decoding fails
        end
    end
    
    % Moving average for GNSS
    if isfield(data, 'gnss')
        persistent gnss_history;
        if isempty(gnss_history)
            gnss_history = repmat(data.gnss, 5, 1);
        end
        
        gnss_history = [data.gnss; gnss_history(1:end-1,:)];
        filtered.gnss = mean(gnss_history, 1);
    end
end

function [fused, new_state, new_cov] = kalman_sensor_fusion(data, state, cov)
    % Simplified Kalman Filter for sensor fusion
    dt = 0.1; % Time step (adjust based on actual timing)
    
    % State transition matrix
    F = [1 0 0 dt 0 0;
         0 1 0 0 dt 0;
         0 0 1 0 0 dt;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1];
     
    % Process noise
    Q = diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5]);
    
    % Prediction step
    pred_state = F * state;
    pred_cov = F * cov * F' + Q;
    
    % Measurement update
    if isfield(data, 'gnss') && isfield(data, 'speed')
        % Measurement vector [x, y, z, vx, vy, vz]
        z = [data.gnss(1), data.gnss(2), data.gnss(3), ...
             data.speed*cos(state(4)), data.speed*sin(state(5)), 0]';
        
        % Measurement matrix
        H = eye(6);
        
        % Measurement noise
        R = diag([0.5, 0.5, 1, 0.2, 0.2, 0.1]);
        
        % Kalman gain
        K = pred_cov * H' / (H * pred_cov * H' + R);
        
        % Update state
        new_state = pred_state + K * (z - H * pred_state);
        new_cov = (eye(6) - K * H) * pred_cov;
    else
        new_state = pred_state;
        new_cov = pred_cov;
    end
    
    % Prepare fused output
    fused.position = new_state(1:3)';
    fused.velocity = new_state(4:6)';
    fused.orientation = []; % Placeholder for actual orientation fusion
end

function env_cond = detect_environment(img_data)
    % Detect environmental conditions from image data
    env_cond = struct('lighting', 'unknown', 'weather', 'unknown');
    
    try
        % Decode base64 image
        img_bytes = base64decode(img_data);
        
        % Create temporary file
        temp_file = [tempname '.jpg'];
        fid = fopen(temp_file, 'wb');
        if fid == -1
            return;
        end
        fwrite(fid, img_bytes, 'uint8');
        fclose(fid);
        
        % Read and process image
        img = imread(temp_file);
        delete(temp_file);
        
        % ==================== LIGHTING DETECTION ====================
        % Convert to HSV and analyze brightness
        hsv = rgb2hsv(img);
        value_channel = hsv(:, :, 3);
        
        % Calculate brightness metrics
        avg_brightness = mean(value_channel(:));
        brightness_std = std(double(value_channel(:)));
        
        % Classify lighting conditions
        if avg_brightness > 0.7
            env_cond.lighting = 'day';
        elseif avg_brightness > 0.4
            env_cond.lighting = 'dawn/dusk';
        elseif avg_brightness > 0.15
            env_cond.lighting = 'night';
        else
            env_cond.lighting = 'dark_night';
        end
        
        % ==================== WEATHER DETECTION ====================
        % Analyze color channels for weather patterns
        red_ch = img(:, :, 1);
        green_ch = img(:, :, 2);
        blue_ch = img(:, :, 3);
        
        % Calculate color ratios
        blue_ratio = sum(blue_ch(:)) / (sum(red_ch(:)) + sum(green_ch(:)) + eps);
        red_ratio = sum(red_ch(:)) / (sum(green_ch(:)) + sum(blue_ch(:)) + eps);
        
        % Calculate contrast metric (lower in fog/snow)
        gray_img = rgb2gray(img);
        contrast_metric = std(double(gray_img(:)));
        
        % Classify weather conditions
        if blue_ratio > 0.45 && contrast_metric < 25
            env_cond.weather = 'rainy';
        elseif blue_ratio > 0.42 && avg_brightness > 0.75
            env_cond.weather = 'snowy';
        elseif blue_ratio > 0.4 && contrast_metric < 20
            env_cond.weather = 'foggy';
        elseif red_ratio > 0.38 && avg_brightness > 0.7
            env_cond.weather = 'clear';
        else
            env_cond.weather = 'cloudy';
        end
        
        % ==================== VALIDATION CHECKS ====================
        % Nighttime overrides - can't have bright conditions at night
        if contains(env_cond.lighting, {'night', 'dark_night'})
            if contains(env_cond.weather, {'snowy', 'clear'}) && avg_brightness < 0.3
                env_cond.weather = 'cloudy';
            end
        end
        
    catch ME
        % Fallback to CARLA's weather data if available
        env_cond.lighting = 'unknown';
        env_cond.weather = 'unknown';
    end
end

function attention = estimate_attention(img_data)
    % Simplified attention estimation
    try
        img_bytes = base64decode(img_data);
        % Actual implementation would use face/eye tracking
        % Placeholder value - would be replaced with real algorithm
        attention = 0.9;
    catch
        attention = 1.0; % Default to attentive
    end
end

function processCompleteFrame(frameData, log_fid, diag_text)
    % Process a complete frame with all sensor data - handle flexible field names
    persistent last_print;
    
    if isempty(last_print)
        last_print = tic;
    end
    
    % Determine frame identifier (handle different field names)
    frame_id = 0;
    if isfield(frameData, 'frame')
        frame_id = frameData.frame;
    elseif isfield(frameData, 'frame_id')
        frame_id = frameData.frame_id;
    elseif isfield(frameData, 'frameId')
        frame_id = frameData.frameId;
    end
    
    % Determine timestamp
    timestamp_val = 0;
    if isfield(frameData, 'timestamp')
        timestamp_val = frameData.timestamp;
    elseif isfield(frameData, 'time')
        timestamp_val = frameData.time;
    else
        timestamp_val = now();
    end
    
    % Log data to CSV (only if we have position data)
    position_data = [];
    if isfield(frameData, 'position')
        position_data = frameData.position;
    elseif isfield(frameData, 'location')
        position_data = frameData.location;
    elseif isfield(frameData, 'transform') && isfield(frameData.transform, 'location')
        position_data = frameData.transform.location;
    end
    
    if log_fid ~= -1 && ~isempty(position_data)
        % Extract GNSS data
        gnss_lat = 0; gnss_lon = 0; gnss_alt = 0;
        if isfield(frameData, 'gnss')
            gnss_lat = frameData.gnss.lat;
            gnss_lon = frameData.gnss.lon;
            gnss_alt = frameData.gnss.alt;
        end

        % Extract environment data
        env_map = '';
        env_weather = '';
        env_layer = '';
        if isfield(frameData, 'environment')
            env_map = strrep(frameData.environment.map, ',', '_');
            env_weather = strrep(frameData.environment.weather, ',', '_');
            env_layer = strrep(frameData.environment.layer, ',', '_');
        end
        
        % Update fprintf for CSV
        fprintf(log_fid, '%.6f,%d,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%s,%s,%s\n', ...
                timestamp_val, frame_id, speed_val, ...
                position_data.x, position_data.y, position_data.z, ...
                rotation_data.yaw, rotation_data.pitch, rotation_data.roll, ...
                control_data.throttle, control_data.steer, control_data.brake, ...
                gnss_lat, gnss_lon, gnss_alt, ...
                env_map, env_weather, env_layer);  % Added environment fields
        
        % Extract speed
        speed_val = 0;
        if isfield(frameData, 'speed')
            speed_val = frameData.speed;
        elseif isfield(frameData, 'velocity')
            if isstruct(frameData.velocity)
                speed_val = sqrt(frameData.velocity.x^2 + frameData.velocity.y^2 + frameData.velocity.z^2) * 3.6; % m/s to km/h
            else
                speed_val = frameData.velocity;
            end
        end
        
        % Extract rotation data
        rotation_data = struct('yaw', 0, 'pitch', 0, 'roll', 0);
        if isfield(frameData, 'rotation')
            rotation_data = frameData.rotation;
        elseif isfield(frameData, 'transform') && isfield(frameData.transform, 'rotation')
            rotation_data = frameData.transform.rotation;
        end
        
        % Extract control data
        control_data = struct('throttle', 0, 'steer', 0, 'brake', 0);
        if isfield(frameData, 'control')
            control_data = frameData.control;
        end
        
        fprintf(log_fid, '%.6f,%d,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f\n', ...
            timestamp_val, frame_id, speed_val, ...
            position_data.x, position_data.y, position_data.z, ...
            rotation_data.yaw, rotation_data.pitch, rotation_data.roll, ...
            control_data.throttle, control_data.steer, control_data.brake, ...
            gnss_lat, gnss_lon, gnss_alt);
    end
    
    % Save ALL raw sensor data to disk
    try
        frame_dir = fullfile('matlab_data', 'frames', sprintf('frame_%06d', frame_id));
        if ~exist(frame_dir, 'dir')
            mkdir(frame_dir);
        end
        saveFrameData(frameData, frame_dir, diag_text);
    catch ME
        log_diag(diag_text, sprintf('Frame %d save error: %s', frame_id, ME.message));
    end
    
    % Print detailed update every 2 seconds
    if toc(last_print) > 2.0
        % List all available fields for debugging
        available_fields = fieldnames(frameData);
        log_diag(diag_text, sprintf('[DEBUG] Available fields: %s', strjoin(available_fields, ', ')));
        
        try
            msg = sprintf('Frame: %d', frame_id);
            
            if ~isempty(position_data)
                msg = [msg sprintf(' | Position: (%.2f, %.2f, %.2f)', ...
                                   position_data.x, position_data.y, position_data.z)];
            end
            
            if isfield(frameData, 'speed')
                msg = [msg sprintf(' | Speed: %.2f km/h', frameData.speed)];
            elseif isfield(frameData, 'velocity')
                if isstruct(frameData.velocity)
                    speed_calc = sqrt(frameData.velocity.x^2 + frameData.velocity.y^2 + frameData.velocity.z^2) * 3.6;
                    msg = [msg sprintf(' | Speed: %.2f km/h (calc)', speed_calc)];
                end
            end
            
            if isfield(frameData, 'gnss')
                msg = [msg sprintf(' | GNSS: (%.6f, %.6f)', frameData.gnss.lat, frameData.gnss.lon)];
            end
            
            % Check for different image field patterns
            image_fields = available_fields(contains(available_fields, 'image', 'IgnoreCase', true) | ...
                                          contains(available_fields, 'camera', 'IgnoreCase', true));
            if ~isempty(image_fields)
                msg = [msg sprintf(' | Images: %s', strjoin(image_fields, ', '))];
            end
            
            if isfield(frameData, 'imu')
                msg = [msg ' | IMU: Available'];
            end
            
            if isfield(frameData, 'lidar')
                msg = [msg ' | LIDAR: Available'];
            end
            
            log_diag(diag_text, msg);
            last_print = tic;
        catch ME
            log_diag(diag_text, sprintf('Data print error: %s', ME.message));
        end
    end
end

function saveFrameData(frameData, frame_dir, diag_text)
    % Save all sensor data for a single frame while preserving environment data
    try
        % Save camera images
        cameras = {'front', 'back', 'left', 'right'};
        for i = 1:numel(cameras)
            field = ['image_' cameras{i}];
            if isfield(frameData, field)
                img_bytes = base64decode(frameData.(field));
                img_path = fullfile(frame_dir, [field '.jpg']);
                fid = fopen(img_path, 'wb');
                if fid ~= -1
                    fwrite(fid, img_bytes, 'uint8');
                    fclose(fid);
                else
                    log_diag(diag_text, sprintf('Failed to write image: %s', img_path));
                end
            end
        end
        
        % Save LIDAR data
        if isfield(frameData, 'lidar')
            lidar_bytes = base64decode(frameData.lidar);
            lidar_path = fullfile(frame_dir, 'lidar.bin');
            fid = fopen(lidar_path, 'wb');
            if fid ~= -1
                fwrite(fid, lidar_bytes, 'uint8');
                fclose(fid);
            else
                log_diag(diag_text, sprintf('Failed to write LIDAR: %s', lidar_path));
            end
        end
        
        % Save IMU data
        if isfield(frameData, 'imu')
            imu_bytes = base64decode(frameData.imu);
            imu_path = fullfile(frame_dir, 'imu.bin');
            fid = fopen(imu_path, 'wb');
            if fid ~= -1
                fwrite(fid, imu_bytes, 'uint8');
                fclose(fid);
            else
                log_diag(diag_text, sprintf('Failed to write IMU: %s', imu_path));
            end
        end
        
        % ====== KEY UPDATE: Preserve environment data in metadata ======
        % Create metadata copy
        meta = frameData;
        
        % List of large binary fields to remove
        large_fields = [...
            arrayfun(@(c) ['image_' c], cameras, 'UniformOutput', false), ...
            {'lidar', 'imu'}];
        
        % Remove large binary fields while preserving environment data
        for i = 1:length(large_fields)
            if isfield(meta, large_fields{i})
                meta = rmfield(meta, large_fields{i});
            end
        end
        
        % ====== ENVIRONMENT DATA IS AUTOMATICALLY PRESERVED ======
        % The 'environment' field remains in the metadata structure
        % since it's not in the large_fields removal list
        
        % Save metadata as JSON
        json_str = jsonencode(meta);
        json_path = fullfile(frame_dir, 'frame.json');
        fid = fopen(json_path, 'w');
        if fid ~= -1
            fprintf(fid, '%s', json_str);
            fclose(fid);
        else
            log_diag(diag_text, sprintf('Failed to write metadata: %s', json_path));
        end
        
    catch ME
        log_diag(diag_text, sprintf('Frame save error: %s', ME.message));
    end
end

function updateVisualizations(frameData, ax_front, ax_back, ax_left, ax_right, ax_lidar, ax_imu, diag_text)
    % Update camera images, LIDAR, and other sensor visualizations
    try
        % Process camera images
        cameras = {'front', 'back', 'left', 'right'};
        axes_handles = {ax_front, ax_back, ax_left, ax_right};
        
        for i = 1:numel(cameras)
            camName = cameras{i};
            fieldName = ['image_' camName];
            ax = axes_handles{i};
            
            if isfield(frameData, fieldName) && ~isempty(frameData.(fieldName))
                try
                    % Decode base64 image
                    imgData = base64decode(frameData.(fieldName));
                    
                    % Create temporary file for image decoding
                    temp_file = tempname;
                    temp_file = [temp_file '.jpg'];
                    
                    % Write bytes to file
                    fid = fopen(temp_file, 'wb');
                    fwrite(fid, imgData, 'uint8');
                    fclose(fid);
                    
                    % Read image
                    img = imread(temp_file);
                    
                    % Display image
                    cla(ax);
                    imshow(img, 'Parent', ax);
                    title(ax, [camName ' Camera']);
                    
                    % Clean up temp file
                    delete(temp_file);
                catch ME
                    log_diag(diag_text, sprintf('Image %s error: %s', camName, ME.message));
                end
            end
        end
        
        % Process LIDAR data
        if isfield(frameData, 'lidar') && ~isempty(frameData.lidar)
            try
                % Decode base64 LIDAR data
                lidarBytes = base64decode(frameData.lidar);
                
                % Convert to point cloud (Nx4 float array)
                pointCloud = typecast(lidarBytes, 'single');
                if mod(length(pointCloud), 4) == 0
                    pointCloud = reshape(pointCloud, 4, [])';
                    
                    % Extract XYZ coordinates (ignore intensity)
                    xyz = pointCloud(:,1:3);
                    
                    % Filter points (remove ground and distant points)
                    valid_points = xyz(:,3) > -2 & xyz(:,3) < 5 & ...
                                   sqrt(xyz(:,1).^2 + xyz(:,2).^2) < 50;
                    xyz = xyz(valid_points, :);
                    
                    % Display point cloud
                    cla(ax_lidar);
                    scatter3(ax_lidar, xyz(:,1), xyz(:,2), xyz(:,3), 1, 'b.');
                    title(ax_lidar, sprintf('LIDAR (%d points)', size(xyz,1)));
                    xlabel(ax_lidar, 'X (m)');
                    ylabel(ax_lidar, 'Y (m)');
                    zlabel(ax_lidar, 'Z (m)');
                    axis(ax_lidar, 'equal');
                    grid(ax_lidar, 'on');
                    view(ax_lidar, 0, 90); % Top-down view
                end
            catch ME
                log_diag(diag_text, sprintf('LIDAR error: %s', ME.message));
            end
        end
        
        % Process IMU data
        if isfield(frameData, 'imu') && ~isempty(frameData.imu)
            try
                % Decode base64 IMU data
                imuBytes = base64decode(frameData.imu);
                
                % Convert to float array [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
                if length(imuBytes) >= 24  % 6 floats * 4 bytes each
                    imuData = typecast(imuBytes(1:24), 'single');
                    
                    % Simple IMU visualization (could be enhanced)
                    cla(ax_imu);
                    bar(ax_imu, 1:3, imuData(1:3));
                    set(ax_imu, 'XTickLabel', {'Accel X', 'Accel Y', 'Accel Z'});
                    ylabel(ax_imu, 'Acceleration (m/s²)');
                    title(ax_imu, 'IMU Acceleration');
                    grid(ax_imu, 'on');
                end
            catch ME
                log_diag(diag_text, sprintf('IMU error: %s', ME.message));
            end
        end
        
    catch ME
        log_diag(diag_text, sprintf('Visualization error: %s', ME.message));
    end
end

function decoded = base64decode(str)
    % Simple base64 decoder using MATLAB's built-in functionality
    try
        % Remove any whitespace
        str = strrep(str, ' ', '');
        str = strrep(str, newline, '');
        str = strrep(str, char(13), '');
        str = strrep(str, char(10), '');
        
        % Use Java base64 decoder
        import java.util.Base64
        decoder = Base64.getDecoder();
        decoded = typecast(decoder.decode(uint8(str)), 'uint8');
    catch
        % Fallback: simple base64 decode (incomplete implementation)
        decoded = [];
        warning('Base64 decoding failed - ensure data is properly encoded');
    end
end

function log_diag(text_handle, message)
    % Append message to diagnostics console
    try
        current_text = get(text_handle, 'String');
        if ischar(current_text)
            current_text = {current_text};
        end
        new_text = [current_text; {sprintf('%s: %s', datestr(now, 'HH:MM:SS'), message)}];
        
        % Keep only last 25 lines
        if length(new_text) > 25
            new_text = new_text(end-24:end);
        end
        
        set(text_handle, 'String', new_text);
        drawnow;
    catch
        % Silently handle logging errors
    end
end

function close_receiver(u, log_fid)
    % Cleanup resources
    if nargin < 2
        log_fid = -1;
    end
    
    if log_fid > 0
        fclose(log_fid);
    end
    
    if isvalid(u)
        try
            flush(u);
            delete(u);
        catch
            % Ignore cleanup errors
        end
    end
end

function [jsonStr, remaining] = extractJSON(buffer)
    % Extract first complete JSON object from buffer
    jsonStr = '';
    remaining = buffer;
    
    % Find first opening brace
    startIdx = find(buffer == '{', 1);
    if isempty(startIdx)
        return;
    end
    
    % Find matching closing brace
    braceCount = 0;
    endIdx = 0;
    inString = false;
    escaped = false;
    
    for i = startIdx:length(buffer)
        char = buffer(i);
        
        if ~inString
            if char == '{'
                braceCount = braceCount + 1;
            elseif char == '}'
                braceCount = braceCount - 1;
                if braceCount == 0
                    endIdx = i;
                    break;
                end
            elseif char == '"'
                inString = true;
            end
        else
            if ~escaped
                if char == '"'
                    inString = false;
                elseif char == '\'
                    escaped = true;
                end
            else
                escaped = false;
            end
        end
    end
    
    % Extract complete JSON if found
    if endIdx > 0
        jsonStr = buffer(startIdx:endIdx);
        remaining = buffer(endIdx+1:end);
    end
end
function carla_udp_receiver(port)
% CARLA_UDP_RECEIVER - Receives, decrypts, and visualizes data from a CARLA simulation.
%
% This function sets up a UDP receiver to listen for encrypted and chunked data
% from the corresponding Python script. It handles data decryption, chunk
% reassembly, real-time visualization, and basic data analysis.
%
% Required Toolboxes:
%   - Lidar Toolbox
%   - Computer Vision Toolbox
%   - Deep Learning Toolbox (for face detection in attention estimation)
%
% Author: AI Assistant
% Version: 2.7
% Last Updated: 2023-10-27

% =========================================================================
%                             CONFIGURATION
% =========================================================================
if nargin < 1
    port = 10000;
end

% --- IMPORTANT: AES DECRYPTION KEY ---
% This key MUST be the same 32-byte (64-hex-character) key used in the Python script.
AES_KEY_HEX = 'eb04f73148e637b03050f665bf5adaa540edaad600e24de22d1166dd29b43ba7';

% History lengths for plots
MAX_HISTORY = 500; % Number of data points to store for plots


% =========================================================================
%                        GLOBAL OUTPUT & SETUP
% =========================================================================
% Declare global output structure for external access
global carla_outputs;
carla_outputs = struct(...
    'network_status', struct('active', false, 'last_message_time', tic), ...
    'processed_data', struct(...
        'fused_state', [], 'sensor_health', [], 'environment', [], ...
        'attention', 1.0, 'image_features', []), ...
    'fallback_initiation', false ...
);

% Set up UDP Port
fprintf('[INFO] Starting CARLA receiver on port %d...\n', port);
try
    udp_receiver = udpport("LocalPort", port, "Timeout", 1);
catch ME
    fprintf('[ERROR] Failed to set up UDP port: %s\n', ME.message);
    fprintf('        >> Ensure no other application is using port %d.\n', port);
    return;
end

% Set up Figure and Axes
[fig, ax] = setupFigure();

% Set up Data Storage
data_history = struct(...
    'trajectory', nan(MAX_HISTORY, 2), 'speed', nan(MAX_HISTORY, 1), ...
    'time', nan(MAX_HISTORY, 1), 'ptr', 1);
chunk_buffer = containers.Map('KeyType', 'double', 'ValueType', 'any');
start_time = tic;


% =========================================================================
%                           MAIN RECEIVER LOOP
% =========================================================================
fprintf('[INFO] Waiting for data from CARLA...\n');
try
    while ishandle(fig)
        if udp_receiver.NumBytesAvailable > 0
            datagram = read(udp_receiver, udp_receiver.NumBytesAvailable, "uint8");
            
            carla_outputs.network_status.active = true;
            carla_outputs.network_status.last_message_time = tic;
            
            payload = datagram;
            if isstruct(payload), payloads = {payload.Data}; else, payloads = {payload}; end

            for k = 1:numel(payloads)
                current_payload = payloads{k};
                
                try
                    if current_payload(1) == '{'
                        chunk_packet = jsondecode(char(current_payload'));
                        full_payload = handleChunk(chunk_packet, chunk_buffer, ax.diag_text);
                        if isempty(full_payload), continue; end
                        current_payload = full_payload;
                    end
                    
                    decrypted_json = decryptAES(current_payload, AES_KEY_HEX, ax.diag_text);
                    
                    if isempty(decrypted_json) || ~ischar(decrypted_json)
                        continue; % Error was already logged by decryptAES
                    end
                    
                    frame_data = jsondecode(decrypted_json);
                    
                catch ME
                    logDiagnostic(ax.diag_text, sprintf('[WARN] Packet processing error: %s', ME.message));
                    continue;
                end
                
                [processed_data, health_status] = processFrameData(frame_data, ax.diag_text);
                
                carla_outputs.processed_data = processed_data;
                carla_outputs.processed_data.sensor_health = health_status;
                carla_outputs.fallback_initiation = checkFallbackConditions(processed_data, health_status);
                
                data_history = updateDataHistory(data_history, processed_data, toc(start_time), MAX_HISTORY);
                
                updateVisualizations(processed_data, ax);
                updatePlotHistory(data_history, ax);
            end
        end
        
        updateDiagnostics(carla_outputs, ax.diag_text, chunk_buffer.Count);
        if carla_outputs.network_status.active && toc(carla_outputs.network_status.last_message_time) > 10
            logDiagnostic(ax.diag_text, '[WARN] Connection timeout. No data for 10s.');
            carla_outputs.network_status.active = false;
        end
        
        pause(0.01);
    end
catch ME
    fprintf('[FATAL ERROR] An unrecoverable error occurred in the main loop.\n');
    fprintf('Error: %s\n', ME.message);
    fprintf('File: %s, Line: %d\n', ME.stack(1).name, ME.stack(1).line);
end

fprintf('[INFO] Closing receiver...\n');
delete(udp_receiver);
end


% =========================================================================
%                        CORE PROCESSING FUNCTIONS
% =========================================================================

function [processed_data, health_status] = processFrameData(frame_data, diag_text_handle)
    processed_data = struct(...
        'fused_state', [], 'environment', [], 'attention', 1.0, ...
        'image_features', [], 'images', struct(), 'lidar_pc', [], ...
        'imu', [], 'speed', NaN);
    health_status = struct();
    
    if isfield(frame_data, 'fused_state') && ~isempty(frame_data.fused_state)
        processed_data.fused_state = frame_data.fused_state;
        health_status.fusion = 'OK';
    else
        health_status.fusion = 'MISSING';
    end
    
    if isfield(frame_data, 'speed'), processed_data.speed = frame_data.speed; end
    if isfield(frame_data, 'environment'), processed_data.environment = frame_data.environment; end

    sensor_fields = fieldnames(frame_data);
    for i = 1:numel(sensor_fields)
        field = sensor_fields{i};
        data_list = frame_data.(field);
        
        if iscell(data_list) && ~isempty(data_list)
            primary_data = data_list{1};
            try
                if startsWith(field, 'cam_')
                    img = imdecode(matlab.net.base64decode(primary_data), 'jpg');
                    cam_name = erase(field, 'cam_');
                    processed_data.images.(cam_name) = img;
                    health_status.(cam_name) = 'OK';
                elseif startsWith(field, 'lidar_')
                    pc_raw = typecast(matlab.net.base64decode(primary_data), 'single');
                    pc_raw = reshape(pc_raw, 4, [])';
                    processed_data.lidar_pc = pointCloud(pc_raw(:, 1:3));
                    health_status.lidar = 'OK';
                elseif startsWith(field, 'imu_')
                    processed_data.imu = primary_data;
                    health_status.imu = 'OK';
                end
            catch ME
                logDiagnostic(diag_text_handle, sprintf('[WARN] Failed to decode %s: %s', field, ME.message));
                cam_name_match = regexp(field, '^cam_(\w+)', 'tokens');
                if ~isempty(cam_name_match)
                    health_status.(cam_name_match{1}{1}) = 'ERROR';
                elseif startsWith(field, 'lidar_')
                    health_status.lidar = 'ERROR';
                end
            end
        end
    end
    
    if isfield(processed_data.images, 'interior')
        processed_data.attention = estimateAttention(processed_data.images.interior);
    end
    if isfield(processed_data.images, 'front_windshield')
        processed_data.image_features = extractImageFeatures(processed_data.images.front_windshield);
    end
end

function fallback = checkFallbackConditions(processed_data, health)
    fallback = false;
    is_risky_weather = false;
    if isfield(processed_data.environment, 'weather')
        is_risky_weather = (processed_data.environment.weather.precipitation > 50 || ...
                            processed_data.environment.weather.fog_density > 50);
    end
    if processed_data.attention < 0.4 && is_risky_weather
        fallback = true;
        disp('[FALLBACK] Low attention in risky weather!');
        return;
    end
    critical_sensors = {'fusion', 'front_windshield', 'lidar'};
    for i = 1:numel(critical_sensors)
        sensor_name = critical_sensors{i};
        if ~isfield(health, sensor_name) || ~strcmp(health.(sensor_name), 'OK')
            fallback = true;
            disp(['[FALLBACK] Critical sensor failure: ' sensor_name]);
            return;
        end
    end
end

% =========================================================================
%                       DATA ANALYSIS FUNCTIONS
% =========================================================================

function attention = estimateAttention(interior_img)
    persistent faceDetector;
    attention = 0.0;
    if isempty(faceDetector)
        try
            faceDetector = vision.CascadeObjectDetector('Model', 'FrontalFaceLBP');
        catch
            disp('[WARN] Could not create face detector. Attention estimation disabled.');
            attention = 1.0;
            return;
        end
    end
    bbox = faceDetector.step(interior_img);
    if ~isempty(bbox)
        face_area = bbox(1, 3) * bbox(1, 4);
        image_area = size(interior_img, 1) * size(interior_img, 2);
        attention = min(1.0, (face_area / image_area) * 10);
    end
end

function features = extractImageFeatures(front_img)
    features = struct('lanes', [], 'obstacles', []);
    try
        [~, ~, lanes] = extractLanes(front_img);
        features.lanes = lanes;
        persistent obstacleDetector;
        if isempty(obstacleDetector), obstacleDetector = acfObjectDetector('caltech-50x21'); end
        [bboxes, scores] = obstacleDetector.detect(front_img, 'SelectStrongest', false);
        strong_detections = scores > 20;
        features.obstacles.bboxes = bboxes(strong_detections, :);
        features.obstacles.scores = scores(strong_detections);
    catch
        disp('[WARN] Feature extraction failed. Computer Vision Toolbox may be missing.');
    end
end


% =========================================================================
%                   NETWORKING & HELPER FUNCTIONS
% =========================================================================

function decrypted = decryptAES(data, key_hex, diag_handle)
    % Decrypts AES-256-CBC data from Python, handling MATLAB's data types
    % by explicitly creating native Java byte arrays.
    
    try
        % FIX: Convert hex key to uint8, then create a native Java byte array
        key_uint8 = uint8(hex2dec(reshape(key_hex, 2, [])'));
        java_key_bytes = java.lang.reflect.Array.newInstance(java.lang.Byte.TYPE, numel(key_uint8));
        for i = 1:numel(key_uint8), java_key_bytes(i) = typecast(key_uint8(i), 'int8'); end

        % FIX: Convert uint8 data to native Java byte array
        data_uint8 = uint8(data(:)');
        java_data_bytes = java.lang.reflect.Array.newInstance(java.lang.Byte.TYPE, numel(data_uint8));
        for i = 1:numel(data_uint8), java_data_bytes(i) = typecast(data_uint8(i), 'int8'); end
        
        iv_size = 16;
        if numel(java_data_bytes) <= iv_size
            error('Invalid data: payload is too small for IV.');
        end
        
        % Create native Java byte arrays for IV and ciphertext
        iv_bytes = java.lang.reflect.Array.newInstance(java.lang.Byte.TYPE, iv_size);
        java.lang.System.arraycopy(java_data_bytes, 0, iv_bytes, 0, iv_size);

        cipher_len = numel(java_data_bytes) - iv_size;
        ciphertext_bytes = java.lang.reflect.Array.newInstance(java.lang.Byte.TYPE, cipher_len);
        java.lang.System.arraycopy(java_data_bytes, iv_size, ciphertext_bytes, 0, cipher_len);
        
        % Perform decryption using Java objects
        cipher = javax.crypto.Cipher.getInstance('AES/CBC/PKCS5Padding');
        secretKey = javax.crypto.spec.SecretKeySpec(java_key_bytes, 'AES');
        ivSpec = javax.crypto.spec.IvParameterSpec(iv_bytes);
        cipher.init(javax.crypto.Cipher.DECRYPT_MODE, secretKey, ivSpec);
        
        plaintext_java_bytes = cipher.doFinal(ciphertext_bytes);
        
        % Convert the result back to a MATLAB string
        decrypted = native2unicode(typecast(plaintext_java_bytes, 'uint8'), 'UTF-8');
    catch me
        if contains(me.message, 'BadPaddingException')
            logDiagnostic(diag_handle, '[FATAL] DECRYPTION FAILED: Bad Padding. The AES KEY is WRONG.');
        else
            logDiagnostic(diag_handle, sprintf('[ERROR] Decryption error: %s', me.message));
        end
        decrypted = []; % Return empty on failure
    end
end

function full_payload = handleChunk(chunk_packet, chunk_buffer, diag_text_handle)
    full_payload = [];
    frame_id = chunk_packet.frame;
    
    if ~isKey(chunk_buffer, frame_id)
        chunk_buffer(frame_id) = struct(...
            'chunks', {cell(1, chunk_packet.total_chunks)}, ...
            'received_count', 0, 'total_chunks', chunk_packet.total_chunks);
    end
    
    buffer_entry = chunk_buffer(frame_id);
    chunk_index = chunk_packet.chunk + 1;
    
    if isempty(buffer_entry.chunks{chunk_index})
        buffer_entry.chunks{chunk_index} = matlab.net.base64decode(chunk_packet.data);
        buffer_entry.received_count = buffer_entry.received_count + 1;
        chunk_buffer(frame_id) = buffer_entry;
    end
    
    if buffer_entry.received_count == buffer_entry.total_chunks
        logDiagnostic(diag_text_handle, sprintf('[INFO] Reassembled frame %d.', frame_id));
        if ~any(cellfun('isempty', buffer_entry.chunks))
            full_payload = horzcat(buffer_entry.chunks{:});
        end
        remove(chunk_buffer, frame_id);
    end
end

function history = updateDataHistory(history, data, time, max_size)
    ptr = history.ptr;
    if ~isempty(data.fused_state)
        history.trajectory(ptr, :) = [data.fused_state.x, data.fused_state.y];
    end
    history.speed(ptr) = data.speed;
    history.time(ptr) = time;
    history.ptr = mod(ptr, max_size) + 1;
end

function logDiagnostic(handle, message)
    try
        current_text = get(handle, 'String');
        if ~iscell(current_text), current_text = {current_text}; end
        new_text = [current_text; {sprintf('%s: %s', datestr(now, 'HH:MM:SS'), message)}];
        if numel(new_text) > 20, new_text = new_text(end-19:end); end
        set(handle, 'String', new_text);
        drawnow('limitrate');
    catch
    end
end


% =========================================================================
%                         VISUALIZATION FUNCTIONS
% =========================================================================

function [fig, ax] = setupFigure()
    fig = figure('Name', 'CARLA Real-Time Dashboard', 'NumberTitle', 'off', ...
                 'Position', [50, 50, 1600, 900], 'Color', [0.1 0.1 0.1]);
    
    tiled_layout = tiledlayout(3, 3, 'Parent', fig, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    ax.trajectory = nexttile(tiled_layout, 1, [2 1]);
    title(ax.trajectory, 'Fused Trajectory', 'Color', 'w');
    set(ax.trajectory, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.4 0.4 0.4]);
    xlabel(ax.trajectory, 'X (m)'); ylabel(ax.trajectory, 'Y (m)');
    grid(ax.trajectory, 'on'); hold(ax.trajectory, 'on'); axis(ax.trajectory, 'equal');

    ax.front_cam = nexttile(tiled_layout, 2); title(ax.front_cam, 'Front Camera', 'Color', 'w'); axis(ax.front_cam, 'off');
    ax.rear_cam = nexttile(tiled_layout, 3); title(ax.rear_cam, 'Rear Camera', 'Color', 'w'); axis(ax.rear_cam, 'off');
    ax.left_cam = nexttile(tiled_layout, 5); title(ax.left_cam, 'Left Mirror', 'Color', 'w'); axis(ax.left_cam, 'off');
    ax.right_cam = nexttile(tiled_layout, 6); title(ax.right_cam, 'Right Mirror', 'Color', 'w'); axis(ax.right_cam, 'off');
    
    ax.lidar = nexttile(tiled_layout, 4);
    title(ax.lidar, 'LIDAR Point Cloud', 'Color', 'w');
    set(ax.lidar, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', 'GridColor', [0.4 0.4 0.4]);
    grid(ax.lidar, 'on'); axis(ax.lidar, 'equal'); view(ax.lidar, 3);
    
    ax.speed = nexttile(tiled_layout, 7);
    title(ax.speed, 'Speed (km/h)', 'Color', 'w');
    set(ax.speed, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.4 0.4 0.4]);
    xlabel(ax.speed, 'Time (s)'); grid(ax.speed, 'on');
    
    ax.attention = nexttile(tiled_layout, 8);
    title(ax.attention, 'Driver Attention', 'Color', 'w');
    set(ax.attention, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
    ylim(ax.attention, [0 1]);
    
    diag_panel = uipanel(fig, 'Title', 'Diagnostics', 'Position', [0.67 0.01 0.32 0.32], ...
                         'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'w');
    ax.diag_text = uicontrol(diag_panel, 'Style', 'edit', 'Max', 10, 'Min', 0, ...
                          'HorizontalAlignment', 'left', 'Position', [10, 10, 500, 240], ...
                          'String', 'Initializing...', 'FontName', 'Consolas', 'FontSize', 9, ...
                          'BackgroundColor', 'k', 'ForegroundColor', 'g');
end

function updateVisualizations(data, ax)
    cam_fields = {'front_windshield', 'rear', 'left_mirror', 'right_mirror'};
    ax_map = {ax.front_cam, ax.rear_cam, ax.left_cam, ax.right_cam};
    for i = 1:numel(cam_fields)
        if isfield(data.images, cam_fields{i})
            imshow(data.images.(cam_fields{i}), 'Parent', ax_map{i});
        end
    end
    
    if ~isempty(data.lidar_pc) && data.lidar_pc.Count > 0
        pcshow(data.lidar_pc, 'Parent', ax.lidar, 'MarkerSize', 2);
        view(ax.lidar, -45, 30);
    end
    
    cla(ax.attention);
    bar(ax.attention, data.attention, 'FaceColor', [0.2 0.8 0.2]);
    ylim(ax.attention, [0 1]);
    set(ax.attention, 'XTick', []);
end

function updatePlotHistory(history, ax)
    valid_traj = ~isnan(history.trajectory(:,1));
    if any(valid_traj)
        cla(ax.trajectory);
        plot(ax.trajectory, history.trajectory(valid_traj, 1), history.trajectory(valid_traj, 2), 'c-', 'LineWidth', 1.5);
        plot(ax.trajectory, history.trajectory(history.ptr-1, 1), history.trajectory(history.ptr-1, 2), 'r*', 'MarkerSize', 10, 'LineWidth', 2);
    end
    
    valid_speed = ~isnan(history.speed);
    if any(valid_speed)
        cla(ax.speed);
        plot(ax.speed, history.time(valid_speed), history.speed(valid_speed), 'r-', 'LineWidth', 1.5);
    end
    
    drawnow('limitrate');
end

function updateDiagnostics(outputs, handle, chunk_count)
    persistent last_update;
    if isempty(last_update), last_update = tic; end
    if toc(last_update) < 1.0, return; end 
    
    status_str = {'-- SYSTEM STATUS --'};
    if outputs.network_status.active
        status_str{end+1} = 'Network: ACTIVE';
    else
        status_str{end+1} = 'Network: INACTIVE';
    end
    status_str{end+1} = sprintf('Buffered Chunks: %d', chunk_count);
    status_str{end+1} = sprintf('Attention Level: %.2f', outputs.processed_data.attention);
    
    status_str{end+1} = '-- SENSOR HEALTH --';
    if isfield(outputs.processed_data, 'sensor_health') && isstruct(outputs.processed_data.sensor_health)
        fields = fieldnames(outputs.processed_data.sensor_health);
        for i = 1:numel(fields)
            status_str{end+1} = sprintf('%-18s: %s', fields{i}, outputs.processed_data.sensor_health.(fields{i}));
        end
    end
    
    status_str{end+1} = '-- FALLBACK STATUS --';
    if outputs.fallback_initiation
        status_str{end+1} = 'ACTION: INITIATE FALLBACK';
    else
        status_str{end+1} = 'ACTION: Nominal';
    end
    
    set(handle, 'String', status_str);
    last_update = tic;
end
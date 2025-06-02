function carla_udp_receiver(port)
    % Enhanced CARLA UDP receiver with chunked data, base64 decoding, and sensor processing
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
    if ~exist(data_dir, 'dir')
        mkdir(data_dir);
        log_diag(diag_text, sprintf('Created data directory: %s', data_dir));
    end
    
    % Create CSV log file
    log_file = fullfile(data_dir, 'vehicle_data.csv');
    log_fid = fopen(log_file, 'w');
    if log_fid == -1
        log_diag(diag_text, sprintf('[ERROR] Failed to create log file: %s', log_file));
    else
        fprintf(log_fid, 'timestamp,frame,speed,x,y,z,yaw,pitch,roll,throttle,steer,brake,gnss_lat,gnss_lon,gnss_alt\n');
    end
    
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
            
            % Update diagnostics panel
            if toc(last_diag_update) > 1.0
                status_msg = sprintf('PORT STATUS: %d\n', port);
                status_msg = [status_msg sprintf('Bytes received: %d\n', bytes_received)];
                status_msg = [status_msg sprintf('Messages received: %d\n', messages_received)];
                status_msg = [status_msg sprintf('Buffered chunks: %d\n', chunkBuffer.Count)];
                
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
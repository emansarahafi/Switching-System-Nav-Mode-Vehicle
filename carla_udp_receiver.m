function carla_udp_receiver(port)
    % Enhanced CARLA UDP receiver with diagnostics
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
                 'Position', [100, 100, 1200, 800], 'DeleteFcn', @(src,event)close_receiver(u));
    
    % Create diagnostic console
    diag_panel = uipanel(fig, 'Title', 'Diagnostics', 'Position', [0.01 0.01 0.98 0.25]);
    diag_text = uicontrol(diag_panel, 'Style', 'edit', 'Max', 10, 'Min', 0, ...
                          'HorizontalAlignment', 'left', ...
                          'Position', [10, 10, 1150, 130], 'String', 'Initializing...', ...
                          'FontName', 'Consolas', 'FontSize', 9);
    
    % Create axes for trajectory plot
    ax_traj = subplot(2,2,1, 'Parent', fig);
    hold(ax_traj, 'on');
    grid(ax_traj, 'on');
    xlabel(ax_traj, 'X Position (m)');
    ylabel(ax_traj, 'Y Position (m)');
    title(ax_traj, 'Vehicle Trajectory');
    
    % Create axes for camera images
    ax_front = subplot(2,2,2, 'Parent', fig);
    title(ax_front, 'Front Camera');
    axis(ax_front, 'off');
    
    ax_back = subplot(2,2,3, 'Parent', fig);
    title(ax_back, 'Back Camera');
    axis(ax_back, 'off');
    
    ax_left = subplot(2,2,4, 'Parent', fig);
    title(ax_left, 'Left Camera');
    axis(ax_left, 'off');
    
    % Initialize data storage
    trajectory = [];
    last_plot_update = tic;
    last_diag_update = tic;
    buffer = '';
    frame_data = struct();
    image_base_dir = 'data/images';
    connection_active = false;
    bytes_received = 0;
    messages_received = 0;
    last_message_time = tic;
    
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
        fprintf(log_fid, 'timestamp,frame,speed,x,y,z,yaw,pitch,roll,throttle,steer,brake\n');
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
                        data = jsondecode(jsonStr);
                        messages_received = messages_received + 1;
                        
                        % Log data to CSV
                        if log_fid ~= -1
                            fprintf(log_fid, '%.6f,%d,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n', ...
                                data.timestamp, data.frame, data.speed, ...
                                data.position.x, data.position.y, data.position.z, ...
                                data.rotation.yaw, data.rotation.pitch, data.rotation.roll, ...
                                data.control.throttle, data.control.steer, data.control.brake);
                        end
                        
                        % Store for processing
                        frame_data = data;
                        
                        % Add to trajectory
                        trajectory = [trajectory; [data.position.x, data.position.y]];
                        
                        % Process data
                        process_carla_data(data, diag_text);
                    catch ME
                        log_diag(diag_text, sprintf('[ERROR] JSON processing: %s', ME.message));
                        log_diag(diag_text, sprintf('Message: %s', jsonStr(1:min(100, end))));
                    end
                end
            end
            
            % Update diagnostics panel
            if toc(last_diag_update) > 1.0
                status_msg = sprintf('PORT STATUS: %d\n', port);
                status_msg = [status_msg sprintf('Bytes received: %d\n', bytes_received)];
                status_msg = [status_msg sprintf('Messages received: %d\n', messages_received)];
                
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
                
                if isfield(frame_data, 'frame')
                    status_msg = [status_msg sprintf('Last frame: %d\n', frame_data.frame)];
                end
                
                set(diag_text, 'String', status_msg);
                last_diag_update = tic;
            end
            
            % Update plot periodically
            if ~isempty(trajectory) && toc(last_plot_update) > 0.2
                try
                    cla(ax_traj);
                    plot(ax_traj, trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 1.5);
                    plot(ax_traj, trajectory(end,1), trajectory(end,2), 'ro', ...
                         'MarkerSize', 8, 'MarkerFaceColor', 'r');
                    drawnow limitrate;
                catch
                    % Silently handle plot errors
                end
                last_plot_update = tic;
            end
            
            % Update images periodically
            if isfield(frame_data, 'frame') && isfield(frame_data, 'image_paths')
                try
                    % Front camera
                    if isfield(frame_data.image_paths, 'front')
                        front_path = frame_data.image_paths.front;
                        if exist(front_path, 'file')
                            img = imread(front_path);
                            image(ax_front, img);
                            axis(ax_front, 'off');
                        end
                    end
                    
                    % Back camera
                    if isfield(frame_data.image_paths, 'back')
                        back_path = frame_data.image_paths.back;
                        if exist(back_path, 'file')
                            img = imread(back_path);
                            image(ax_back, img);
                            axis(ax_back, 'off');
                        end
                    end
                    
                    % Left camera
                    if isfield(frame_data.image_paths, 'left')
                        left_path = frame_data.image_paths.left;
                        if exist(left_path, 'file')
                            img = imread(left_path);
                            image(ax_left, img);
                            axis(ax_left, 'off');
                        end
                    end
                    
                    drawnow limitrate;
                catch ME
                    log_diag(diag_text, sprintf('Image error: %s', ME.message));
                end
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

function log_diag(text_handle, message)
    % Append message to diagnostics console
    current_text = get(text_handle, 'String');
    if ischar(current_text)
        current_text = {current_text};
    end
    new_text = [current_text; {sprintf('%s: %s', datestr(now, 'HH:MM:SS'), message)}];
    
    % Keep only last 20 lines
    if length(new_text) > 20
        new_text = new_text(end-19:end);
    end
    
    set(text_handle, 'String', new_text);
    drawnow;
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
    for i = startIdx:length(buffer)
        if buffer(i) == '{'
            braceCount = braceCount + 1;
        elseif buffer(i) == '}'
            braceCount = braceCount - 1;
            if braceCount == 0
                endIdx = i;
                break;
            end
        end
    end
    
    % Extract complete JSON if found
    if endIdx > 0
        jsonStr = buffer(startIdx:endIdx);
        remaining = buffer(endIdx+1:end);
    end
end

function process_carla_data(data, diag_text)
    % Process incoming CARLA data
    persistent last_print;
    
    if isempty(last_print)
        last_print = tic;
    end
    
    % Print update every 2 seconds
    if toc(last_print) > 2.0
        try
            msg = sprintf('Frame: %d', data.frame);
            msg = [msg sprintf('\n  Position: (%.2f, %.2f, %.2f)', ...
                               data.position.x, data.position.y, data.position.z)];
            msg = [msg sprintf('\n  Speed: %.2f km/h', data.speed)];
            msg = [msg sprintf('\n  Rotation: Yaw=%.1f, Pitch=%.1f, Roll=%.1f', ...
                               data.rotation.yaw, data.rotation.pitch, data.rotation.roll)];
            msg = [msg sprintf('\n  Control: Throttle=%.2f, Steer=%.2f, Brake=%.2f', ...
                               data.control.throttle, data.control.steer, data.control.brake)];
            
            log_diag(diag_text, msg);
            last_print = tic;
        catch ME
            log_diag(diag_text, sprintf('Data print error: %s', ME.message));
        end
    end
    
    % Add image paths to data structure
    if ~isfield(data, 'image_paths')
        image_base_dir = 'data/images';
        data.image_paths = struct();
        data.image_paths.front = fullfile(image_base_dir, 'front', sprintf('frame_%05d.png', data.frame));
        data.image_paths.back = fullfile(image_base_dir, 'back', sprintf('frame_%05d.png', data.frame));
        data.image_paths.left = fullfile(image_base_dir, 'left', sprintf('frame_%05d.png', data.frame));
    end
end
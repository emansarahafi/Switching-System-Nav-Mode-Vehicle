% =======================================================================
%                          FDIR CYCLE
% =======================================================================

function run_fdir_cycle(command_sender)
% RUN_FDIR_CYCLE - Performs fault detection, isolation, and RECOVERY.
% This version actively tries to switch to backup sensors before failing.

    % --- Configuration ---
    % Define which sensors are absolutely critical. If no backup is available
    % for these, the system will enter safe mode.
    CRITICAL_SENSORS = {'gnss', 'imu', 'lidar_roof', 'lidar_front', 'lidar_back','cam_front', 'cam_back', 'cam_left', 'cam_right', 'cam_interior', 'radar_front', 'radar_back', 'ultrasonic_front', 'ultrasonic_back', 'radar_front_left', 'radar_front_right', 'radar_rear_left', 'radar_rear_right', 'collision', 'lane_invasion'};

    global carla_outputs;

    % Guard clause: Do nothing if the global outputs aren't ready or don't
    % contain the necessary health and active index information.
    if ~isstruct(carla_outputs) || ~isfield(carla_outputs, 'sensor_health') || ...
       ~isfield(carla_outputs, 'active_sensor_indices') || ...
       isempty(fieldnames(carla_outputs.sensor_health))
        return;
    end
    
    % Assume the system is nominal until a fault is found.
    carla_outputs.fallback_initiation = false;
    
    health_data = carla_outputs.sensor_health;
    active_indices = carla_outputs.active_sensor_indices;
    sensor_groups = fieldnames(health_data);

    % Iterate through each sensor group (e.g., 'cam_front', 'gnss')
    for i = 1:numel(sensor_groups)
        group_name = sensor_groups{i};
        
        % Get the health statuses and active index for this group.
        % Note: Python is 0-indexed, MATLAB is 1-indexed.
        statuses = health_data.(group_name);
        active_idx = active_indices.(group_name) + 1; % Convert to 1-based index

        % Check if the currently active sensor is faulty.
        if active_idx <= numel(statuses) && ~strcmp(statuses{active_idx}, 'OK')
            
            faulty_status = statuses{active_idx};
            fprintf('[FDIR] Fault Detected in active sensor %s #%d (Status: %s)\n', ...
                    group_name, active_idx - 1, faulty_status);
            
            % --- ISOLATION & RECOVERY ---
            % Try to find a healthy backup sensor in the same group.
            % Find the index of the first available 'OK' sensor.
            new_active_idx = find(strcmp(statuses, 'OK'), 1, 'first');
            
            if ~isempty(new_active_idx)
                % A healthy backup was found!
                fprintf('[FDIR] RECOVERY: Activating backup for %s. New index: %d\n', ...
                        group_name, new_active_idx - 1);
                
                % Command Python to switch to the backup sensor (use 0-based index).
                command_activate_backup(command_sender, group_name, new_active_idx - 1);
                
                % We've initiated recovery, so we exit this FDIR cycle.
                % The next cycle will see the new active sensor.
                return; 
            end
            
            % --- NO RECOVERY POSSIBLE ---
            if isempty(new_active_idx)
                fprintf('[FDIR] RECOVERY FAILED for %s. No healthy backups available.\n', group_name);
                
                % Check if this unrecoverable sensor is critical.
                if ismember(group_name, CRITICAL_SENSORS)
                    carla_outputs.fallback_initiation = true;
                    reason = sprintf('Critical sensor failure: %s. No backups available.', group_name);
                    command_switch_to_safe_mode(command_sender, reason);
                    return; % A critical fault occurred, exit cycle.
                else
                    fprintf('[FDIR] Warning: Non-critical sensor %s has failed permanently.\n', group_name);
                end
            end
        end
    end
end

%% --- Helper Functions for FDIR Actions ---

function command_activate_backup(udp_sender, sensor_group, backup_index)
% Sends a command to Python to activate a specific backup sensor.
    persistent last_command_time;
    if isempty(last_command_time), last_command_time = containers.Map('KeyType', 'char', 'ValueType', 'any'); end

    % Prevent spamming the same command for the same sensor group.
    if isKey(last_command_time, sensor_group) && toc(last_command_time(sensor_group)) < 2.0
        return;
    end
    
    if ~isempty(udp_sender) && isvalid(udp_sender)
        command = struct('command', 'ACTIVATE_BACKUP', 'sensor_group', sensor_group, 'backup_index', backup_index);
        json_cmd = jsonencode(command);
        write(udp_sender, json_cmd, "char", "127.0.0.1", 10001);
        last_command_time(sensor_group) = tic;
    end
end


function command_switch_to_safe_mode(udp_sender, reason)
    persistent last_command_time;
    
    % To prevent spamming, only send the command once every 2 seconds.
    if ~isempty(last_command_time) && toc(last_command_time) < 2.0
        return;
    end

    fprintf('\n====================================================\n');
    fprintf('!!! FDIR CRITICAL FAULT: SWITCH TO SAFE MODE !!!\n');
    fprintf('!!! REASON: %s\n', reason);
    fprintf('====================================================\n');
    beep; pause(0.1); beep;
    
    if ~isempty(udp_sender) && isvalid(udp_sender)
        command = struct('command', 'ENTER_SAFE_MODE', 'reason', reason);
        json_cmd = jsonencode(command);
        write(udp_sender, json_cmd, "char", "127.0.0.1", 10001);
        last_command_time = tic;
    end
end

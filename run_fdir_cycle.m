% =======================================================================
%                          FDIR CYCLE
% =======================================================================

function run_fdir_cycle(command_sender)
% RUN_FDIR_CYCLE - Performs fault detection, isolation, and RECOVERY.
% This version only considers 'DEAD' as a hard fault requiring recovery.

    % --- Configuration ---
    CRITICAL_SENSORS = {'gnss', 'imu', 'lidar_roof', 'lidar_front', 'lidar_back','cam_front', 'cam_back', 'cam_left', 'cam_right', 'cam_interior', 'radar_front', 'radar_back', 'ultrasonic_front', 'ultrasonic_back', 'collision', 'lane_invasion'};

    global carla_outputs;

    % Guard clause
    if ~isstruct(carla_outputs) || ~isfield(carla_outputs, 'sensor_health') || ...
       ~isfield(carla_outputs, 'active_sensor_indices') || ...
       isempty(fieldnames(carla_outputs.sensor_health))
        return;
    end
    
    carla_outputs.fallback_initiation = false;
    
    health_data = carla_outputs.sensor_health;
    active_indices = carla_outputs.active_sensor_indices;
    sensor_groups = fieldnames(health_data);

    for i = 1:numel(sensor_groups)
        group_name = sensor_groups{i};
        
        statuses = health_data.(group_name);
        active_idx = active_indices.(group_name) + 1;

        % --- FAULT CONDITION ---
        % Check if the currently active sensor is explicitly 'DEAD'.
        % This will now ignore 'NO_DATA' and only react to hard failures.
        if active_idx <= numel(statuses) && strcmp(statuses{active_idx}, 'DEAD')
            
            fprintf('[FDIR] Hard Fault Detected in active sensor %s #%d (Status: DEAD)\n', ...
                    group_name, active_idx - 1);
            
            % --- ISOLATION & RECOVERY ---
            % Logic remains the same: find the first available 'OK' sensor.
            new_active_idx = find(strcmp(statuses, 'OK'), 1, 'first');
            
            if ~isempty(new_active_idx)
                fprintf('[FDIR] RECOVERY: Activating backup for %s. New index: %d\n', ...
                        group_name, new_active_idx - 1);
                
                command_activate_backup(command_sender, group_name, new_active_idx - 1);
                return; 
            end
            
            % --- NO RECOVERY POSSIBLE ---
            if isempty(new_active_idx)
                fprintf('[FDIR] RECOVERY FAILED for %s. No healthy backups available.\n', group_name);
                
                if ismember(group_name, CRITICAL_SENSORS)
                    carla_outputs.fallback_initiation = true;
                    reason = sprintf('Critical sensor failure: %s. No backups available.', group_name);
                    command_switch_to_safe_mode(command_sender, reason);
                    return;
                else
                    fprintf('[FDIR] Warning: Non-critical sensor %s has failed permanently.\n', group_name);
                end
            end
        end
    end
end

%% --- Helper Functions for FDIR Actions (These are already correct) ---

function command_activate_backup(udp_sender, sensor_group, backup_index)
    persistent last_command_time;
    if isempty(last_command_time), last_command_time = containers.Map('KeyType', 'char', 'ValueType', 'any'); end

    if isKey(last_command_time, sensor_group) && toc(last_command_time(sensor_group)) < 2.0
        return;
    end
    
    if ~isempty(udp_sender) && isvalid(udp_sender)
        command = struct(...
            'command', 'ACTIVATE_BACKUP', ...
            'sensor_group', sensor_group, ...
            'backup_index', backup_index, ...
            'api_key', "SECRET_CARLA_KEY_123" ...
        );
        json_cmd = jsonencode(command);
        write(udp_sender, json_cmd, "char", "127.0.0.1", 10001);
        last_command_time(sensor_group) = tic;
    end
end


function command_switch_to_safe_mode(udp_sender, reason)
    persistent last_command_time;
    
    if ~isempty(last_command_time) && toc(last_command_time) < 2.0
        return;
    end

    fprintf('\n====================================================\n');
    fprintf('!!! FDIR CRITICAL FAULT: SWITCH TO SAFE MODE !!!\n');
    fprintf('!!! REASON: %s\n', reason);
    fprintf('====================================================\n');
    beep; pause(0.1); beep;
    
    if ~isempty(udp_sender) && isvalid(udp_sender)
        command = struct(...
            'command', 'ENTER_SAFE_MODE', ...
            'reason', reason, ...
            'api_key', "SECRET_CARLA_KEY_123" ...
        );
        json_cmd = jsonencode(command);
        write(udp_sender, json_cmd, "char", "127.0.0.1", 10001);
        last_command_time = tic;
    end
end

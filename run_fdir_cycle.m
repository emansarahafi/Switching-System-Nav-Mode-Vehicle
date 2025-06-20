function run_fdir_cycle(command_sender)
% RUN_FDIR_CYCLE - Performs a single check cycle of the FDIR algorithm.
% This function is designed to be called repeatedly by a MATLAB timer.
% It monitors the global 'carla_outputs' struct and makes decisions,
% sending commands back to the Python simulation via UDP.

    % --- State Tracking ---
    persistent sensor_states;
    
    if isempty(sensor_states)
        fprintf('FDIR System Initialized. Monitoring CARLA outputs...\n');
        sensor_states = containers.Map('KeyType', 'char', 'ValueType', 'char');
    end

    global carla_outputs;

    if ~isstruct(carla_outputs) || isempty(fieldnames(carla_outputs))
        return;
    end
    
    % --- Decision Logic ---
    is_critical_risk = false;
    reason_for_safe_mode = '';
    
    if isfield(carla_outputs, 'processed_sensor_data')
        if isfield(carla_outputs.processed_sensor_data, 'is_collision_event') && carla_outputs.processed_sensor_data.is_collision_event
            is_critical_risk = true;
            reason_for_safe_mode = 'Collision Detected';
        elseif isfield(carla_outputs.processed_sensor_data, 'is_lane_invasion_event') && carla_outputs.processed_sensor_data.is_lane_invasion_event
             is_critical_risk = true;
             reason_for_safe_mode = 'Lane Invasion Detected';
        end
    end

    if is_critical_risk
        command_switch_to_safe_mode(command_sender, reason_for_safe_mode);
        return;
    end
    
    if isfield(carla_outputs, 'fallback_initiation') && carla_outputs.fallback_initiation
        if carla_outputs.sensor_fusion_status.health_score < 1.0
            raw_health = carla_outputs.sensor_fusion_status.raw_health;
            sensor_groups = fieldnames(raw_health);
            
            for i = 1:numel(sensor_groups)
                group_name = sensor_groups{i};
                statuses = raw_health.(group_name);
                current_state = get_sensor_state(sensor_states, group_name);
                first_failed_idx = find(~strcmp(statuses, 'OK'), 1);

                if ~isempty(first_failed_idx)
                    if first_failed_idx == 1 && strcmp(current_state, 'Nominal')
                        command_recover_sensor(command_sender, group_name, 1);
                        sensor_states(group_name) = 'Degraded_1';
                        break;
                    elseif first_failed_idx <= 2 && strcmp(current_state, 'Degraded_1')
                        command_recover_sensor(command_sender, group_name, 2);
                        sensor_states(group_name) = 'Degraded_2';
                        break;
                    elseif first_failed_idx <= 3 && strcmp(current_state, 'Degraded_2')
                        reason = sprintf("Unrecoverable failure in sensor group '%s'.", group_name);
                        command_switch_to_safe_mode(command_sender, reason);
                        sensor_states(group_name) = 'Failed';
                        break;
                    end
                end
            end
        end
    end
end

%% --- Helper Functions for FDIR ---

function state = get_sensor_state(state_map, sensor_name)
    if isKey(state_map, sensor_name), state = state_map(sensor_name);
    else, state = 'Nominal'; end
end

function command_recover_sensor(udp_sender, sensor_name, backup_level)
    fprintf('[FDIR COMMAND]: Recovering sensor group ''%s''. Activating backup #%d.\n', sensor_name, backup_level);
    if ~isempty(udp_sender)
        command = struct('command', 'ACTIVATE_BACKUP', 'sensor_group', sensor_name, 'backup_index', backup_level);
        json_cmd = jsonencode(command);
        write(udp_sender, json_cmd, "char", "127.0.0.1", 10001);
    end
end

function command_switch_to_safe_mode(udp_sender, reason)
    fprintf('\n====================================================\n');
    fprintf('!!! FDIR CRITICAL ACTION: SWITCH TO SAFE MODE !!!\n');
    fprintf('!!! REASON: %s\n', reason);
    fprintf('====================================================\n');
    beep; pause(0.1); beep;
    
    if ~isempty(udp_sender)
        command = struct('command', 'ENTER_SAFE_MODE', 'reason', reason);
        json_cmd = jsonencode(command);
        write(udp_sender, json_cmd, "char", "127.0.0.1", 10001);
    end
end
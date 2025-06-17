function run_fdir_cycle()
% RUN_FDIR_CYCLE - Performs a single check cycle of the FDIR algorithm.
% This function is designed to be called repeatedly by a MATLAB timer.
% It monitors the global 'carla_outputs' struct and makes decisions.

    % --- State Tracking ---
    % 'persistent' ensures this variable retains its value across calls.
    persistent sensor_states;
    if isempty(sensor_states)
        fprintf('FDIR System Initialized. Monitoring CARLA outputs...\n');
        sensor_states = containers.Map('KeyType', 'char', 'ValueType', 'char');
    end

    % Access the global data structure populated by the receiver
    global carla_outputs;

    % Check if the data structure exists and is populated
    if ~isstruct(carla_outputs) || isempty(fieldnames(carla_outputs))
        % No data yet, do nothing for this cycle.
        return;
    end

    % --- Decision Logic ---
    % Check for a fallback trigger from the main analysis function.
    if isfield(carla_outputs, 'fallback_initiation') && carla_outputs.fallback_initiation

        % Isolate the fault. Prioritize non-recoverable faults first.
        
        % --- PRIORITY 1: CRITICAL, NON-RECOVERABLE FAULTS ---
        is_critical_fault = false;
        if carla_outputs.sensor_fusion_status.position_uncertainty > 10
            is_critical_fault = true;
            reason = "Critical system failure: High uncertainty (>10).";
        elseif isfield(carla_outputs.processed_sensor_data, 'is_collision_event') && carla_outputs.processed_sensor_data.is_collision_event
            is_critical_fault = true;
            reason = "Critical system failure: Collision detected.";
        end

        if is_critical_fault
            command_switch_to_safe_mode(reason);
            
            % Find and stop the timer to halt the FDIR system.
            fdir_timer = timerfind('Tag', 'FDIR_Timer');
            if ~isempty(fdir_timer)
                stop(fdir_timer);
                fprintf('CRITICAL FAULT DETECTED. FDIR SYSTEM HALTED.\n');
            end
            return; % Stop further processing in this cycle
        end

        % --- PRIORITY 2: RECOVERABLE SENSOR FAULTS ---
        if carla_outputs.sensor_fusion_status.health_score < 0.9 % A less strict threshold
            raw_health = carla_outputs.sensor_fusion_status.raw_health;
            sensor_groups = fieldnames(raw_health);
            
            found_fault = false;
            for i = 1:numel(sensor_groups)
                group_name = sensor_groups{i};
                statuses = raw_health.(group_name);
                
                % Check if the primary sensor is down ('OK' is good)
                if ~strcmp(statuses{1}, 'OK')
                    current_state = get_sensor_state(sensor_states, group_name);
                    
                    switch current_state
                        case 'Nominal'
                            command_recover_sensor(group_name, 1);
                            sensor_states(group_name) = 'Degraded_1';
                            found_fault = true;
                        case 'Degraded_1'
                            command_recover_sensor(group_name, 2);
                            sensor_states(group_name) = 'Degraded_2';
                            found_fault = true;
                        case 'Degraded_2'
                            command_switch_to_safe_mode(sprintf("Unrecoverable failure in sensor group '%s'. All backups exhausted.", group_name));
                            sensor_states(group_name) = 'Failed';
                            found_fault = true;
                        case 'Failed'
                            % Already handled, do nothing.
                    end
                    
                    if found_fault, break; end % Handle one fault per cycle
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

function command_recover_sensor(sensor_name, backup_level)
    fprintf('[FDIR COMMAND]: Recovering sensor group ''%s''. Activating backup #%d.\n', sensor_name, backup_level);
end

function command_switch_to_safe_mode(reason)
    fprintf('\n====================================================\n');
    fprintf('!!! FDIR CRITICAL ACTION: SWITCH TO SAFE MODE !!!\n');
    fprintf('!!! REASON: %s\n', reason);
    fprintf('====================================================\n');
    beep; pause(0.1); beep;
end
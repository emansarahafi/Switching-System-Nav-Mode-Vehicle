function run_fdir_cycle(command_sender)
% RUN_FDIR_CYCLE - Performs a simplified, robust fault detection cycle.
% This function is called by a timer and monitors the global 'carla_outputs'
% struct for critical system-level faults.
%
% This version:
% 1. Detects faults based on latency and system health.
% 2. Commands the vehicle to enter a safe mode upon fault detection.
% 3. Controls the 'fallback_initiation' flag for UI display.

    % --- Configuration: Fault Thresholds ---
    LATENCY_THRESHOLD_S = 0.2;  % 200 milliseconds
    HEALTH_THRESHOLD = 0.75;    % 75% health

    global carla_outputs;

    % Guard clause: Do nothing if the global outputs haven't been populated yet.
    if ~isstruct(carla_outputs) || isempty(fieldnames(carla_outputs))
        return;
    end
    
    % --- Fault Detection Logic ---
    
    % 1. Check for high network latency
    if isfield(carla_outputs, 'network_status') && isfield(carla_outputs.network_status, 'latency')
        if carla_outputs.network_status.latency > LATENCY_THRESHOLD_S
            % FAULT DETECTED: Set the flag and command safe mode.
            carla_outputs.fallback_initiation = true;
            reason = sprintf('High Network Latency detected (%.0fms)', carla_outputs.network_status.latency * 1000);
            command_switch_to_safe_mode(command_sender, reason);
            return; % A fault was found, exit cycle.
        end
    end

    % 2. Check for low overall system health
    if isfield(carla_outputs, 'sensor_fusion_status') && isfield(carla_outputs.sensor_fusion_status, 'health_score')
        if carla_outputs.sensor_fusion_status.health_score < HEALTH_THRESHOLD
            % FAULT DETECTED: Set the flag and command safe mode.
            carla_outputs.fallback_initiation = true;
            reason = sprintf('Low System Health detected (%.1f%%)', carla_outputs.sensor_fusion_status.health_score * 100);
            command_switch_to_safe_mode(command_sender, reason);
            return; % A fault was found, exit cycle.
        end
    end
    
    % --- No Faults Found ---
    % If the function has reached this point, no critical faults were detected
    % in this cycle. We must reset the flag to indicate a nominal state.
    carla_outputs.fallback_initiation = false;
end

%% --- Helper Function for FDIR Action ---

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
function [decision, desirability_score] = fuzzy_bbna(carla_outputs, current_mode)
% FUZZY_BBNA - Decides control mode using a robust fuzzy logic system.
% This function implements a production-grade Behavior-Based Navigation
% Algorithm (BBNA) decider with explicit state management for safe handovers.
%
% Syntax: [decision, desirability_score] = fuzzy_bbna(carla_outputs, current_mode)
%
% Inputs:
%   carla_outputs - A struct containing all processed data from the CARLA
%                   simulation environment.
%   current_mode  - A string for the current control state. Must be one of:
%                   'AUTOPILOT', 'MANUAL', 'REQUEST_HANDOVER'.
%
% Outputs:
%   decision      - The new recommended state. One of:
%                   'AUTOPILOT': Conditions are safe for autonomous control.
%                   'MANUAL': System should be in manual mode.
%                   'REQUEST_HANDOVER': A transition from Autopilot to Manual
%                                       is required. The calling system is
%                                       responsible for alerting the driver
%                                       and managing a timeout to a Minimal
%                                       Risk Condition (e.g., pulling over).
%   desirability_score - The raw numeric output [0-100] from the FIS.

% --- Configuration: Hysteresis Thresholds ---
THRESH_ACTIVATE_AP = 65;   % Score must be ABOVE this to switch TO Autopilot.
THRESH_DEACTIVATE_AP = 35; % Score must be BELOW this to switch FROM Autopilot.

% --- Create or Load the Fuzzy Inference System (FIS) ---
persistent fis;
if isempty(fis)
    disp('Creating and caching BBNA FIS for the first time...');
    fis = create_bbna_fis();
end

% --- Pre-process CARLA data into fuzzy inputs ---
inputs = preprocess_carla_data(carla_outputs);

% --- Evaluate the FIS ---
desirability_score = evalfis(fis, inputs);

% --- Make a Crisp Decision with Hysteresis and State Management ---
decision = current_mode; % Default to staying in the current mode

switch current_mode
    case 'AUTOPILOT'
        % If in Autopilot, we only look for reasons to disengage.
        if desirability_score < THRESH_DEACTIVATE_AP
            decision = 'MANUAL'; % Directly transition to manual
        end
        
    case 'MANUAL'
        % If in Manual, we only look for opportunities to engage Autopilot.
        if desirability_score > THRESH_ACTIVATE_AP
            decision = 'AUTOPILOT';
        end
        
    case 'REQUEST_HANDOVER' % This state is now handled by the transition logic
        % If a handover is already in progress, it can only be cancelled
        % if conditions become exceptionally good again.
        if desirability_score > THRESH_ACTIVATE_AP
            decision = 'AUTOPILOT'; % The danger has passed; cancel handover.
        else
            decision = 'MANUAL'; % Continue the transition to manual
        end
end

end

%% =======================================================================
%                     FIS CREATION
% ========================================================================
function fis = create_bbna_fis()
    fis = mamfis('Name', 'BBNA_Mode_Switcher_V2');

    fis = addInput(fis, [0 1], 'Name', 'SystemHealth');
    fis = addInput(fis, [0 1], 'Name', 'EnvironmentalComplexity');
    fis = addInput(fis, [0 1], 'Name', 'DriverState');
    fis = addInput(fis, [0 1], 'Name', 'CriticalEvent');

    fis = addMF(fis, 'SystemHealth', 'trapmf', [0 0 0.2 0.4], 'Name', 'Poor');
    fis = addMF(fis, 'SystemHealth', 'trimf', [0.3 0.5 0.7], 'Name', 'Degraded');
    fis = addMF(fis, 'SystemHealth', 'trapmf', [0.6 0.8 1 1], 'Name', 'Good');
    fis = addMF(fis, 'EnvironmentalComplexity', 'trapmf', [0 0 0.2 0.4], 'Name', 'Low');
    fis = addMF(fis, 'EnvironmentalComplexity', 'trimf', [0.3 0.5 0.7], 'Name', 'Moderate');
    fis = addMF(fis, 'EnvironmentalComplexity', 'trapmf', [0.6 0.8 1 1], 'Name', 'High');
    fis = addMF(fis, 'DriverState', 'trapmf', [0 0 0.2 0.4], 'Name', 'Unready');
    fis = addMF(fis, 'DriverState', 'trimf', [0.3 0.6 0.9], 'Name', 'Attentive');
    fis = addMF(fis, 'DriverState', 'trapmf', [0.7 0.9 1 1], 'Name', 'Ready');
    fis = addMF(fis, 'CriticalEvent', 'trapmf', [0 0 0.05 0.1], 'Name', 'None');
    fis = addMF(fis, 'CriticalEvent', 'trapmf', [0.9 0.95 1 1], 'Name', 'Occurred');
    
    fis = addOutput(fis, [0 100], 'Name', 'AutopilotDesirability');
    fis = addMF(fis, 'AutopilotDesirability', 'trapmf', [0 0 20 35], 'Name', 'Manual');
    fis = addMF(fis, 'AutopilotDesirability', 'trimf', [30 50 70], 'Name', 'Handover');
    fis = addMF(fis, 'AutopilotDesirability', 'trapmf', [65 80 100 100], 'Name', 'Autopilot');

    rules = [
        0 0 0 2  1 1.0 1;
        1 0 0 0  1 1.0 1;
        3 0 1 1  3 0.9 1;
        0 3 0 1  2 0.8 1;
        3 0 3 1  1 0.9 1;
        3 1 0 1  3 0.7 1;
        2 2 0 1  2 0.7 1;
        2 1 0 1  2 0.6 1;
        2 0 1 1  2 0.8 1;
    ];
    fis = addRule(fis, rules);
end

%% =======================================================================
%                     DATA PRE-PROCESSING
% ========================================================================
function processed_inputs = preprocess_carla_data(outputs)
    required_fields = {'sensor_fusion_status', 'processed_sensor_data', 'fallback_initiation', 'driver_attention', 'driver_readiness'};
    for i = 1:length(required_fields)
        if ~isfield(outputs, required_fields{i})
            error('FuzzyBBNA:MissingInput', 'Missing required input field: %s', required_fields{i});
        end
    end

    MAX_UNCERTAINTY_TRACE = 10.0;
    MAX_OBSTACLE_DENSITY = 20.0;
    
    health_score = outputs.sensor_fusion_status.health_score;
    uncertainty = outputs.sensor_fusion_status.position_uncertainty;
    uncertainty_penalty = min(1, uncertainty / MAX_UNCERTAINTY_TRACE);
    SystemHealth = health_score * (1 - uncertainty_penalty);

    weather_severity = outputs.processed_sensor_data.Weather_Severity;
    obstacle_density_raw = outputs.processed_sensor_data.Obstacle_Density;
    obstacle_density_norm = min(1, obstacle_density_raw / MAX_OBSTACLE_DENSITY);
    EnvironmentalComplexity = (0.6 * weather_severity) + (0.4 * obstacle_density_norm);
    
    attention = outputs.driver_attention;
    readiness = outputs.driver_readiness;
    DriverState = attention * readiness;
    
    is_fallback = outputs.fallback_initiation;
    is_collision = outputs.processed_sensor_data.is_collision_event;
    CriticalEvent = double(is_fallback || is_collision);

    processed_inputs = [SystemHealth, EnvironmentalComplexity, DriverState, CriticalEvent];
    processed_inputs = max(0, min(1, processed_inputs));
end
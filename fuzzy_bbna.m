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
% The 'persistent' keyword ensures the FIS is created only once, improving performance.
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
            decision = 'REQUEST_HANDOVER';
        end
        
    case 'MANUAL'
        % If in Manual, we only look for opportunities to engage Autopilot.
        if desirability_score > THRESH_ACTIVATE_AP
            decision = 'AUTOPILOT';
        end
        
    case 'REQUEST_HANDOVER'
        % If a handover is already in progress, it can only be cancelled
        % if conditions become exceptionally good again. Otherwise, the
        % handover to Manual control continues.
        if desirability_score > THRESH_ACTIVATE_AP
            decision = 'AUTOPILOT'; % The danger has passed; cancel handover.
        end
        % Otherwise, decision remains 'REQUEST_HANDOVER' until the driver
        % takes control (which the parent script would manage).
end

fprintf('Fuzzy BBNA: Current=%-16s | Score=%5.2f | New Decision=%-16s\n', ...
        current_mode, desirability_score, decision);

end


%% =======================================================================
%                     FIS CREATION
% ========================================================================
function fis = create_bbna_fis()
    % Defines the structure, MFs, and rules for the FIS with rule weighting.
    fis = mamfis('Name', 'BBNA_Mode_Switcher_V2');

    % --- Inputs ---
    fis = addInput(fis, [0 1], 'Name', 'SystemHealth');
    fis = addInput(fis, [0 1], 'Name', 'EnvironmentalComplexity');
    fis = addInput(fis, [0 1], 'Name', 'DriverState');
    fis = addInput(fis, [0 1], 'Name', 'CriticalEvent');

    % --- Input MFs (Unchanged) ---
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
    
    % --- Output and its MFs (Unchanged) ---
    fis = addOutput(fis, [0 100], 'Name', 'AutopilotDesirability');
    fis = addMF(fis, 'AutopilotDesirability', 'trapmf', [0 0 20 35], 'Name', 'Manual');
    fis = addMF(fis, 'AutopilotDesirability', 'trimf', [30 50 70], 'Name', 'Handover');
    fis = addMF(fis, 'AutopilotDesirability', 'trapmf', [65 80 100 100], 'Name', 'Autopilot');

    % --- Rules with Weights ---
    % Rule format: [SysHealth EnvComplex DriverState CriticalEvent -> Output (Weight) (AND=1/OR=2)]
    rules = [
        % Critical Overrides (highest priority, max weight)
        0 0 0 2  1 1.0 1;  % IF (CriticalEvent is Occurred) THEN (Manual)
        1 0 0 0  1 1.0 1;  % IF (SystemHealth is Poor) THEN (Manual)

        % Key Safety-driven Rules (high weight)
        3 0 1 1  3 0.9 1;  % IF (SysHealth is Good) AND (Driver is Unready) THEN (Autopilot)
        0 3 0 1  2 0.8 1;  % IF (EnvComplexity is High) THEN (Handover)
        
        % Driver Intent Rule (high weight)
        3 0 3 1  1 0.9 1;  % IF (SysHealth Good) AND (Driver Ready) THEN (Manual) - assuming 'Ready' implies intent to drive

        % Standard operational rules (medium weight)
        3 1 0 1  3 0.7 1;  % IF (SysHealth Good) AND (Env Low) THEN (Autopilot)
        2 2 0 1  2 0.7 1;  % IF (SysHealth Degraded) AND (Env Moderate) THEN (Handover)
        2 1 0 1  2 0.6 1;  % IF (SysHealth Degraded) AND (Env Low) THEN (Handover)
        
        % Redundant rule for Degraded Health with Unready driver (already covered but good to be explicit)
        2 0 1 1  2 0.8 1;  % IF (SysHealth Degraded) AND (Driver Unready) THEN (Handover)
    ];

    fis = addRule(fis, rules);
end


%% =======================================================================
%                     DATA PRE-PROCESSING
% ========================================================================
function processed_inputs = preprocess_carla_data(outputs)
    % Extracts, validates, and calculates the four key metrics for the FIS.
    
    % --- 1. Input Validation ---
    required_fields = {
        'sensor_fusion_status', 'processed_sensor_data', ...
        'fallback_initiation', 'driver_attention', 'driver_readiness'
    };
    for i = 1:length(required_fields)
        if ~isfield(outputs, required_fields{i})
            error('FuzzyBBNA:MissingInput', 'Missing required input field: %s', required_fields{i});
        end
    end

    % --- 2. Configuration Constants for Normalization ---
    MAX_UNCERTAINTY_TRACE = 10.0; % Kalman filter trace > 10 is very bad.
    MAX_OBSTACLE_DENSITY = 20.0;  % >20 nearby obstacles is max density.
    
    % --- 3. Calculate Metrics ---
    
    % SystemHealth [0-1]
    health_score = outputs.sensor_fusion_status.health_score;
    uncertainty = outputs.sensor_fusion_status.position_uncertainty;
    uncertainty_penalty = min(1, uncertainty / MAX_UNCERTAINTY_TRACE);
    SystemHealth = health_score * (1 - uncertainty_penalty);

    % EnvironmentalComplexity [0-1]
    weather_severity = outputs.processed_sensor_data.Weather_Severity;
    obstacle_density_raw = outputs.processed_sensor_data.Obstacle_Density;
    obstacle_density_norm = min(1, obstacle_density_raw / MAX_OBSTACLE_DENSITY);
    EnvironmentalComplexity = (0.6 * weather_severity) + (0.4 * obstacle_density_norm);
    
    % DriverState [0-1]
    attention = outputs.driver_attention;
    readiness = outputs.driver_readiness;
    DriverState = attention * readiness;
    
    % CriticalEvent [0-1]
    is_fallback = outputs.fallback_initiation;
    is_collision = outputs.processed_sensor_data.is_collision_event;
    CriticalEvent = double(is_fallback || is_collision);

    % --- 4. Assemble final input vector ---
    processed_inputs = [SystemHealth, EnvironmentalComplexity, DriverState, CriticalEvent];
    % Clamp values to ensure they are within the [0, 1] range for the FIS
    processed_inputs = max(0, min(1, processed_inputs));
end
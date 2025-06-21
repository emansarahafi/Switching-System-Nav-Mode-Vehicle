function [decision, desirability_score] = fuzzy_bbna(carla_outputs, current_mode)
% FUZZY_BBNA - Decides control mode using a refined fuzzy logic system.
% This version is focused on behavioral decision-making, with FDIR (fault
% detection) handled by a separate module. It incorporates V2X communication
% data and features a re-balanced rule set to improve autopilot engagement.
%
% Syntax: [decision, desirability_score] = fuzzy_bbna(carla_outputs, current_mode)
%
% Inputs:
%   carla_outputs - A struct containing all processed data from the CARLA
%                   simulation environment.
%   current_mode  - A string for the current control state. Must be one of:
%                   'AUTOPILOT', 'MANUAL'.
%
% Outputs:
%   decision      - The new recommended state. One of:
%                   'AUTOPILOT': Conditions are safe for autonomous control.
%                   'MANUAL': Manual control is required.
%   desirability_score - The raw numeric output [0-100] from the FIS, where
%                        higher scores indicate greater suitability for autopilot.

% --- Configuration: Hysteresis Thresholds for robust switching ---
THRESH_ACTIVATE_AP = 65;   % Score must be ABOVE this to switch TO Autopilot.
THRESH_DEACTIVATE_AP = 35; % Score must be BELOW this to switch FROM Autopilot.

% --- Create or Load the Fuzzy Inference System (FIS) ---
persistent fis;
if isempty(fis)
    disp('Creating and caching refined BBNA FIS (V3) for the first time...');
    fis = create_bbna_fis_v3();
end

% --- Pre-process CARLA data into fuzzy inputs ---
inputs = preprocess_carla_data(carla_outputs);

% --- Evaluate the FIS ---
desirability_score = evalfis(fis, inputs);

% --- Make a Crisp Decision with Hysteresis and State Management ---
decision = current_mode; % Default to staying in the current mode

switch current_mode
    case 'AUTOPILOT'
        % If in Autopilot, only disengage if conditions become poor.
        if desirability_score < THRESH_DEACTIVATE_AP
            decision = 'MANUAL'; % Conditions are no longer safe for Autopilot.
        end
        
    case 'MANUAL'
        % If in Manual, only engage Autopilot if conditions are very good.
        if desirability_score > THRESH_ACTIVATE_AP
            decision = 'AUTOPILOT';
        end
        
    % NOTE: 'REQUEST_HANDOVER' is not a state handled here. It is an action
    % that the calling system should perform when the decision switches from
    % AUTOPILOT to MANUAL.
end

end

%% =======================================================================
%                     FIS CREATION (V3 - REFINED)
% ========================================================================
function fis = create_bbna_fis_v3()
    fis = mamfis('Name', 'BBNA_Mode_Switcher_V3');

    % --- Define Inputs ---
    % FDIR input 'SystemHealth' is removed.
    fis = addInput(fis, [0 1], 'Name', 'EnvironmentalComplexity');
    fis = addInput(fis, [0 1], 'Name', 'DriverState');
    fis = addInput(fis, [0 1], 'Name', 'CriticalEvent');
    fis = addInput(fis, [0 1], 'Name', 'CommunicationStatus'); % NEW: V2X input

    % --- Define Input Membership Functions ---
    fis = addMF(fis, 'EnvironmentalComplexity', 'trapmf', [0 0 0.2 0.4], 'Name', 'Low');
    fis = addMF(fis, 'EnvironmentalComplexity', 'trimf', [0.3 0.5 0.7], 'Name', 'Moderate');
    fis = addMF(fis, 'EnvironmentalComplexity', 'trapmf', [0.6 0.8 1 1], 'Name', 'High');

    fis = addMF(fis, 'DriverState', 'trapmf', [0 0 0.2 0.4], 'Name', 'Unready');
    fis = addMF(fis, 'DriverState', 'trimf', [0.3 0.6 0.9], 'Name', 'Attentive');
    fis = addMF(fis, 'DriverState', 'trapmf', [0.7 0.9 1 1], 'Name', 'Ready');

    fis = addMF(fis, 'CriticalEvent', 'trapmf', [0 0 0.05 0.1], 'Name', 'None');
    fis = addMF(fis, 'CriticalEvent', 'trapmf', [0.9 0.95 1 1], 'Name', 'Occurred');
    
    fis = addMF(fis, 'CommunicationStatus', 'trapmf', [0 0 0.3 0.5], 'Name', 'Poor');
    fis = addMF(fis, 'CommunicationStatus', 'trapmf', [0.5 0.7 1 1], 'Name', 'Good');

    % --- Define Output and its Membership Functions ---
    fis = addOutput(fis, [0 100], 'Name', 'AutopilotDesirability');
    fis = addMF(fis, 'AutopilotDesirability', 'trapmf', [0 0 20 35], 'Name', 'Manual');
    fis = addMF(fis, 'AutopilotDesirability', 'trimf', [30 50 70], 'Name', 'Handover');
    fis = addMF(fis, 'AutopilotDesirability', 'trapmf', [65 80 100 100], 'Name', 'Autopilot');

    % --- Define Rules (Rewritten for better performance and V2X) ---
    % This new rule set is more balanced and provides clear paths to Autopilot.
    % Inputs: 1:Env, 2:Driver, 3:Critical, 4:Comm
    % Output: 1:Manual, 2:Handover, 3:Autopilot
    rules = [
    % Env Drv Crit Comm -> Out | Wght | Conn (1=AND)
    % --- Rules for immediate disengagement ---
      0   0   2   0      1      1.0    1;  % Rule 1: A critical event (collision) occurred -> Manual
      3   0   1   0      1      1.0    1;  % Rule 2: Environment is highly complex -> Manual
    
    % --- Rules for caution (suggests handover/maintaining manual) ---
      0   1   1   0      2      0.9    1;  % Rule 3: Driver is unready/distracted -> Handover
      0   0   1   1      2      0.8    1;  % Rule 4: Comms are poor (e.g., red light) -> Handover
      2   2   1   2      2      0.7    1;  % Rule 5: Moderate Env, merely Attentive Driver -> Handover

    % --- Rules for engaging/maintaining Autopilot ---
      1   3   1   2      3      1.0    1;  % Rule 6: Ideal: Low Env, Ready Driver, Good Comms -> Autopilot
      1   2   1   2      3      0.9    1;  % Rule 7: Good: Low Env, Attentive Driver, Good Comms -> Autopilot
      2   3   1   2      3      0.8    1;  % Rule 8: OK: Moderate Env, Ready Driver, Good Comms -> Autopilot
    ];
    fis = addRule(fis, rules);
end

%% =======================================================================
%               DATA PRE-PROCESSING (V3 - REFINED)
% ========================================================================
function processed_inputs = preprocess_carla_data(outputs)
    % Ensure necessary data fields are available
    required_fields = {'processed_sensor_data', 'driver_attention', 'driver_readiness'};
    for i = 1:length(required_fields)
        if ~isfield(outputs, required_fields{i})
            error('FuzzyBBNA:MissingInput', 'Missing required input field: %s', required_fields{i});
        end
    end

    % --- 1. Environmental Complexity ---
    MAX_OBSTACLE_DENSITY = 20.0; % Max obstacles in 50m radius to consider for normalization
    weather_severity = outputs.processed_sensor_data.Weather_Severity;
    obstacle_density_raw = outputs.processed_sensor_data.Obstacle_Density;
    obstacle_density_norm = min(1, obstacle_density_raw / MAX_OBSTACLE_DENSITY);
    EnvironmentalComplexity = (0.6 * weather_severity) + (0.4 * obstacle_density_norm);
    
    % --- 2. Driver State ---
    attention = outputs.driver_attention;
    readiness = outputs.driver_readiness;
    DriverState = attention * readiness;
    
    % --- 3. Critical Event ---
    % Note: FDIR 'fallback' is removed. This now only considers collision events.
    is_collision = outputs.processed_sensor_data.is_collision_event;
    CriticalEvent = double(is_collision);

    % --- 4. Communication Status (V2X) ---
    comm_score = 0.7; % Start with a neutral-to-good baseline
    
    % Process V2I (Infrastructure) data
    v2i_data = get_safe(outputs.processed_sensor_data, 'V2I_Data', struct());
    if isfield(v2i_data, 'traffic_light_state') && ~isempty(v2i_data.traffic_light_state)
        switch upper(v2i_data.traffic_light_state)
            case 'RED'
                comm_score = 0.1; % Red light is a hard stop condition, very poor for AP desirability
            case 'YELLOW'
                comm_score = 0.4; % Yellow light requires caution
            case 'GREEN'
                comm_score = 1.0; % Green light is a positive signal
        end
    end
    
    % Process V2V (Vehicle) data
    v2v_data = get_safe(outputs.processed_sensor_data, 'V2V_Data', []);
    if ~isempty(v2v_data)
        for i = 1:numel(v2v_data)
            % Check for a hypothetical 'emergency_brake' flag from other vehicles.
            % This would need to be added to the data sent from Python.
            if isfield(v2v_data(i), 'emergency_brake') && v2v_data(i).emergency_brake
                comm_score = 0.0; % Critical V2V message, forces lowest score
                break;
            end
        end
    end
    CommunicationStatus = comm_score;

    % --- Assemble and clamp final input vector ---
    processed_inputs = [EnvironmentalComplexity, DriverState, CriticalEvent, CommunicationStatus];
    processed_inputs = max(0, min(1, processed_inputs));
end
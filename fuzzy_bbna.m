function [decision, desirability_score] = fuzzy_bbna(carla_outputs, current_mode)
% FUZZY_BBNA - Decides control mode using a refined fuzzy logic system.
% This version (V4) adds System Latency and Lane Keeping analysis to the
% decision matrix, making it more sensitive to system performance and
% vehicle control stability.
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
    disp('Creating and caching refined BBNA FIS (V4) for the first time...');
    fis = create_bbna_fis_v4();
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
%               FIS CREATION (V4 - LATENCY & LANE KEEPING)
% ========================================================================
function fis = create_bbna_fis_v4()
    fis = mamfis('Name', 'BBNA_Mode_Switcher_V4');

    % --- Define Inputs ---
    fis = addInput(fis, [0 1], 'Name', 'EnvironmentalComplexity');
    fis = addInput(fis, [0 1], 'Name', 'DriverState');
    fis = addInput(fis, [0 1], 'Name', 'CriticalEvent');
    fis = addInput(fis, [0 1], 'Name', 'CommunicationStatus');
    fis = addInput(fis, [0 1], 'Name', 'SystemLatency');       % NEW: Latency input
    fis = addInput(fis, [0 1], 'Name', 'LaneKeeping');         % NEW: Lane invasion input

    % --- Define Input Membership Functions ---
    % EnvironmentalComplexity
    fis = addMF(fis, 'EnvironmentalComplexity', 'trapmf', [0 0 0.2 0.4], 'Name', 'Low');
    fis = addMF(fis, 'EnvironmentalComplexity', 'trimf', [0.3 0.5 0.7], 'Name', 'Moderate');
    fis = addMF(fis, 'EnvironmentalComplexity', 'trapmf', [0.6 0.8 1 1], 'Name', 'High');
    % DriverState
    fis = addMF(fis, 'DriverState', 'trapmf', [0 0 0.2 0.4], 'Name', 'Unready');
    fis = addMF(fis, 'DriverState', 'trimf', [0.3 0.6 0.9], 'Name', 'Attentive');
    fis = addMF(fis, 'DriverState', 'trapmf', [0.7 0.9 1 1], 'Name', 'Ready');
    % CriticalEvent
    fis = addMF(fis, 'CriticalEvent', 'trapmf', [0 0 0.05 0.1], 'Name', 'None');
    fis = addMF(fis, 'CriticalEvent', 'trapmf', [0.9 0.95 1 1], 'Name', 'Occurred');
    % CommunicationStatus
    fis = addMF(fis, 'CommunicationStatus', 'trapmf', [0 0 0.3 0.5], 'Name', 'Poor');
    fis = addMF(fis, 'CommunicationStatus', 'trapmf', [0.5 0.7 1 1], 'Name', 'Good');
    % NEW: SystemLatency
    fis = addMF(fis, 'SystemLatency', 'trapmf', [0 0 0.1 0.2], 'Name', 'Low');    % e.g., < 100ms
    fis = addMF(fis, 'SystemLatency', 'trimf', [0.15 0.3 0.45], 'Name', 'Moderate'); % e.g., 75-225ms
    fis = addMF(fis, 'SystemLatency', 'trapmf', [0.4 0.6 1 1], 'Name', 'High');    % e.g., > 200ms
    % NEW: LaneKeeping
    fis = addMF(fis, 'LaneKeeping', 'trapmf', [0 0 0.1 0.2], 'Name', 'OK');
    fis = addMF(fis, 'LaneKeeping', 'trapmf', [0.8 0.9 1 1], 'Name', 'Drifting');

    % --- Define Output and its Membership Functions ---
    fis = addOutput(fis, [0 100], 'Name', 'AutopilotDesirability');
    fis = addMF(fis, 'AutopilotDesirability', 'trapmf', [0 0 20 35], 'Name', 'Manual');
    fis = addMF(fis, 'AutopilotDesirability', 'trimf', [30 50 70], 'Name', 'Handover');
    fis = addMF(fis, 'AutopilotDesirability', 'trapmf', [65 80 100 100], 'Name', 'Autopilot');

    % --- Define Rules (Rewritten for Latency and Lane Keeping) ---
    % Inputs: 1:Env, 2:Driver, 3:Critical, 4:Comm, 5:Latency, 6:LaneKeep
    % Output: 1:Manual, 2:Handover, 3:Autopilot
    % A '0' in a rule means "don't care" for that input.
    rules = [
    % Env Drv Crit Comm Lat  Lane -> Out | Wght | Conn (1=AND)
    % --- High-priority rules for immediate disengagement (Manual) ---
      0   0   2   0    0    0       1      1.0    1;  % Rule 1: A critical event (collision) occurred
      0   0   0   0    0    2       1      1.0    1;  % Rule 2: A lane invasion occurred (Drifting)
      0   0   0   0    3    0       1      1.0    1;  % Rule 3: System latency is High
      3   0   1   0    0    0       1      1.0    1;  % Rule 4: Environment is highly complex
    
    % --- Rules for caution (suggests Handover/maintaining Manual) ---
      0   1   1   0    0    0       2      0.9    1;  % Rule 5: Driver is unready/distracted
      0   0   1   1    0    0       2      0.8    1;  % Rule 6: Comms are poor (e.g., red light)
      0   0   1   2    2    1       2      0.8    1;  % Rule 7: Comms are good, but latency is Moderate
      2   2   1   2    0    1       2      0.7    1;  % Rule 8: Moderate Env, merely Attentive Driver

    % --- Rules for engaging/maintaining Autopilot (now more strict) ---
      1   3   1   2    1    1       3      1.0    1;  % Rule 9: Ideal: Low Env, Ready Driver, Good Comms, Low Latency, OK LaneKeeping
      1   2   1   2    1    1       3      0.9    1;  % Rule 10: Good: Low Env, Attentive Driver, Good Comms, Low Latency, OK LaneKeeping
      2   3   1   2    1    1       3      0.8    1;  % Rule 11: OK: Moderate Env, Ready Driver, Good Comms, Low Latency, OK LaneKeeping
    ];
    fis = addRule(fis, rules);
end

%% =======================================================================
%               DATA PRE-PROCESSING (V4 - LATENCY & LANE KEEPING)
% ========================================================================
function processed_inputs = preprocess_carla_data(outputs)
    % Ensure necessary data fields are available
    required_fields = {'processed_sensor_data', 'driver_attention', 'driver_readiness', 'network_status'};
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
    is_collision = outputs.processed_sensor_data.is_collision_event;
    CriticalEvent = double(is_collision);

    % --- 4. Communication Status (V2X) ---
    comm_score = 0.7; % Neutral-to-good baseline
    v2i_data = get_safe(outputs.processed_sensor_data, 'V2I_Data', struct());
    if isfield(v2i_data, 'traffic_light_state') && ~isempty(v2i_data.traffic_light_state)
        switch upper(v2i_data.traffic_light_state)
            case 'RED',    comm_score = 0.1;
            case 'YELLOW', comm_score = 0.4;
            case 'GREEN',  comm_score = 1.0;
        end
    end
    v2v_data = get_safe(outputs.processed_sensor_data, 'V2V_Data', []);
    if ~isempty(v2v_data)
        for i = 1:numel(v2v_data)
            if isfield(v2v_data(i), 'emergency_brake') && v2v_data(i).emergency_brake
                comm_score = 0.0; break;
            end
        end
    end
    CommunicationStatus = comm_score;
    
    % --- 5. NEW: System Latency ---
    % Normalize latency. Assume anything > 500ms is max (worst) score.
    MAX_LATENCY_SECONDS = 0.5; 
    latency_sec = get_safe(outputs.network_status, 'latency', 0);
    SystemLatency = min(1.0, latency_sec / MAX_LATENCY_SECONDS);
    
    % --- 6. NEW: Lane Keeping ---
    % Convert the boolean lane invasion event into a numeric value [0, 1].
    % 0 = OK, 1 = Drifting.
    is_drifting = get_safe(outputs.processed_sensor_data, 'is_lane_invasion_event', false);
    LaneKeeping = double(is_drifting);

    % --- Assemble and clamp final input vector ---
    processed_inputs = [EnvironmentalComplexity, DriverState, CriticalEvent, ...
                        CommunicationStatus, SystemLatency, LaneKeeping];
    processed_inputs = max(0, min(1, processed_inputs));
end
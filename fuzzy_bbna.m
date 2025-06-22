function [decision, desirability_score] = fuzzy_bbna(carla_outputs, current_mode)
% FUZZY_BBNA - Decides control mode using a refined fuzzy logic system.
% ## FINAL CORRECTED VERSION ##
% This version fixes the state-flicker issue by making the fuzzy inputs
% dependent on the current control mode.

% --- Configuration: Hysteresis Thresholds ---
THRESH_ACTIVATE_AP = 45;
THRESH_DEACTIVATE_AP = 25;

% --- Create or Load the Fuzzy Inference System (FIS) ---
persistent fis;
if isempty(fis)
    disp('Creating and caching refined BBNA FIS (V4) for the first time...');
    fis = create_bbna_fis_v4();
end

% --- Pre-process CARLA data into fuzzy inputs ---
% ## BUG FIX ## Pass the current_mode into the pre-processor.
inputs = preprocess_carla_data(carla_outputs, current_mode);

% --- Evaluate the FIS ---
desirability_score = evalfis(fis, inputs);

% --- Make a Crisp Decision with Hysteresis and State Management ---
decision = current_mode; % Default to staying in the current mode

switch current_mode
    case 'AUTOPILOT'
        if desirability_score < THRESH_DEACTIVATE_AP
            decision = 'MANUAL';
        end
        
    case 'MANUAL'
        if desirability_score > THRESH_ACTIVATE_AP
            decision = 'AUTOPILOT';
        end
    
    % For other states like AWAITING_CONFIRMATION, we don't change the decision here.
end

end

%% =======================================================================
%               FIS CREATION (V4 - LATENCY & LANE KEEPING)
% ========================================================================
function fis = create_bbna_fis_v4()
    fis = mamfis('Name', 'BBNA_Mode_Switcher_V4');

    fis = addInput(fis, [0 1], 'Name', 'EnvironmentalComplexity');
    fis = addInput(fis, [0 1], 'Name', 'DriverState');
    fis = addInput(fis, [0 1], 'Name', 'CriticalEvent');
    fis = addInput(fis, [0 1], 'Name', 'CommunicationStatus');
    fis = addInput(fis, [0 1], 'Name', 'SystemLatency');       
    fis = addInput(fis, [0 1], 'Name', 'LaneKeeping');         

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
    fis = addMF(fis, 'SystemLatency', 'trapmf', [0 0 0.1 0.2], 'Name', 'Low');
    fis = addMF(fis, 'SystemLatency', 'trimf', [0.15 0.3 0.45], 'Name', 'Moderate');
    fis = addMF(fis, 'SystemLatency', 'trapmf', [0.4 0.6 1 1], 'Name', 'High');
    fis = addMF(fis, 'LaneKeeping', 'trapmf', [0 0 0.1 0.2], 'Name', 'OK');
    fis = addMF(fis, 'LaneKeeping', 'trapmf', [0.8 0.9 1 1], 'Name', 'Drifting');

    fis = addOutput(fis, [0 100], 'Name', 'AutopilotDesirability');
    fis = addMF(fis, 'AutopilotDesirability', 'trapmf', [0 0 20 35], 'Name', 'Manual');
    fis = addMF(fis, 'AutopilotDesirability', 'trimf', [30 50 70], 'Name', 'Handover');
    fis = addMF(fis, 'AutopilotDesirability', 'trapmf', [65 80 100 100], 'Name', 'Autopilot');

    rules = [
      0   0   2   0    0    0       1      1.0    1;
      0   0   0   0    0    2       1      1.0    1;
      0   0   0   0    3    0       1      1.0    1;
      3   0   1   0    0    0       1      1.0    1;
      0   1   1   0    0    0       2      0.9    1;
      0   0   1   1    0    0       2      0.8    1;
      0   0   1   2    2    1       2      0.8    1;
      2   2   1   2    0    1       2      0.7    1;
      1   3   1   2    1    1       3      1.0    1;
      1   2   1   2    1    1       3      0.9    1;
      2   3   1   2    1    1       3      0.8    1;
    ];
    fis = addRule(fis, rules);
end

%% =======================================================================
%               DATA PRE-PROCESSING (V4 - LATENCY & LANE KEEPING)
% ========================================================================
% ## BUG FIX ## Added current_mode as an input argument
function processed_inputs = preprocess_carla_data(outputs, current_mode)
    required_fields = {'processed_sensor_data', 'driver_attention', 'driver_readiness', 'network_status'};
    for i = 1:length(required_fields)
        if ~isfield(outputs, required_fields{i})
            error('FuzzyBBNA:MissingInput', 'Missing required input field: %s', required_fields{i});
        end
    end

    % --- 1. Environmental Complexity ---
    MAX_OBSTACLE_DENSITY = 20.0;
    weather_severity = outputs.processed_sensor_data.Weather_Severity;
    obstacle_density_raw = outputs.processed_sensor_data.Obstacle_Density;
    obstacle_density_norm = min(1, obstacle_density_raw / MAX_OBSTACLE_DENSITY);
    EnvironmentalComplexity = (0.2 * weather_severity) + (0.8 * obstacle_density_norm);
    
    % --- 2. Driver State ---
    % ## BUG FIX ##
    % This logic now correctly separates the check for entering AP from staying in AP.
    if strcmp(current_mode, 'AUTOPILOT')
        % When in Autopilot, the driver is expected to be passive. We force a
        % perfect score for this input so that the system is only judged on
        % other factors (environment, system health, etc.).
        DriverState = 1.0;
    else
        % When in MANUAL or AWAITING_CONFIRMATION, we must evaluate if the
        % driver is ready to hand over control.
        attention = outputs.driver_attention;
        readiness = outputs.driver_readiness;
        DriverState = (0.7 * readiness) + (0.3 * attention);
    end
    
    % --- 3. Critical Event ---
    is_collision = outputs.processed_sensor_data.is_collision_event;
    CriticalEvent = double(is_collision);

    % --- 4. Communication Status (V2X) ---
    comm_score = 0.7;
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
    
    % --- 5. System Latency ---
    MAX_LATENCY_SECONDS = 0.5; 
    latency_sec = get_safe(outputs.network_status, 'latency', 0);
    SystemLatency = min(1.0, latency_sec / MAX_LATENCY_SECONDS);
    
    % --- 6. Lane Keeping ---
    is_drifting = get_safe(outputs.processed_sensor_data, 'is_lane_invasion_event', false);
    LaneKeeping = double(is_drifting);

    % --- Assemble and clamp final input vector ---
    processed_inputs = [EnvironmentalComplexity, DriverState, CriticalEvent, ...
                        CommunicationStatus, SystemLatency, LaneKeeping];
    processed_inputs = max(0, min(1, processed_inputs));
end
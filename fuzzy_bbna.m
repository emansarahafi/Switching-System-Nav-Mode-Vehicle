function [autopilot_decision, autopilot_desirability] = fuzzy_bbna()
    % Fuzzy BBNA decision maker
    global carla_outputs;
    
    % Initialize output
    autopilot_decision = false;
    autopilot_desirability = 0;
    
    % Check if data is available
    if isempty(carla_outputs) || isempty(carla_outputs.processed_sensor_data)
        return;
    end
    
    % Extract inputs from carla_outputs
    psd = carla_outputs.processed_sensor_data;
    
    % 1. Attention (directly from processed data)
    attention = psd.attention;
    
    % 2. EnvironmentRisk: Map environment to risk score
    lighting_risk = map_lighting(psd.environment.lighting);
    weather_risk = map_weather(psd.environment.weather);
    environment_risk = max(lighting_risk, weather_risk); % Conservative estimate
    
    % 3. SensorHealth: Compute health score
    health_status = psd.sensor_health;
    sensor_fields = fieldnames(health_status);
    ok_count = sum(strcmp(struct2cell(health_status)), 'ok');
    sensor_health = ok_count / numel(sensor_fields);
    
    % 4. PositionUncertainty: Max diagonal of position covariance
    cov_pos = diag(carla_outputs.sensor_fusion_status.ekf_covariance(1:3, 1:3));
    position_uncertainty = min(1, max(cov_pos) / 100); % Scale to [0,1]
    
    % 5. LaneConfidence: Min of front/back cameras
    lane_confidence = min(...
        psd.image_features.front.lane_confidence, ...
        psd.image_features.back.lane_confidence...
    );
    
    % Create FIS (persistent to avoid reinitialization)
    persistent fis;
    if isempty(fis)
        fis = create_bbna_fis();
    end
    
    % Evaluate FIS
    inputs = [attention, environment_risk, sensor_health, position_uncertainty, lane_confidence];
    autopilot_desirability = evalfis(fis, inputs);
    
    % Decision threshold (with hysteresis to prevent chattering)
    persistent last_decision;
    if isempty(last_decision)
        last_decision = false;
    end
    
    if autopilot_desirability >= 0.6
        autopilot_decision = true;
    elseif autopilot_desirability <= 0.4
        autopilot_decision = false;
    else
        autopilot_decision = last_decision; % Maintain previous state
    end
    last_decision = autopilot_decision;
end

%% Helper: Create FIS
function fis = create_bbna_fis()
    fis = mamfis('Name', 'AutopilotDecision');
    
    % Add inputs
    fis = addInput(fis, [0 1], 'Name', 'Attention');
    fis = addMF(fis, 'Attention', 'trimf', [0 0 0.4], 'Name', 'low');
    fis = addMF(fis, 'Attention', 'trimf', [0.3 0.5 0.7], 'Name', 'medium');
    fis = addMF(fis, 'Attention', 'trimf', [0.6 1 1], 'Name', 'high');
    
    fis = addInput(fis, [0 1], 'Name', 'EnvironmentRisk');
    fis = addMF(fis, 'EnvironmentRisk', 'trimf', [0 0 0.3], 'Name', 'low');
    fis = addMF(fis, 'EnvironmentRisk', 'trimf', [0.2 0.5 0.8], 'Name', 'medium');
    fis = addMF(fis, 'EnvironmentRisk', 'trimf', [0.7 1 1], 'Name', 'high');
    
    fis = addInput(fis, [0 1], 'Name', 'SensorHealth');
    fis = addMF(fis, 'SensorHealth', 'trimf', [0 0 0.4], 'Name', 'poor');
    fis = addMF(fis, 'SensorHealth', 'trimf', [0.3 0.5 0.7], 'Name', 'moderate');
    fis = addMF(fis, 'SensorHealth', 'trimf', [0.6 1 1], 'Name', 'good');
    
    fis = addInput(fis, [0 1], 'Name', 'PositionUncertainty');
    fis = addMF(fis, 'PositionUncertainty', 'trimf', [0 0 0.3], 'Name', 'low');
    fis = addMF(fis, 'PositionUncertainty', 'trimf', [0.2 0.5 0.8], 'Name', 'medium');
    fis = addMF(fis, 'PositionUncertainty', 'trimf', [0.7 1 1], 'Name', 'high');
    
    fis = addInput(fis, [0 1], 'Name', 'LaneConfidence');
    fis = addMF(fis, 'LaneConfidence', 'trimf', [0 0 0.4], 'Name', 'low');
    fis = addMF(fis, 'LaneConfidence', 'trimf', [0.3 0.6 0.9], 'Name', 'medium');
    fis = addMF(fis, 'LaneConfidence', 'trimf', [0.8 1 1], 'Name', 'high');
    
    % Add output
    fis = addOutput(fis, [0 1], 'Name', 'AutopilotDecision');
    fis = addMF(fis, 'AutopilotDecision', 'trimf', [0 0 0.4], 'Name', 'manual');
    fis = addMF(fis, 'AutopilotDecision', 'trimf', [0.3 0.5 0.7], 'Name', 'transition');
    fis = addMF(fis, 'AutopilotDecision', 'trimf', [0.6 1 1], 'Name', 'autopilot');
    
    % Define rules
    rules = [
        1  0  0  0  0  1  1  1;
        0  0  0  3  0  1  1  1;
        0  0  0  0  1  1  1  1;
        1  0  3  1  3  3  1  1;
        0  3  3  1  3  3  1  1;
        3  1  0  0  0  1  1  1;
        2  2  2  0  0  2  1  1;
    ];
    fis = addRule(fis, rules);
end

%% Helper: Map lighting/weather to risk
function risk = map_lighting(lighting)
    switch lighting
        case 'night', risk = 0.8;
        case 'unknown', risk = 0.5;
        otherwise, risk = 0.2; % day
    end
end

function risk = map_weather(weather)
    switch weather
        case {'heavy rain', 'thick fog'}, risk = 0.95;
        case {'rain', 'fog'}, risk = 0.7;
        case 'cloudy', risk = 0.3;
        case 'unknown', risk = 0.5;
        otherwise, risk = 0.1; % clear
    end
end
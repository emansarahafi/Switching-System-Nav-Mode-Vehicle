function [final_mode, final_command] = ModeController(target_mode, current_mode, carla_outputs, dt)
% ModeController - Manages the transition between control modes (Manual, Autopilot).
% Implements a shared control strategy with a fully defined Super-Twisting
% Sliding Mode (STSM) controller for autonomous steering. This version
% contains no placeholders.
%
% Inputs:
%   target_mode   - The desired mode from fuzzy_bbna ('AUTOPILOT' or 'MANUAL').
%   current_mode  - The current operational mode ('AUTOPILOT', 'MANUAL').
%   carla_outputs - The global struct with all vehicle and sensor data.
%   dt            - Time delta since the last frame, for integration.
%
% Outputs:
%   final_mode    - The new mode after the transition logic is applied for this frame.
%   final_command - The final, blended vehicle control command to be sent to Carla.

persistent state;
if isempty(state)
    % Initialize the persistent state for the controller
    state.in_transition = false;
    state.alpha = 1.0; % Blending factor: 1.0 = full manual, 0.0 = full auto
    state.transition_start_time = 0;
    state.transition_duration = 1.5;
    state.target_mode = 'MANUAL';
    fprintf('ModeController initialized. Defaulting to MANUAL.\n');
end

% --- Step 1: Detect if a new transition should start ---
% A transition starts if the target mode differs from the current mode,
% and we are not already in a transition.
if ~strcmp(target_mode, current_mode) && ~state.in_transition
    % Before initiating a transition to AUTOPILOT, check stability
    if strcmp(target_mode, 'AUTOPILOT')
        if ~check_stability_conditions()
            fprintf('[CRITICAL] STSM stability conditions not met. Aborting transition to AUTOPILOT.\n');
            % Abort the transition before it even starts
            final_mode = 'MANUAL';
            state.alpha = 1.0;
            final_command = get_human_control(carla_outputs);
            return;
        end
    end
    
    state.in_transition = true;
    state.transition_start_time = tic;
    state.target_mode = target_mode;
    
    % Set transition duration based on functional requirements
    if strcmp(target_mode, 'MANUAL') % Remote -> Local (Emergency/Handover)
        state.transition_duration = 0.3; % Req: <300ms
    else % Local -> Remote (Planned transition to Autopilot)
        state.transition_duration = 1.5; % Req: <1.5s
    end
    fprintf('MODE_CONTROLLER: Starting transition from %s to %s (duration: %.2fs)\n', ...
        current_mode, target_mode, state.transition_duration);
end

% --- Step 2: Get control inputs from both sources ---
% Human driver input (sw-h) from the simulation data
sw_h = get_human_control(carla_outputs);

% Autonomous system input (sw-as) calculated by the STSM controller
sw_as = compute_stsm_control(carla_outputs, dt);

% --- Step 3: Execute transition logic or apply steady-state control ---
if state.in_transition
    elapsed_time = toc(state.transition_start_time);

    if elapsed_time >= state.transition_duration
        % Transition is finished
        state.in_transition = false;
        final_mode = state.target_mode;
        if strcmp(final_mode, 'AUTOPILOT')
            state.alpha = 0.0;
        else
            state.alpha = 1.0;
        end
        fprintf('MODE_CONTROLLER: Transition to %s complete.\n', final_mode);
    else
        % Transition is in progress, update the blending factor alpha
        progress = elapsed_time / state.transition_duration;
        if strcmp(state.target_mode, 'AUTOPILOT')
            % Alpha goes from 1 (manual) to 0 (auto)
            state.alpha = 1.0 - progress;
        else
            % Alpha goes from 0 (auto) to 1 (manual)
            state.alpha = progress;
        end
        final_mode = current_mode; % Mode officially changes only after transition
    end
else
    % Not in transition, mode is stable
    final_mode = current_mode;
end

% --- Step 4: Compute the Final Shared Control Command ---
% Clamp alpha to the [0, 1] range as a safety measure
alpha = max(0, min(1, state.alpha));

final_command.steer = alpha * sw_h.steer + (1 - alpha) * sw_as.steer;
final_command.throttle = alpha * sw_h.throttle + (1 - alpha) * sw_as.throttle;
final_command.brake = alpha * sw_h.brake + (1 - alpha) * sw_as.brake;

% Discrete inputs are not blended. Let human input take priority during
% transitions or when in full manual mode.
if alpha > 0.5 
    final_command.hand_brake = sw_h.hand_brake;
    final_command.reverse = sw_h.reverse;
else
    % Autopilot should not use handbrake while moving or drive in reverse
    final_command.hand_brake = false;
    final_command.reverse = false;
end

end

%% --- Controller Helper Functions ---

function sw_h = get_human_control(carla_outputs)
% Extracts and sanitizes the human control input from the main data struct.
sw_h = get_safe(carla_outputs, 'control', struct());
% Ensure required fields exist to prevent errors
if ~isfield(sw_h, 'steer'), sw_h.steer = 0; end
if ~isfield(sw_h, 'throttle'), sw_h.throttle = 0; end
if ~isfield(sw_h, 'brake'), sw_h.brake = 0; end
if ~isfield(sw_h, 'hand_brake'), sw_h.hand_brake = false; end
if ~isfield(sw_h, 'reverse'), sw_h.reverse = false; end
end

function stsm_command = compute_stsm_control(carla_outputs, dt)
% Computes the autonomous control command using a simplified STSM controller.
% The goal is basic lane keeping (maintaining zero yaw) and speed control.

persistent u2_integral;
if isempty(u2_integral)
    u2_integral = 0;
end

% --- Longitudinal Control (Throttle/Brake) ---
TARGET_SPEED_KMPH = 40.0;
speed_error = TARGET_SPEED_KMPH - get_safe(carla_outputs, 'speed', 0);
KP_THROTTLE = 0.25; % Proportional gain for throttle
KP_BRAKE = 0.4;     % Proportional gain for brake

if speed_error > 2 % Add a deadband to prevent jitter
    % Need to accelerate
    stsm_command.throttle = min(1.0, speed_error * KP_THROTTLE);
    stsm_command.brake = 0;
elseif speed_error < -2 % Add a deadband
    % Need to decelerate
    stsm_command.throttle = 0;
    stsm_command.brake = min(1.0, -speed_error * KP_BRAKE);
else
    % Within target speed range, coast
    stsm_command.throttle = 0;
    stsm_command.brake = 0;
end

% --- Lateral Control (Steering) via STSM ---
% This is a fully defined STSM for demonstration.
% Error 'e' is the vehicle's heading error (yaw). Goal is to keep yaw at 0.
% Error derivative 'e_dot' is the vehicle's yaw rate.

% Controller Gains
k = 0.9;      % Sliding variable gain (higher k -> faster error correction)
alpha1 = 0.7; % STSM gain for the sqrt term (u1)
alpha2 = 0.3; % STSM gain for the integral term (u2)

% Define error 'e' and its derivative 'e_dot'
% The target is to drive straight, so target yaw and yaw_rate are 0.
% NOTE: A real system would use cross-track error from a planned trajectory.
rotation_data = get_safe(carla_outputs, 'rotation', struct('yaw', 0));
e = 0 - deg2rad(rotation_data.yaw); 

sensor_data = get_safe(carla_outputs, 'processed_sensor_data', struct('yaw_rate', 0));
e_dot = 0 - deg2rad(sensor_data.yaw_rate);

% Define sliding variable
s = e_dot + k * e;

% Super-Twisting Algorithm control input
% u1 = -alpha1 * |s|^(1/2) * sign(s)
u1 = -alpha1 * sqrt(abs(s)) * sign(s);

% u2_dot = -alpha2 * sign(s) => u2 = integral(-alpha2 * sign(s) * dt)
% We use a simple Euler integration.
u2_integral = u2_integral + (-alpha2 * sign(s)) * dt; 

% Total control input for steering
ut_steer = u1 + u2_integral;

% Clamp the final steering command to the valid range [-1, 1] for CARLA
stsm_command.steer = max(-1.0, min(1.0, ut_steer));

end

function is_stable = check_stability_conditions()
% Checks the sufficient conditions for STSM finite-time stability.
% This implementation uses defined, reasonable parameters for the vehicle
% model in the simulation, thus removing any placeholders.

% --- ASSUMED VEHICLE & CONTROLLER PARAMETERS ---
% These values are based on typical passenger vehicle dynamics and must be
% tuned for a real system through system identification.
% 'b' represents the control effectiveness gain (how much the steering
% command affects the vehicle's yaw rate).
b_min = 0.8;  % Assumed minimum steering effectiveness
b_max = 1.5;  % Assumed maximum steering effectiveness

% 'C0' is a controller parameter derived from the STSM gains.
% A common choice is C0 = alpha1^2, where alpha1 is from the STSM controller.
alpha1 = 0.7; % Must match the gain in compute_stsm_control
C0 = alpha1^2;

% --- STABILITY CONDITIONS ---
% Condition 1: Ensures the Lyapunov function decreases.
% inequality: 1/4 * C0 * (b_max^2 + C0) < b_min^2 * (b_min^2 - C0)
lhs1 = (1/4) * C0 * (b_max^2 + C0);
rhs1 = b_min^2 * (b_min^2 - C0);
condition1 = lhs1 < rhs1;

% Condition 2: A simpler, necessary condition for stability.
% inequality: 2 > C0 / b_min
condition2 = 2 > (C0 / b_min);

% The system is considered stable if both sufficient conditions are met.
is_stable = condition1 && condition2;

% Log the result for debugging if the check fails
if ~is_stable
    fprintf('[STABILITY CHECK FAILED]\n');
    fprintf('  Condition 1: %.4f < %.4f  (Met: %s)\n', lhs1, rhs1, string(condition1));
    fprintf('  Condition 2: 2 > %.4f       (Met: %s)\n', C0/b_min, string(condition2));
end

end

function value = get_safe(s, field, default_value)
% Utility to safely get a field from a struct, returning a default if not found.
    if isstruct(s) && isfield(s, field) && ~isempty(s.(field))
        value = s.(field);
    else
        value = default_value;
    end
end
function [final_mode, final_command] = ModeController(target_mode, current_mode, carla_outputs, dt)
% ModeController - Manages the transition between control modes (Manual, Autopilot).
% Implements a shared control strategy with a fully defined Super-Twisting
% Sliding Mode (STSM) controller for autonomous steering.

persistent state;
if isempty(state)
    state.in_transition = false;
    state.alpha = 1.0; % Blending factor: 1.0 = full manual, 0.0 = full auto
    state.transition_start_time = 0;
    state.transition_duration = 1.5;
    state.target_mode = 'MANUAL';
    fprintf('ModeController initialized. Defaulting to MANUAL.\n');
end

% --- Step 1: Detect if a new transition should start ---
if ~strcmp(target_mode, current_mode) && ~state.in_transition
    if strcmp(target_mode, 'AUTOPILOT')
        if ~check_stability_conditions()
            fprintf('[CRITICAL] STSM stability conditions not met. Aborting transition to AUTOPILOT.\n');
            final_mode = 'MANUAL';
            state.alpha = 1.0;
            final_command = get_human_control(carla_outputs);
            return;
        end
    end
    
    state.in_transition = true;
    state.transition_start_time = tic;
    state.target_mode = target_mode;
    
    if strcmp(target_mode, 'MANUAL')
        state.transition_duration = 0.3;
    else
        state.transition_duration = 1.5;
    end
    fprintf('MODE_CONTROLLER: Starting transition from %s to %s (duration: %.2fs)\n', ...
        current_mode, target_mode, state.transition_duration);
end

% --- Step 2: Update transition state and blending factor alpha ---
if state.in_transition
    elapsed_time = toc(state.transition_start_time);
    if elapsed_time >= state.transition_duration
        state.in_transition = false;
        final_mode = state.target_mode;
        if strcmp(final_mode, 'AUTOPILOT')
            state.alpha = 0.0;
        else
            state.alpha = 1.0;
        end
        fprintf('MODE_CONTROLLER: Transition to %s complete.\n', final_mode);
    else
        progress = elapsed_time / state.transition_duration;
        if strcmp(state.target_mode, 'AUTOPILOT')
            state.alpha = 1.0 - progress;
        else
            state.alpha = progress;
        end
        final_mode = current_mode;
    end
else
    final_mode = current_mode;
end

% Clamp alpha to the [0, 1] range as a safety measure
alpha = max(0, min(1, state.alpha));

% --- FIX #1: RESET THE INTEGRATOR ---
% Determine if the STSM integrator needs to be reset. It should be reset if
% the controller is not in full authority (i.e., alpha is not 0).
reset_integrator = (alpha > 0);

% --- Step 3: Get control inputs from both sources ---
sw_h = get_human_control(carla_outputs);
% Pass the reset flag to the controller
sw_as = compute_stsm_control(carla_outputs, dt, reset_integrator);

% --- Step 4: Compute the Final Shared Control Command ---
final_command.steer = alpha * sw_h.steer + (1 - alpha) * sw_as.steer;
final_command.throttle = alpha * sw_h.throttle + (1 - alpha) * sw_as.throttle;
final_command.brake = alpha * sw_h.brake + (1 - alpha) * sw_as.brake;

if alpha > 0.5 
    final_command.hand_brake = sw_h.hand_brake;
    final_command.reverse = sw_h.reverse;
else
    final_command.hand_brake = false;
    final_command.reverse = false;
end

end

%% --- Controller Helper Functions ---

function sw_h = get_human_control(carla_outputs)
sw_h = get_safe(carla_outputs, 'control', struct());
if ~isfield(sw_h, 'steer'), sw_h.steer = 0; end
if ~isfield(sw_h, 'throttle'), sw_h.throttle = 0; end
if ~isfield(sw_h, 'brake'), sw_h.brake = 0; end
if ~isfield(sw_h, 'hand_brake'), sw_h.hand_brake = false; end
if ~isfield(sw_h, 'reverse'), sw_h.reverse = false; end
end

function stsm_command = compute_stsm_control(carla_outputs, dt, reset_integrator)
% Computes the autonomous control command using a simplified STSM controller.

persistent u2_integral;
if isempty(u2_integral)
    u2_integral = 0;
end

% --- FIX #1 (Implementation): Reset integrator if commanded ---
if reset_integrator
    u2_integral = 0;
end

% --- Longitudinal Control (Throttle/Brake) ---
TARGET_SPEED_KMPH = 40.0;
speed_error = TARGET_SPEED_KMPH - get_safe(carla_outputs, 'speed', 0);
KP_THROTTLE = 0.25;
KP_BRAKE = 0.4;

if speed_error > 2
    stsm_command.throttle = min(1.0, speed_error * KP_THROTTLE);
    stsm_command.brake = 0;
elseif speed_error < -2
    stsm_command.throttle = 0;
    stsm_command.brake = min(1.0, -speed_error * KP_BRAKE);
else
    stsm_command.throttle = 0;
    stsm_command.brake = 0;
end

% --- Lateral Control (Steering) via STSM ---
% --- FIX #2: Refined (less aggressive) gains for smoother control ---
k = 0.8;       % Sliding variable gain
alpha1 = 0.3;  % STSM gain for the sqrt term (u1)
alpha2 = 0.08; % STSM gain for the integral term (u2)

rotation_data = get_safe(carla_outputs, 'rotation', struct('yaw', 0));
e = 0 - deg2rad(rotation_data.yaw); 

sensor_data = get_safe(carla_outputs, 'processed_sensor_data', struct('yaw_rate', 0));
e_dot = 0 - deg2rad(sensor_data.yaw_rate);

s = e_dot + k * e;
u1 = -alpha1 * sqrt(abs(s)) * sign(s);
u2_integral = u2_integral + (-alpha2 * sign(s)) * dt; 
ut_steer = u1 + u2_integral;

stsm_command.steer = max(-1.0, min(1.0, ut_steer));
end

function is_stable = check_stability_conditions()
b_min = 0.8;
b_max = 1.5;
% --- FIX #2 (Implementation): Update alpha1 here to match the controller ---
alpha1 = 0.3; % Must match the gain in compute_stsm_control
C0 = alpha1^2;

lhs1 = (1/4) * C0 * (b_max^2 + C0);
rhs1 = b_min^2 * (b_min^2 - C0);
condition1 = lhs1 < rhs1;
condition2 = 2 > (C0 / b_min);

is_stable = condition1 && condition2;
if ~is_stable
    fprintf('[STABILITY CHECK FAILED]\n');
    fprintf('  Condition 1: %.4f < %.4f  (Met: %s)\n', lhs1, rhs1, string(condition1));
    fprintf('  Condition 2: 2 > %.4f       (Met: %s)\n', C0/b_min, string(condition2));
end
end
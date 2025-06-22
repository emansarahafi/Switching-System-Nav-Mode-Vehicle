function [final_mode, final_command] = ModeController(target_mode, current_mode, carla_outputs, dt)
% ModeController - Manages control modes and implements advanced controllers.
% Lateral Control: Super-Twisting Sliding Mode (STSM).
% Longitudinal Control: Gain-Scheduled H-infinity for robust speed tracking.

persistent state;
if isempty(state)
    state.in_transition = false;
    state.alpha = 1.0;
    state.transition_start_time = 0;
    state.transition_duration = 1.5;
    state.target_mode = 'MANUAL';
    fprintf('ModeController initialized. Defaulting to MANUAL.\n');
end

% --- State Transition Logic ---
if ~strcmp(target_mode, current_mode) && ~state.in_transition
    % --- REALISTIC MODIFICATION: Trust the Fuzzy Arbitrator ---
    % The fuzzy system (target_mode) has already weighed all the nuanced factors
    % like driver readiness and environmental complexity. ModeController should
    % not second-guess it with its own hard-coded thresholds.
    % The only remaining check is for a catastrophic, non-negotiable
    % controller instability, which is a system-level fault.
    if strcmp(target_mode, 'AUTOPILOT')
        is_stable = check_stability_conditions();
        
        if ~is_stable
            fprintf('[FATAL ABORT] Transition to AUTOPILOT denied. Controller stability conditions not met.\n');
            final_mode = 'MANUAL'; state.alpha = 1.0;
            final_command = get_human_control(carla_outputs); 
            return;
        end
    end
    
    state.in_transition = true; state.transition_start_time = tic; state.target_mode = target_mode;
    if strcmp(target_mode, 'MANUAL'), state.transition_duration = 0.2; else, state.transition_duration = 1.5; end
    fprintf('MODE_CONTROLLER: Starting transition from %s to %s (duration: %.2fs)\n', current_mode, target_mode, state.transition_duration);
end

if state.in_transition
    elapsed_time = toc(state.transition_start_time);
    if elapsed_time >= state.transition_duration
        state.in_transition = false; final_mode = state.target_mode;
        if strcmp(final_mode, 'AUTOPILOT'), state.alpha = 0.0; else, state.alpha = 1.0; end
        fprintf('MODE_CONTROLLER: Transition to %s complete.\n', final_mode);
    else
        progress = elapsed_time / state.transition_duration;
        if strcmp(state.target_mode, 'AUTOPILOT'), state.alpha = 1.0 - progress; else, state.alpha = progress; end
        final_mode = current_mode;
    end
else, final_mode = current_mode; 
end

% --- Explicit State-Based Control Generation ---
alpha = max(0, min(1, state.alpha));
reset_integrator = (alpha > 0.95); 

if alpha >= 1.0 % FULL MANUAL MODE
    final_command = get_human_control(carla_outputs);
    compute_stsm_control(carla_outputs, dt, true);
    
elseif alpha <= 0.0 % FULL AUTOPILOT MODE
    final_command = compute_stsm_control(carla_outputs, dt, false);
    final_command.hand_brake = false;
    final_command.reverse = false;
    
else % TRANSITIONING (BLENDING)
    sw_h = get_human_control(carla_outputs);
    sw_as = compute_stsm_control(carla_outputs, dt, reset_integrator);

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
MAX_SPEED_KPH = 50.0;
LOOKAHEAD_CURVATURE_DIST_M = 20.0;
planner_target_speed_ms = get_safe(carla_outputs, 'intelligent_target_speed_ms', MAX_SPEED_KPH / 3.6);
proc_data = get_safe(carla_outputs, 'processed_sensor_data', struct());
lane_wps = get_safe(proc_data, 'lane_waypoints', [0 0; 1 0]);
curvature_speed_limit_ms = calculate_curvature_speed_limit(lane_wps, LOOKAHEAD_CURVATURE_DIST_M, MAX_SPEED_KPH / 3.6);
final_target_speed_ms = min([MAX_SPEED_KPH / 3.6, planner_target_speed_ms, curvature_speed_limit_ms]);
current_speed_ms = get_safe(carla_outputs, 'speed', 0) / 3.6;
speed_error = final_target_speed_ms - current_speed_ms;
[throttle, brake] = compute_gainsched_hinf_control(speed_error, current_speed_ms, reset_integrator, dt);
stsm_command.throttle = throttle;
stsm_command.brake = brake;
persistent u2_integral last_final_steer;
if isempty(u2_integral) || reset_integrator
    u2_integral = 0;
    last_final_steer = 0;
end
k = 0.8; alpha1 = 0.3; alpha2 = 0.08;
fused_state = get_safe(carla_outputs, 'sensor_fusion_status', struct());
ego_pos = get_safe(fused_state, 'fused_state', struct('x',0,'y',0));
ego_rot = get_safe(carla_outputs, 'rotation', struct('yaw', 0));
refPath = referencePathFrenet(lane_wps);
currentState = [ego_pos.x, ego_pos.y, deg2rad(ego_rot.yaw), 0, 0, current_speed_ms];
frenetState = global2frenet(refPath, currentState);
e = frenetState(2);
path_angle = frenetState(3);
psi_error = wrapToPi(currentState(3) - path_angle);
e_dot = current_speed_ms * sin(psi_error);
s = e_dot + k * e;
u1 = -alpha1 * sqrt(abs(s)) * sign(s);
u2_integral = u2_integral + (-alpha2 * sign(s)) * dt; 
ut_steer = u1 + u2_integral;
negated_steer_cmd = -ut_steer;
MAX_STEER_RATE = 0.1; 
steer_change = negated_steer_cmd - last_final_steer;
clamped_change = max(min(steer_change, MAX_STEER_RATE), -MAX_STEER_RATE);
final_steer = last_final_steer + clamped_change;
last_final_steer = final_steer;
stsm_command.steer = max(-1.0, min(1.0, final_steer)); 
end

function speed_limit_ms = calculate_curvature_speed_limit(waypoints, lookahead_dist, max_speed)
persistent LATERAL_ACCEL_MAX;
if isempty(LATERAL_ACCEL_MAX), LATERAL_ACCEL_MAX = 1.5; end % m/s^2
if size(waypoints, 1) < 3, speed_limit_ms = max_speed; return; end
min_radius = Inf;
total_dist = 0;
for i = 1:(size(waypoints, 1) - 2)
    p1 = waypoints(i, :); p2 = waypoints(i+1, :); p3 = waypoints(i+2, :);
    total_dist = total_dist + norm(p2 - p1);
    if total_dist > lookahead_dist, break; end
    a = norm(p2-p3); b = norm(p1-p3); c = norm(p1-p2);
    area = 0.5 * abs(p1(1)*(p2(2)-p3(2)) + p2(1)*(p3(2)-p1(2)) + p3(1)*(p1(2)-p2(2)));
    if area < 1e-6, continue; end
    radius = (a * b * c) / (4 * area);
    min_radius = min(min_radius, radius);
end
if isinf(min_radius), speed_limit_ms = max_speed;
else, speed_limit_ms = sqrt(LATERAL_ACCEL_MAX * min_radius); end
speed_limit_ms = min(speed_limit_ms, max_speed);
end

function [throttle, brake] = compute_gainsched_hinf_control(speed_error, current_speed, reset_controller, dt)
persistent sched_controller sched_state;
if isempty(sched_controller) || reset_controller
    if isempty(sched_controller), fprintf('Designing Gain-Scheduled H-inf Speed Controller...\n'); end
    sched_controller = design_gainsched_hinf_controller();
    sched_state = zeros(sched_controller.num_states, 1);
end
speed_min = sched_controller.speed_range(1);
speed_max = sched_controller.speed_range(2);
alpha = (current_speed - speed_min) / (speed_max - speed_min);
alpha = max(0, min(1, alpha));
[A_min, B_min, C_min, D_min] = ssdata(sched_controller.K_min);
[A_max, B_max, C_max, D_max] = ssdata(sched_controller.K_max);
A_k = (1-alpha)*A_min + alpha*A_max; B_k = (1-alpha)*B_min + alpha*B_max;
C_k = (1-alpha)*C_min + alpha*C_max; D_k = (1-alpha)*D_min + alpha*D_max;
sched_state = sched_state + dt * (A_k * sched_state + B_k * speed_error);
control_output = C_k * sched_state + D_k * speed_error;
if control_output > 0, throttle = min(1.0, control_output); brake = 0;
else, throttle = 0; brake = min(1.0, -control_output); end
end

function sched_controller = design_gainsched_hinf_controller()
speed_range = [0, 25];
a_min = 0.2; b_min = 0.8; P_min = ss(-a_min, b_min, 1, 0);
[K_min, ~, ~] = hinfsyn(P_min, 1, 1);
a_max = 0.2 + 0.02*25; b_max = 0.8 - 0.02*25; P_max = ss(-a_max, b_max, 1, 0);
[K_max, ~, ~] = hinfsyn(P_max, 1, 1);
if size(K_min.A,1) ~= size(K_max.A,1), error('Controller orders do not match.'); end
sched_controller.K_min = K_min; sched_controller.K_max = K_max;
sched_controller.speed_range = speed_range;
sched_controller.num_states = size(K_min.A, 1);
end

function is_stable = check_stability_conditions()
b_min = 0.8; b_max = 1.5; alpha1 = 0.3; C0 = alpha1^2;
lhs1 = (1/4) * C0 * (b_max^2 + C0); rhs1 = b_min^2 * (b_min^2 - C0);
condition1 = lhs1 < rhs1; condition2 = 2 > (C0 / b_min);
is_stable = condition1 && condition2;
if ~is_stable
    fprintf('[STABILITY CHECK FAILED] Condition 1: %.4f < %.4f (%s), Condition 2: 2 > %.4f (%s)\n', lhs1, rhs1, string(condition1), C0/b_min, string(condition2));
end
end
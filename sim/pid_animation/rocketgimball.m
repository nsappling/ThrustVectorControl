%% rocketgimball.m
% MATLAB port of rocketgimball.py: a top-heavy "rocket" balanced on a
% pivot, stabilized by a PID controller vectoring a ducted fan mounted
% on a servo below the pivot. Same physics/control logic as the Python
% turtle version -- this file just swaps turtle graphics for MATLAB's
% patch/line graphics objects, updated every frame with drawnow.

clear; clc; close all;

% =====================================================================
% PHYSICAL / CONTROL PARAMETERS
% =====================================================================

dt = 0.0005;
% Physics timestep, in seconds.

F_t = 6.87; %[N] guess for fan strength
R = 0.15; %[m] guess for distance from pivot to fan
m = 1; %[kg] guess for weight of total rocket
r_0 = 0.03; %[m] guess for distance from pivot to CG
g = 9.81; %[m/s^2] gravity



a = g / r_0;
% "Gravity" term for the inverted-pendulum instability (g / r_0, where r_0
% is roughly the distance from the pivot to the effective center of
% mass above it). Bigger = tips over faster with no control.
%Value from coefficient of sintheta for equation of motion

b = F_t * R/(m*r_0^2);
% Converts PID output "u" into angular acceleration (rad/s^2 per unit
% u). Must be POSITIVE so the controller pushes back toward upright
% (negative feedback) -- a negative value here makes the loop unstable
% almost instantly.
%value derived from guesses of input values. the b coefficient: b = F_t·R / (m·r₀²) = (6.87 × 0.15) / (1 × 0.03²) ≈ 1144

Kp = 0.42; Ki = 0.1; Kd = 0.015;
% PID gains: proportional / integral / derivative.

sigma_noise = 0.02;   % simulated IMU noise std dev, in radians
tau_filter  = 0.02;   % low-pass filter time constant, in seconds
alpha = dt / (tau_filter + dt);         %this computes the filter coefficient (the discrete version of the transfer function 1/(τs+1)).
% Discrete low-pass filter coefficient. Must be computed AFTER dt has
% its final value.

VIS_SCALE = 200;
% Cosmetic only: scales PID output "u" into a fan-tilt angle (degrees)
% purely for drawing. Does not affect the physics.

fan_tilt_max_deg = 35;
% Physical limit on how far the fan/servo can gimbal, in degrees. Used
% both to clamp the autopilot's visual fan tilt AND to size the manual
% control's authority below.

% --- Actuator saturation (applies in BOTH modes -- a real servo has ---
% --- a hard limit no matter who/what is commanding it)              ---
Umax_actuator = fan_tilt_max_deg / VIS_SCALE;
% The biggest "u" the real servo/fan can produce, full deflection.
% Previously only the DRAWING was clamped to +/-35 degrees while the
% physics used the raw, unclamped PID output -- meaning the simulated
% actuator had effectively infinite authority. This clamp is applied
% to accel_cmd in the main loop so the physics can never assume more
% authority than the drawing shows.

manual_decay_tau = 0.02;     % seconds; how fast u relaxes back to 0 when no key is held
% NOTE: this plant tips over on a ~55ms timescale (sqrt(a) ~= 18
% rad/s), which is close to or faster than human reaction time. Manual
% control snaps straight to +/-Umax while a key is held (no ramp-up
% delay) -- any added lag here beyond human reaction time makes the
% rocket essentially unrecoverable by hand, verified numerically.

% --- Mechanical stops (pins) ---
% Real hardware will have a physical limit -- pins or a bumper -- that
% stop the body from tipping past a set angle. This is not just for
% looks: given the actuator authority above, the fan can only out-
% torque gravity up to asin(b*Umax_actuator/a). Past that angle NO
% controller (PID or human) has enough authority to recover, so the
% stop angle should sit comfortably inside that limit.
max_recoverable_deg = asind(b * Umax_actuator / a);
theta_limit_deg = 30;   % keep this well under max_recoverable_deg
theta_limit = deg2rad(theta_limit_deg);

% =====================================================================
% SIMULATION STATE
% =====================================================================

theta      = 0.2;   % true body angle (rad), 0 = upright. Start tipped so you see the recovery.
theta_dot  = 0.0;    % angular velocity (rad/s)
theta_filt = 0.0;    % filtered angle estimate (what the controller sees)
integral_term = 0.0;  % running integral of error
prev_error = [];    % previous error, [] = "not measured yet" (like Python's None)
ref = 0.0;           % target angle: upright
manual_u = 0.0;       % current manually-commanded control signal (MANUAL mode only)

% =====================================================================
% FIGURE / GRAPHICS SETUP
% Build the figure once and create the graphics objects (patch for the
% body, line for the fan, markers for the pivot and fan tip). Every
% frame after this just updates their XData/YData -- no re-creating.
% =====================================================================

pivot_x = 0; pivot_y = 20;

Lu = 120;   % body length ABOVE pivot, up to nose tip
Ll = 80;    % body length BELOW pivot, down to fan mount point
Lf = 35;    % fan housing length, hanging below the mount point

% Body outline, relative to the pivot at local (0,0), nose pointing local +y
body_local = [
    0,   Lu;
   -15,  Lu-40;
   -15, -Ll+25;
   -30, -Ll+10;
   -10, -Ll;
    10, -Ll;
    30, -Ll+10;
    15, -Ll+25;
    15,  Lu-40;
];

fig = figure('Color', 'w');
ax = axes(fig); hold(ax, 'on'); axis(ax, 'equal');
xlim(ax, [-300 300]); ylim(ax, [-300 300]);
title(ax, 'Rocket TVC Demo (MATLAB)');

plot(ax, pivot_x, pivot_y, 'k.', 'MarkerSize', 20); % fixed pivot dot, drawn once

% --- Stop pegs: fixed in world space (they don't rotate with the body). ---
% Positioned at exactly where the body's shoulder sits when theta is at
% the mechanical limit, so the body visibly comes to rest against them
% (up near the shoulder, above the pivot -- not down at the fin tips).
pegLocal = [15, Lu-40];   % shoulder contact point, in body-local coords
[pegRx, pegRy] = rotate_pt(pegLocal(1), pegLocal(2), theta_limit);
rightPeg = [pivot_x + pegRx, pivot_y + pegRy];
[pegLx, pegLy] = rotate_pt(-pegLocal(1), pegLocal(2), -theta_limit);
leftPeg  = [pivot_x + pegLx, pivot_y + pegLy];

pegRadius = 10;
rectangle(ax, 'Position', [rightPeg(1)-pegRadius, rightPeg(2)-pegRadius, 2*pegRadius, 2*pegRadius], ...
              'Curvature', [1 1], 'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k');
rectangle(ax, 'Position', [leftPeg(1)-pegRadius, leftPeg(2)-pegRadius, 2*pegRadius, 2*pegRadius], ...
              'Curvature', [1 1], 'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k');

bodyPatch = patch(ax, 'XData', nan(size(body_local,1),1), ...
                       'YData', nan(size(body_local,1),1), ...
                       'FaceColor', [0.27 0.51 0.71], 'EdgeColor', 'k');
% steelblue-ish fill, matches the Python body.color("black", "steelblue")

fanArrow = quiver(ax, 0, 0, 0, 0, 'AutoScale', 'off', 'Color', [0.70 0.13 0.13], ...
                       'LineWidth', 3, 'MaxHeadSize', 1.2);
% Arrow from the mount point to the fan tip -- shows the actual
% direction the exhaust is being aimed (thrust direction), not just an
% abstract housing line.

statusText = text(ax, -290, 270, '', 'FontSize', 11, 'FontWeight', 'bold', ...
                       'VerticalAlignment', 'top');
% On-screen mode/instructions readout, updated every frame.

% =====================================================================
% USER INPUT: keyboard-controlled manual fan tilt
% Left/Right arrow keys (or A/D) ramp the fan tilt manually; M toggles
% between AUTO (the PID autopilot demo) and MANUAL (you fly it by
% hand). Key state is stored with setappdata/getappdata on the figure
% because MATLAB callbacks fire asynchronously between drawnow calls --
% this is the standard way to share state between a callback and a
% running script loop.
% =====================================================================

setappdata(fig, 'leftDown', false);
setappdata(fig, 'rightDown', false);
setappdata(fig, 'manualMode', false);   % start in AUTO mode

fig.KeyPressFcn   = @keyDownCB;
fig.KeyReleaseFcn = @keyUpCB;

% =====================================================================
% SIMULATION LOOP
% =====================================================================

substeps_per_frame = 4;
kick_interval_frames = 1000;   % periodically bump the rocket so there's always something to watch

wasManualMode = false;   % tracks the previous frame's mode, to detect AUTO<->MANUAL transitions
frame = 0;

while ishandle(fig)   % run until the user closes the window
    frame = frame + 1;

    % --- Read current key state / mode (set asynchronously by the callbacks) ---
    leftDown   = getappdata(fig, 'leftDown');
    rightDown  = getappdata(fig, 'rightDown');
    manualMode = getappdata(fig, 'manualMode');

    % --- Handle AUTO <-> MANUAL transitions cleanly ---
    if manualMode && ~wasManualMode
        manual_u = 0.0;   % start manual input centered
    elseif ~manualMode && wasManualMode
        integral_term = 0.0;   % don't let the autopilot inherit a stale integral/derivative history
        prev_error = [];        % from whatever the rocket was doing under manual control
    end
    wasManualMode = manualMode;

    % --- Random disturbance: only in AUTO mode, so it doesn't fight the user's own input ---
    if ~manualMode && frame > 1 && mod(frame, kick_interval_frames) == 0
        theta = theta + sign(randn) * (0.12 + 0.10*rand);
    end

    u_last = 0.0;
    for k = 1:substeps_per_frame

        if manualMode
            % --- Manual control: snap straight to +/-Umax while a key is ---
            % --- held (no ramp-up lag -- see the note above on why),   ---
            % --- and relax back toward 0 quickly when released.        ---
            if rightDown && ~leftDown
                manual_u = Umax_actuator;
            elseif leftDown && ~rightDown
                manual_u = -Umax_actuator;
            else
                manual_u = manual_u - (dt / (manual_decay_tau + dt)) * manual_u;
            end
            u = manual_u;
        else
            % --- Simulated IMU measurement ---
            measured_theta = theta + sigma_noise * randn;

            % --- Low-pass filter ---
            theta_filt = theta_filt + alpha * (measured_theta - theta_filt); %Fake Control loop - This is the recursive difference equation itself, applied every substep. This is the line that's actually doing the filtering.

            % --- PID controller ---
            error = ref - theta_filt;
            if isempty(prev_error)
                prev_error = error;  % avoid a derivative-kick on the very first sample
            end
            integral_term = integral_term + error * dt;
            derivative = (error - prev_error) / dt;
            prev_error = error;

            u = Kp*error + Ki*integral_term + Kd*derivative;
        end

        % --- Actuator saturation: the real servo can't exceed +/-35 ---
        % deg no matter which mode commanded it (previously only the
        % drawing was clamped -- the physics assumed infinite authority).
        u = max(-Umax_actuator, min(Umax_actuator, u));
        u_last = u;
        accel_cmd = b * u;

        % --- Physics: semi-implicit ("symplectic") Euler integration ---
        % (same physics either way -- AUTO and MANUAL only differ in
        % how "u" gets computed above). Uses sin(theta), not theta, for
        % the gravity torque -- theta routinely leaves the small-angle
        % range in MANUAL mode, and the small-angle version grows
        % without bound instead of saturating like the real torque does.
        theta_ddot = accel_cmd + a * sin(theta);
        theta_dot  = theta_dot + theta_ddot * dt;   % update velocity first...
        theta      = theta + theta_dot * dt;         % ...then use the NEW velocity for angle

        % --- Mechanical stops (pins): body rests against them, doesn't ---
        % --- bounce -- an inelastic contact, same as a real pin/bumper. ---
        if theta > theta_limit
            theta = theta_limit;
            theta_dot = 0;
        elseif theta < -theta_limit
            theta = -theta_limit;
            theta_dot = 0;
        end
    end

    % --- Visual fan tilt for drawing -- SIGN FLIPPED relative to u ---
    % Our rotate_pt() convention is clockwise-positive (verified: growing
    % theta swings the nose right = clockwise). Tracing the real physics
    % (exhaust direction -> Newton's-3rd-law reaction at the mount ->
    % torque about the pivot) shows that a rightward-pointing exhaust
    % produces a CLOCKWISE torque -- i.e. exhaust direction and theta_ddot
    % should have the SAME sign. Since fan_tilt_rad is combined with theta
    % through rotate_pt (also clockwise-positive), matching the drawn
    % arrow to the true exhaust direction requires fan_tilt to be the
    % NEGATIVE of u here (theta_ddot itself is still just b*u, unchanged).
    fan_tilt_deg = max(-fan_tilt_max_deg, min(fan_tilt_max_deg, -u_last * VIS_SCALE));
    fan_tilt_rad = deg2rad(fan_tilt_deg);

    draw_scene(bodyPatch, fanArrow, body_local, pivot_x, pivot_y, Ll, Lf, theta, fan_tilt_rad);

    if manualMode
        statusText.String = sprintf('MANUAL -- Left/Right arrows (or A/D) to gimbal the fan.  [M] = back to AUTO');
        statusText.Color = [0.70 0.13 0.13];
    else
        statusText.String = sprintf('AUTO -- PID autopilot balancing itself.  [M] = take manual control');
        statusText.Color = [0.13 0.35 0.13];
    end

    drawnow;
end

% =====================================================================
% LOCAL FUNCTIONS
% (MATLAB scripts require local functions to be defined at the end of
% the file; they're still usable from anywhere above.)
% =====================================================================

function [xr, yr] = rotate_pt(x, y, angle)
    % Same 2D rotation convention used throughout, applied consistently
    % to the body and the fan so the geometry stays correct.
    xr = x*cos(angle) + y*sin(angle);
    yr = -x*sin(angle) + y*cos(angle);
end

function draw_scene(bodyPatch, fanArrow, body_local, pivot_x, pivot_y, Ll, Lf, theta, fan_tilt_rad)
    % --- Body: rotates about the fixed pivot by theta ---
    n = size(body_local, 1);
    bx = zeros(n,1); by = zeros(n,1);
    for i = 1:n
        [rx, ry] = rotate_pt(body_local(i,1), body_local(i,2), theta);
        bx(i) = pivot_x + rx;
        by(i) = pivot_y + ry;
    end
    set(bodyPatch, 'XData', bx, 'YData', by);

    % --- Fan mount point: rotated by theta only (rigidly attached to body) ---
    [mx, my] = rotate_pt(0, -Ll, theta);
    mount = [pivot_x + mx, pivot_y + my];

    % --- Fan/thrust arrow: rotates by theta + fan_tilt (relative to body) -- ---
    % --- this is the vectored thrust, drawn as an arrow from the mount    ---
    % --- pointing in the actual exhaust/thrust direction.                 ---
    [fx, fy] = rotate_pt(0, -Lf, theta + fan_tilt_rad);
    set(fanArrow, 'XData', mount(1), 'YData', mount(2), 'UData', fx, 'VData', fy);
end

function keyDownCB(src, event)
    % Fires while a key is held down (figure's KeyPressFcn). Just
    % records which keys are currently pressed / toggles the mode --
    % the actual control logic lives in the main loop, which polls
    % this state every frame via getappdata.
    switch event.Key
        case {'leftarrow', 'a'}
            setappdata(src, 'leftDown', true);
        case {'rightarrow', 'd'}
            setappdata(src, 'rightDown', true);
        case 'm'
            setappdata(src, 'manualMode', ~getappdata(src, 'manualMode'));
    end
end

function keyUpCB(src, event)
    % Fires when a key is released (figure's KeyReleaseFcn).
    switch event.Key
        case {'leftarrow', 'a'}
            setappdata(src, 'leftDown', false);
        case {'rightarrow', 'd'}
            setappdata(src, 'rightDown', false);
    end
end

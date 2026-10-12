%% rocketgimball.m
clear; clc; close all;

% PHYSICAL / CONTROL PARAMETERS

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
% both to clamp the autopilot's visual fan tilt AND as the range of the
% manual-control dial.

% --- Actuator saturation (applies in BOTH modes -- a real servo has ---
% --- a hard limit no matter who/what is commanding it)              ---
Umax_actuator = fan_tilt_max_deg / VIS_SCALE;
% The biggest "u" the real servo/fan can produce, full deflection.
% Previously only the DRAWING was clamped to +/-35 degrees while the
% physics used the raw, unclamped PID output -- meaning the simulated
% actuator had effectively infinite authority. This clamp is applied
% to accel_cmd in the main loop so the physics can never assume more
% authority than the drawing shows.

sim_speed = 0.25;
% Playback speed: 1 = real time, 0.25 = 4x slow motion. Changeable live
% from the dropdown in the window. The animation is paced to the wall
% clock, so this is a TRUE speed no matter how fast the computer draws.
% NOTE: this plant tips over on a ~55ms timescale (1/sqrt(a), sqrt(a) ~=
% 18 rad/s), faster than human reaction time (~200 ms). At 1x, flying it
% by hand with the dial is close to impossible -- that's the point of
% the demo (TVC needs a computer). Slow motion makes manual mode playable.

dial_step_deg = 2;
% How far one press of the Left/Right (or A/D) key turns the dial.

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
sim_time = 0.0;       % simulated time elapsed (s), used for the periodic kicks

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

% The window is a uifigure so it can hold a real dial (uiknob) next to
% the animation: left = animation axes, right = control panel.
fig = uifigure('Name', 'Rocket TVC Demo (MATLAB)', 'Color', 'w', ...
               'Position', [100 100 1000 640]);
layout = uigridlayout(fig, [1 2], 'ColumnWidth', {'1x', 240});

ax = uiaxes(layout); hold(ax, 'on'); axis(ax, 'equal');
xlim(ax, [-300 300]); ylim(ax, [-300 300]);
title(ax, 'Rocket TVC Demo (MATLAB)');
ax.XTick = []; ax.YTick = []; ax.XColor = 'none'; ax.YColor = 'none';
disableDefaultInteractivity(ax);   % no accidental pan/zoom while flying

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

statusText = text(ax, -290, 290, '', 'FontSize', 11, 'FontWeight', 'bold', ...
                       'VerticalAlignment', 'top');
% On-screen mode/instructions readout, updated every frame.

% =====================================================================
% CONTROL PANEL (right side): mode switch, thrust-vector dial, speed,
% kick button, live readouts.
% =====================================================================

panel = uigridlayout(layout, [9 1], 'RowHeight', {30, 22, 190, 22, 30, 22, 30, 30, '1x'}, ...
                     'Padding', [10 10 10 10]);

modeSwitch = uiswitch(panel, 'slider', 'Items', {'AUTO', 'MANUAL'}, 'Value', 'AUTO');
% AUTO = PID autopilot flies it; MANUAL = you fly it with the dial.

uilabel(panel, 'Text', 'Fan tilt (thrust vector), deg', 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');

dial = uiknob(panel, 'continuous', 'Limits', [-fan_tilt_max_deg fan_tilt_max_deg], ...
              'MajorTicks', [-35 -20 -10 0 10 20 35], 'Value', 0, 'Enable', 'off');
% The dial angle IS the fan tilt: turning it clockwise swings the
% exhaust clockwise, same as the red arrow. In AUTO it's greyed out and
% just shows what the autopilot is commanding; in MANUAL it's yours.

dialLabel = uilabel(panel, 'Text', '0.0 deg', 'HorizontalAlignment', 'center', ...
                    'FontSize', 14);

centerBtn = uibutton(panel, 'Text', 'Center dial (Space)', 'Enable', 'off');

uilabel(panel, 'Text', 'Simulation speed', 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');
speedMenu = uidropdown(panel, 'Items', {'1x (real time)', '0.5x', '0.25x', '0.1x'}, ...
                       'ItemsData', [1 0.5 0.25 0.1], 'Value', sim_speed);

kickBtn = uibutton(panel, 'Text', 'Kick the rocket (K)');

readout = uilabel(panel, 'Text', '', 'VerticalAlignment', 'top', 'WordWrap', 'on');
% Live numbers: angle, rate, fan tilt, sim time.

% =====================================================================
% USER INPUT
% The dial (or Left/Right arrows, A/D) sets the fan tilt in MANUAL mode;
% the switch (or M) toggles AUTO/MANUAL; K or the button kicks the
% rocket. All of it is stored with setappdata/getappdata on the figure
% because MATLAB callbacks fire asynchronously between drawnow calls --
% this is the standard way to share state between a callback and a
% running script loop.
% =====================================================================

setappdata(fig, 'manualMode', false);   % start in AUTO mode
setappdata(fig, 'dialDeg', 0);          % manual fan-tilt command, degrees
setappdata(fig, 'kickRequest', false);
setappdata(fig, 'dialMovedByKey', false);   % keys/button changed dialDeg -> move the on-screen dial to match

modeSwitch.ValueChangedFcn = @(src, ~) setappdata(fig, 'manualMode', strcmp(src.Value, 'MANUAL'));
dial.ValueChangingFcn = @(~, evt) setappdata(fig, 'dialDeg', evt.Value);   % live, while dragging
dial.ValueChangedFcn  = @(src, ~) setappdata(fig, 'dialDeg', src.Value);   % on release
centerBtn.ButtonPushedFcn = @(~, ~) set_dial(fig, 0);
kickBtn.ButtonPushedFcn   = @(~, ~) setappdata(fig, 'kickRequest', true);
fig.WindowKeyPressFcn = @(src, evt) keyDownCB(src, evt, dial_step_deg, fan_tilt_max_deg);


% =====================================================================
% SIMULATION LOOP
% =====================================================================

kick_interval_s = 2.0;   % AUTO mode: bump the rocket every 2 s of sim time so there's always something to watch
next_kick_time = kick_interval_s;
max_substeps = 200;      % cap per frame (= 0.1 s of sim time) so a slow frame can't stall the loop

wasManualMode = false;   % tracks the previous frame's mode, to detect AUTO<->MANUAL transitions
u_last = 0.0;
frameClock = tic;

while isvalid(fig)   % run until the user closes the window

    % --- Read current mode / dial / buttons (set asynchronously by the callbacks) ---
    manualMode = getappdata(fig, 'manualMode');
    sim_speed  = speedMenu.Value;

    % --- Handle AUTO <-> MANUAL transitions cleanly ---
    if manualMode && ~wasManualMode
        % Bumpless handover: the dial starts wherever the autopilot left
        % the fan, so taking control doesn't jerk the thrust vector.
        setappdata(fig, 'dialDeg', dial.Value);
        dial.Enable = 'on'; centerBtn.Enable = 'on';
        modeSwitch.Value = 'MANUAL';   % keep the switch in sync if M was pressed
    elseif ~manualMode && wasManualMode
        integral_term = 0.0;   % don't let the autopilot inherit a stale integral/derivative history
        prev_error = [];        % from whatever the rocket was doing under manual control
        dial.Enable = 'off'; centerBtn.Enable = 'off';
        modeSwitch.Value = 'AUTO';
    end
    wasManualMode = manualMode;

    % --- Disturbances: the Kick button (any mode), plus automatic ---
    % --- kicks in AUTO mode only, so they don't fight the user's input ---
    if getappdata(fig, 'kickRequest')
        theta = theta + sign(randn) * (0.12 + 0.10*rand);
        setappdata(fig, 'kickRequest', false);
    end
    if ~manualMode && sim_time >= next_kick_time
        theta = theta + sign(randn) * (0.12 + 0.10*rand);
        next_kick_time = sim_time + kick_interval_s;
    elseif manualMode
        next_kick_time = sim_time + kick_interval_s;
    end

    % --- Real-time pacing: simulate exactly as much time as actually ---
    % --- passed on the wall clock (times sim_speed) since last frame. ---
    wall_dt = toc(frameClock); frameClock = tic;
    nsub = min(max_substeps, max(1, round(wall_dt * sim_speed / dt)));

    % Manual command from the dial: dial angle = fan tilt in degrees.
    % Fan tilt is drawn as -u*VIS_SCALE (see the sign note below), so
    % u = -dial/VIS_SCALE.
    manual_u = -getappdata(fig, 'dialDeg') / VIS_SCALE;

    for k = 1:nsub

        if manualMode
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
    sim_time = sim_time + nsub * dt;

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
        statusText.String = sprintf('MANUAL -- turn the dial (or Left/Right, A/D) to aim the fan.  [M] = AUTO');
        statusText.Color = [0.70 0.13 0.13];
    else
        statusText.String = sprintf('AUTO -- PID autopilot balancing itself.  [M] = take manual control');
        statusText.Color = [0.13 0.35 0.13];
        dial.Value = fan_tilt_deg;   % dial mirrors what the autopilot is commanding
    end
    if getappdata(fig, 'dialMovedByKey')
        % Keyboard/button turned the dial: move the on-screen knob to
        % match. (Not done every frame -- that would fight a mouse drag.)
        dial.Value = getappdata(fig, 'dialDeg');
        setappdata(fig, 'dialMovedByKey', false);
    end
    dialLabel.Text = sprintf('%+.1f deg', fan_tilt_deg);
    readout.Text = sprintf(['Body angle:  %+6.1f deg\nAngle rate:  %+6.0f deg/s\n' ...
                            'Fan tilt:    %+6.1f deg\nSim time:    %6.1f s  (%gx)\n\n' ...
                            'Tip: turn the dial TOWARD the side the rocket is leaning.\n\n' ...
                            'Stops at +/-%g deg. Fan can only recover up to %.0f deg.'], ...
                           rad2deg(theta), rad2deg(theta_dot), fan_tilt_deg, sim_time, sim_speed, ...
                           theta_limit_deg, max_recoverable_deg);

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

function keyDownCB(fig, event, step_deg, max_deg)
    % Fires on every key press in the window (WindowKeyPressFcn), even
    % while the dial or a button has focus. Only records the request --
    % the actual control logic lives in the main loop, which polls this
    % state every frame via getappdata. Holding a key auto-repeats, so
    % holding Left/Right keeps turning the dial.
    manual = getappdata(fig, 'manualMode');
    switch event.Key
        case {'leftarrow', 'a'}
            if manual, nudge_dial(fig, -step_deg, max_deg); end
        case {'rightarrow', 'd'}
            if manual, nudge_dial(fig, +step_deg, max_deg); end
        case 'space'
            if manual, set_dial(fig, 0); end
        case 'k'
            setappdata(fig, 'kickRequest', true);
        case 'm'
            setappdata(fig, 'manualMode', ~manual);
    end
end

function nudge_dial(fig, delta, max_deg)
    v = getappdata(fig, 'dialDeg') + delta;
    set_dial(fig, max(-max_deg, min(max_deg, v)));
end

function set_dial(fig, deg)
    % Set the manual fan-tilt command from the keyboard or Center button.
    setappdata(fig, 'dialDeg', deg);
    setappdata(fig, 'dialMovedByKey', true);
end

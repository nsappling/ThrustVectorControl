%% rocketgimball.m
% NOTE: firmware/tvc_controller/tvc_controller.ino does NOT match this
% file anymore (old gains, old VIS_SCALE units, D from a differentiated
% angle). Don't copy gains between them; see the warning at its top.
clear; clc; close all;

% PHYSICAL / CONTROL PARAMETERS

dt = 0.0005;
% Physics timestep, in seconds.

F_t = 6.87; %[N] guess for fan strength
R = 0.15; %[m] guess for distance from pivot to fan
m = 1; %[kg] guess for weight of total rocket
r_0 = 0.03; %[m] guess for distance from pivot to CG
g = 9.81; %[m/s^2] gravity

% --- Moment of inertia: rocket modeled as a UNIFORM SOLID CYLINDER ---
% Body dimensions from cad/rocket_assembly/CAD_Equations.txt (Body_Height
% = 12 in, Body_OD = 3.5 in). The pivot sits r_0 below the cylinder's
% center (= its CG, since it's uniform), so by the parallel-axis theorem:
%   I = m*(3*r_body^2 + L_body^2)/12   (cylinder about its own center,
%                                        transverse axis)
%     + m*r_0^2                         (shift to the pivot)
% Previously the model assumed ALL the mass sat at the CG (I = m*r_0^2),
% which made I ~10x too small and the rocket tip over ~3x too fast.
L_body = 12 * 0.0254;         %[m] body length (CAD Body_Height)
r_body = 3.5/2 * 0.0254;      %[m] body radius (CAD Body_OD / 2)
I = m*(3*r_body^2 + L_body^2)/12 + m*r_0^2;   %[kg m^2] about the pivot

a = m * g * r_0 / I;
% "Gravity" term for the inverted-pendulum instability: gravity torque
% (m*g*r_0*sin(theta)) divided by inertia. Bigger = tips over faster with
% no control; sqrt(a) is the unstable pole, 1/sqrt(a) its time constant.
%Value from coefficient of sintheta for equation of motion

b = F_t * R / I;
% Fan "control effectiveness": the angular acceleration the fan can
% produce, per unit of sin(fan tilt). The fan pushes with thrust F_t; tilting
% it by an angle delta gives a SIDEWAYS force F_t*sin(delta) at distance R
% below the pivot, i.e. a torque F_t*R*sin(delta). Divide by I to get
% angular acceleration: b*sin(delta). Must be POSITIVE (with the sign
% convention below) so the controller pushes back toward upright.
% With the guesses above: I ~= 0.0091 kg m^2, a ~= 32 1/s^2, b ~= 113.
%
% EQUATION OF MOTION (what the physics loop integrates):
%   theta_ddot = b*sin(u) + a*sin(theta)
%                ^ fan      ^ gravity (tips it over -- the instability)
% where u = fan tilt angle in RADIANS (the controller's output).

% =====================================================================
% CONTROLLER GAINS -- how they were chosen (pole placement)
% ---------------------------------------------------------------------
% For small angles, sin(x) ~= x, so the plant is approximately
%   theta_ddot = a*theta + b*u                                    (1)
% A PD controller commands  u = -Kp*theta - Kd*theta_dot           (2)
% (fan tilt proportional to how far it leans, plus how fast). Put (2)
% into (1):
%   theta_ddot + (b*Kd)*theta_dot + (b*Kp - a)*theta = 0
% That's a mass-spring-damper. Compare it with the standard form
%   theta_ddot + 2*zeta*wn*theta_dot + wn^2*theta = 0
% and match terms:
%   b*Kp - a = wn^2   ->   Kp = (wn^2 + a)/b    "spring": must beat gravity
%   b*Kd = 2*zeta*wn  ->   Kd = 2*zeta*wn/b     "damper": stops oscillation
% We picked wn = 6 rad/s (about 1x the unstable pole sqrt(a) = 5.7 --
% fast enough to catch it, slow enough to leave room for servo delay)
% and zeta = 1 (critically damped: fastest return with no oscillation).
%   Kp = (36 + 32.2)/112.8 = 0.60,   Kd = 2*1*6/112.8 = 0.105
% Note Kp > a/b = 0.29 is REQUIRED: below that the "spring" is weaker
% than gravity and it falls over no matter what Kd is.
% Ki (integral) slowly removes any steady lean caused by a constant push
% or an off-center CG. It's kept small (integral zero at ~0.05*wn) so it
% barely disturbs the fast PD behavior.
% Units: Kp in (rad of fan tilt) per (rad of lean) -- Kp = 0.6 means 1
% deg of lean -> 0.6 deg of fan tilt. Kd in rad per (rad/s). Ki in rad
% per (rad*s).
% Linear-model check (analyze_behavior.m): ~50 deg phase margin and
% ~85 ms delay margin -- room for a slower real servo.
% =====================================================================
Kp = 0.60; Ki = 0.18; Kd = 0.105;

% =====================================================================
% SENSOR MODEL (simulated MPU-6050 IMU) -- mirrors the real firmware
% ---------------------------------------------------------------------
% A real IMU can't measure the angle directly. It has two sensors:
%  * ACCELEROMETER: measures gravity's direction -> gives an angle, but
%    it's NOISY (vibration, jolts) every single sample.
%  * GYRO: measures angular RATE (theta_dot) directly and cleanly, but
%    if you integrate it to get angle, small errors pile up (drift).
% So we use each for what it's good at (see the control loop below).
% Noise values are [EST] from MPU-6050 datasheet noise densities at our
% 2 kHz sample rate, rounded up; EDF vibration will make the real
% accelerometer worse -- measure it.
% =====================================================================
sigma_acc  = 0.02;    % [rad]   accelerometer-angle noise, std dev per sample (~1.1 deg)
sigma_gyro = 0.003;   % [rad/s] gyro rate noise, std dev per sample (~0.17 deg/s)
tau_comp   = 0.5;     % [s] complementary-filter time constant (same as firmware COMP_TAU)
k_comp = tau_comp / (tau_comp + dt);
% Complementary-filter weight: each step, trust the gyro-propagated angle
% by k_comp (~0.999) and the accelerometer by (1 - k_comp) (~0.001).

tau_filter  = 0.02;   % [s] extra low-pass on the angle used by P and I (same as firmware)
alpha = dt / (tau_filter + dt);
% Discrete version of the low-pass filter 1/(tau*s + 1): each step moves
% the filtered value a fraction alpha of the way toward the new reading.
% Must be computed AFTER dt has its final value.

% =====================================================================
% ACTUATOR (gimbal servo)
% =====================================================================
fan_tilt_max_deg = 35;
% Physical limit on how far the fan/servo can gimbal, in degrees. Used
% as the actuator saturation (both modes) AND as the dial's range.
Umax_actuator = deg2rad(fan_tilt_max_deg);
% The controller output u IS the fan tilt angle in radians, so the
% biggest u the servo can deliver is simply +/-35 deg in radians. (An
% earlier version used u = fan_tilt/200 and capped u at 0.175 -- that
% gave the fan only sin(10 deg) worth of torque instead of sin(35 deg),
% i.e. ~3x too weak.)

sim_speed = 0.25;
% Playback speed: 1 = real time, 0.25 = 4x slow motion. Changeable live
% from the dropdown in the window. The animation is paced to the wall
% clock, so this is a TRUE speed no matter how fast the computer draws.
% NOTE: this plant tips over on a ~175 ms timescale (1/sqrt(a), sqrt(a)
% ~= 5.7 rad/s) -- about human reaction time (~200 ms), and the hard
% limit for any delayed PD balancer is sqrt(2)/sqrt(a) ~= 250 ms. At 1x,
% flying it by hand with the dial is very hard -- that's the point of
% the demo (TVC needs a computer). Slow motion makes manual mode playable.

dial_step_deg = 2;
% How far one press of the Left/Right (or A/D) key turns the dial.

% --- Mechanical stops (pins) ---
% Real hardware will have a physical limit -- pins or a bumper -- that
% stop the body from tipping past a set angle. The fan's best torque is
% b*sin(35 deg); gravity's is a*sin(theta). Past the angle where gravity
% wins, asin(b*sin(Umax)/a), NO controller (PID or human) can recover,
% so the stop angle must sit inside that limit. With the current guesses
% b*sin(35)/a ~= 2, i.e. the fan beats gravity at ANY angle (reported as
% 90 deg).
max_recoverable_deg = asind(min(1, b * sin(Umax_actuator) / a));
if max_recoverable_deg >= 89.9
    recover_msg = 'Fan can beat gravity at any angle.';
else
    recover_msg = sprintf('Fan can only recover up to %.0f deg.', max_recoverable_deg);
end
theta_limit_deg = 30;   % keep this well under max_recoverable_deg
theta_limit = deg2rad(theta_limit_deg);

% =====================================================================
% SIMULATION STATE
% =====================================================================

theta      = 0.2;   % true body angle (rad), 0 = upright. Start tipped so you see the recovery.
theta_dot  = 0.0;    % angular velocity (rad/s)
theta_est  = theta;  % complementary-filter angle estimate. Starts at the true
                     % angle, as if the filter had already settled (the real
                     % firmware initializes from the accelerometer the same way).
theta_filt = theta;  % low-passed estimate used by P and I
integral_term = 0.0; % running integral of the error (rad*s)
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
% ---------------------------------------------------------------------
% Every physics step (dt = 0.5 ms) runs the same chain a real flight
% computer runs, then advances the physics:
%
%   1. SENSE     simulated IMU: noisy accelerometer angle + gyro rate
%   2. ESTIMATE  complementary filter fuses them into one clean angle
%   3. SMOOTH    a light low-pass on that angle (for the P and I terms)
%   4. ERROR     how far we are from where we want to be (ref - angle)
%   5. PID       P: push back in proportion to the lean
%                I: push back in proportion to how long it's leaned
%                D: resist motion, using the gyro's measured rate
%   6. SATURATE  the servo can't tilt the fan past +/-35 deg
%   7. ACTUATE   fan tilt -> sideways thrust -> torque about the pivot
%   8. PHYSICS   add gravity's torque, integrate angle and rate
%   9. STOPS     mechanical pins stop the body at +/-30 deg
%
% In MANUAL mode, steps 4-5 are replaced by the dial, but steps 1-3 keep
% running so the autopilot has a good angle estimate the moment you
% hand control back.
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
        integral_term = 0.0;   % don't let the autopilot inherit a stale integral
                               % from whatever the rocket was doing under manual control
        dial.Enable = 'off'; centerBtn.Enable = 'off';
        modeSwitch.Value = 'AUTO';
    end
    wasManualMode = manualMode;

    % --- Disturbances: the Kick button (any mode), plus automatic ---
    % --- kicks in AUTO mode only, so they don't fight the user's input ---
    % A kick is a sudden shove: it changes the rotation SPEED (theta_dot)
    % instantly, and the angle follows. (Teleporting the angle instead, as
    % an earlier version did, is physically impossible -- and the gyro,
    % which senses motion, would never see it, so the estimator would be
    % fooled.) 1.5-2.5 rad/s swings the body roughly 6-10 deg.
    if getappdata(fig, 'kickRequest')
        theta_dot = theta_dot + sign(randn) * (1.5 + 1.0*rand);
        setappdata(fig, 'kickRequest', false);
    end
    if ~manualMode && sim_time >= next_kick_time
        theta_dot = theta_dot + sign(randn) * (1.5 + 1.0*rand);
        next_kick_time = sim_time + kick_interval_s;
    elseif manualMode
        next_kick_time = sim_time + kick_interval_s;
    end

    % --- Real-time pacing: simulate exactly as much time as actually ---
    % --- passed on the wall clock (times sim_speed) since last frame. ---
    wall_dt = toc(frameClock); frameClock = tic;
    nsub = min(max_substeps, max(1, round(wall_dt * sim_speed / dt)));

    % Manual command from the dial: dial angle = fan tilt in degrees.
    % The fan is drawn at -u (see the sign note below the loop), so the
    % dial setting maps to u = -dial, in radians.
    manual_u = -deg2rad(getappdata(fig, 'dialDeg'));

    for k = 1:nsub

        % ---- 1. SENSE: what the IMU reports this sample ----
        % Each reading = truth + random noise (randn = standard normal).
        acc_angle  = theta     + sigma_acc  * randn;   % accelerometer's angle (noisy)
        gyro_rate  = theta_dot + sigma_gyro * randn;   % gyro's angular rate (clean)

        % ---- 2. ESTIMATE: complementary filter ----
        % Predict the new angle by integrating the gyro (angle += rate*dt),
        % then nudge it a tiny bit (weight 1 - k_comp ~= 0.001) toward the
        % accelerometer's angle. Fast changes come from the gyro (clean,
        % no lag); the slow, long-term level comes from the accelerometer
        % (no drift). Accelerometer noise gets averaged over ~tau_comp =
        % 0.5 s, so it all but disappears.
        theta_est = k_comp * (theta_est + gyro_rate * dt) + (1 - k_comp) * acc_angle;

        % ---- 3. SMOOTH: low-pass for the P and I terms ----
        % theta_filt moves a fraction alpha of the way toward theta_est
        % each step: a first-order filter with time constant tau_filter.
        theta_filt = theta_filt + alpha * (theta_est - theta_filt);

        if manualMode
            u = manual_u;            % you are the controller
        else
            % ---- 4. ERROR ----
            error = ref - theta_filt;   % + when leaning left of target, - when right

            % ---- 5. PID ----
            % P term: Kp*error. Leaning 10 deg -> fan tilts 0.6*10 = 6 deg
            %   back the other way. This is the "spring" that fights gravity.
            % I term: Ki*integral of error over time. If something keeps it
            %   leaning (a steady push, an off-center CG), this term keeps
            %   growing until the lean is gone.
            integral_term = integral_term + error * dt;
            % D term: Kd*(rate of change of error). The target is fixed, so
            %   d(error)/dt = -theta_dot, and the gyro MEASURES theta_dot
            %   directly. Using it (instead of differencing the noisy angle,
            %   which amplifies noise by 1/dt = 2000x) gives clean damping:
            %   the "shock absorber" that stops it swinging past upright.
            derivative = -gyro_rate;

            u = Kp*error + Ki*integral_term + Kd*derivative;
        end

        % ---- 6. SATURATE: the servo can't exceed +/-35 deg ----
        % Applies in BOTH modes: a real servo has a hard limit no matter
        % who or what is commanding it.
        u = max(-Umax_actuator, min(Umax_actuator, u));
        u_last = u;

        % ---- 7. ACTUATE: fan tilt -> torque -> angular acceleration ----
        % Only the sideways part of the thrust, F_t*sin(u), makes torque.
        accel_cmd = b * sin(u);

        % ---- 8. PHYSICS: semi-implicit ("symplectic") Euler integration ----
        % (same physics either way -- AUTO and MANUAL only differ in
        % how "u" gets computed above). Uses sin(theta), not theta, for
        % the gravity torque -- theta routinely leaves the small-angle
        % range in MANUAL mode, and the small-angle version grows
        % without bound instead of saturating like the real torque does.
        theta_ddot = accel_cmd + a * sin(theta);
        theta_dot  = theta_dot + theta_ddot * dt;   % update velocity first...
        theta      = theta + theta_dot * dt;         % ...then use the NEW velocity for angle

        % ---- 9. STOPS: body rests against the pins, doesn't bounce ----
        % (an inelastic contact, same as a real pin/bumper)
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
    % NEGATIVE of u here (theta_ddot itself is still b*sin(u), unchanged).
    % u is already in radians and already clamped to +/-35 deg, so this is
    % the true fan angle, not a scaled-up picture of it.
    fan_tilt_rad = -u_last;
    fan_tilt_deg = rad2deg(fan_tilt_rad);

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
                            'Stops at +/-%g deg. %s'], ...
                           rad2deg(theta), rad2deg(theta_dot), fan_tilt_deg, sim_time, sim_speed, ...
                           theta_limit_deg, recover_msg);

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

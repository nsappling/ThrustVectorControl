function out = tvc_sim(p, T)
%% tvc_sim.m
% Headless (no graphics) version of the AUTO-mode physics + PID loop in
% rocketgimball.m. Runs for a fixed time T and returns the full time
% history, so the controller can be tested/tuned from a script or with
% `matlab -batch` (rocketgimball.m itself loops forever on a figure).
%
% Usage:
%   out = tvc_sim();                    % defaults = rocketgimball.m values, 3 s
%   out = tvc_sim(struct('Kp',0.6), 5); % override any parameter, run 5 s
%
% Any field left out of p falls back to the defaults in tvc_defaults()
% below. KEEP THOSE IN SYNC with rocketgimball.m if you change the
% parameters there.
%
% Output struct fields:
%   t, theta, theta_dot, theta_est, theta_filt, u -- time histories
%                             (column vectors; rad, rad/s, rad, rad, and
%                             u = fan tilt in rad -- drawn fan angle is -u)
%   p                         -- the full parameter set actually used
%   I, a, b, Umax, theta_limit -- derived constants (same meaning as rocketgimball.m)
%   hit_stop                  -- true if the body ever reached the mechanical stops

if nargin < 1 || isempty(p), p = struct(); end
if nargin < 2 || isempty(T), T = 3; end
p = fill_defaults(p);

% --- Derived constants (identical formulas to rocketgimball.m) ---
% Inertia: uniform solid cylinder (L_body x r_body) pivoting r_0 below its
% center -- same model as rocketgimball.m (parallel-axis theorem).
I = p.m*(3*p.r_body^2 + p.L_body^2)/12 + p.m*p.r_0^2;
a = p.m * p.g * p.r_0 / I;                  % inverted-pendulum "gravity" term
b = p.F_t * p.R / I;                        % control effectiveness, must be > 0
alpha = p.dt / (p.tau_filter + p.dt);       % discrete low-pass filter coefficient
k_comp = p.tau_comp / (p.tau_comp + p.dt);  % complementary-filter weight on the gyro path
Umax = deg2rad(p.fan_tilt_max_deg);         % actuator saturation: u is fan tilt in rad
if ~p.noise, p.sigma_acc = 0; p.sigma_gyro = 0; end
theta_limit = deg2rad(p.theta_limit_deg);   % mechanical stop angle

% Private random stream so runs are repeatable (same seed -> same noise)
% and so this function doesn't disturb the global rng.
rs = RandStream('twister', 'Seed', p.seed);

N = round(T / p.dt);
t          = (0:N)' * p.dt;
theta_h    = zeros(N+1, 1);
thdot_h    = zeros(N+1, 1);
est_h      = zeros(N+1, 1);
filt_h     = zeros(N+1, 1);
u_h        = zeros(N+1, 1);

% --- Initial state (same as rocketgimball.m) ---
theta = p.theta0;
theta_dot = 0.0;
theta_est = theta;    % estimator starts converged (as the firmware does)
theta_filt = theta;
integral_term = 0.0;
hit_stop = false;

% Disturbance kicks: rows of [time_s, delta_theta_dot_rad_per_s], applied
% as an instantaneous change in rotation speed (a shove), same as the
% kicks in the demo.
kick_steps = [];
if ~isempty(p.kicks)
    kick_steps = round(p.kicks(:,1) / p.dt);
end

theta_h(1) = theta; est_h(1) = theta; filt_h(1) = theta;
for k = 1:N
    j = find(kick_steps == k);
    if ~isempty(j)
        theta_dot = theta_dot + sum(p.kicks(j,2));
    end

    % Same control chain as rocketgimball.m -- see the step-by-step
    % comments there. Steps 1-9:
    % 1. SENSE: noisy accelerometer angle, clean gyro rate
    acc_angle = theta     + p.sigma_acc  * randn(rs);
    gyro_rate = theta_dot + p.sigma_gyro * randn(rs) + p.gyro_bias;

    % 2. ESTIMATE: complementary filter (gyro for fast changes, accel for
    %    the long-term level)
    theta_est = k_comp * (theta_est + gyro_rate * p.dt) + (1 - k_comp) * acc_angle;

    % 3. SMOOTH: low-pass for P and I
    theta_filt = theta_filt + alpha * (theta_est - theta_filt);

    % 4-5. ERROR and PID (D uses the measured rate: d(error)/dt = -theta_dot)
    err = p.ref - theta_filt;
    integral_term = integral_term + err * p.dt;
    u = p.Kp*err + p.Ki*integral_term + p.Kd*(-gyro_rate);

    % 6. SATURATE at +/- fan_tilt_max_deg
    u = max(-Umax, min(Umax, u));

    % 7-8. ACTUATE + PHYSICS: semi-implicit Euler; fan torque b*sin(u),
    %      full sin(theta) gravity torque, optional disturbance
    theta_ddot = b*sin(u) + a*sin(theta) + dist_at(p.dist_accel, t(k));
    theta_dot  = theta_dot + theta_ddot * p.dt;
    theta      = theta + theta_dot * p.dt;

    % 9. STOPS: inelastic contact
    if abs(theta) >= theta_limit
        theta = sign(theta) * theta_limit;
        theta_dot = 0;
        hit_stop = true;
    end

    theta_h(k+1) = theta;
    thdot_h(k+1) = theta_dot;
    est_h(k+1)   = theta_est;
    filt_h(k+1)  = theta_filt;
    u_h(k+1)     = u;
end

out = struct('t', t, 'theta', theta_h, 'theta_dot', thdot_h, 'theta_est', est_h, ...
             'theta_filt', filt_h, 'u', u_h, ...
             'p', p, 'I', I, 'a', a, 'b', b, 'Umax', Umax, 'theta_limit', theta_limit, ...
             'hit_stop', hit_stop);
end

% =====================================================================
% LOCAL FUNCTIONS
% =====================================================================

function d = tvc_defaults()
    % Mirrors the parameter block at the top of rocketgimball.m.
    % Physical values (F_t, R, m, r_0) are still GUESSES -- update both
    % files once they're measured.
    d.dt = 0.0005;          % [s] physics timestep
    d.F_t = 6.87;           % [N] fan thrust
    d.R = 0.15;             % [m] pivot to fan
    d.m = 1;                % [kg] total mass
    d.r_0 = 0.03;           % [m] pivot to CG
    d.L_body = 12 * 0.0254; % [m] body length   (CAD Body_Height)  -- for I,
    d.r_body = 3.5/2 * 0.0254; % [m] body radius (CAD Body_OD / 2)  uniform cylinder
    d.g = 9.81;             % [m/s^2]
    d.Kp = 0.60; d.Ki = 0.18; d.Kd = 0.105;   % pole placement, wn = 6, zeta = 1
    d.noise = true;         % false = turn off all sensor noise (clean metrics)
    d.sigma_acc = 0.02;     % [rad] accelerometer-angle noise per sample [EST]
    d.sigma_gyro = 0.003;   % [rad/s] gyro rate noise per sample [EST]
    d.gyro_bias = 0;        % [rad/s] constant gyro offset (0 = perfectly calibrated)
    d.tau_comp = 0.5;       % [s] complementary-filter time constant
    d.tau_filter = 0.02;    % [s] low-pass on the angle used by P and I
    d.fan_tilt_max_deg = 35;% [deg] gimbal limit; u is fan tilt in rad
    d.theta_limit_deg = 30; % mechanical stop angle
    d.theta0 = 0.2;         % [rad] initial tip
    d.ref = 0.0;            % [rad] target angle
    d.kicks = [];           % [time_s, delta_theta_dot (rad/s)] rows
    d.dist_accel = 0;       % [rad/s^2] disturbance angular acceleration: a
                            % constant, or a function handle @(t) (e.g. a
                            % steady push/wind torque divided by inertia)
    d.seed = 1;             % noise seed
end

function x = dist_at(d, t)
    % Disturbance angular acceleration at time t (constant or @(t)).
    if isa(d, 'function_handle'), x = d(t); else, x = d; end
end

function p = fill_defaults(p)
    d = tvc_defaults();
    f = fieldnames(d);
    for i = 1:numel(f)
        if ~isfield(p, f{i})
            p.(f{i}) = d.(f{i});
        end
    end
    unknown = setdiff(fieldnames(p), f);
    if ~isempty(unknown)
        error('tvc_sim:unknownParam', 'Unknown parameter(s): %s', strjoin(unknown, ', '));
    end
end

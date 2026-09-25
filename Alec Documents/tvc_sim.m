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
%   t, theta, theta_filt, u   -- time histories (column vectors, rad / rad / u)
%   p                         -- the full parameter set actually used
%   a, b, Umax, theta_limit   -- derived constants (same meaning as rocketgimball.m)
%   hit_stop                  -- true if the body ever reached the mechanical stops

if nargin < 1 || isempty(p), p = struct(); end
if nargin < 2 || isempty(T), T = 3; end
p = fill_defaults(p);

% --- Derived constants (identical formulas to rocketgimball.m) ---
a = p.g / p.r_0;                            % inverted-pendulum "gravity" term
b = p.F_t * p.R / (p.m * p.r_0^2);          % control effectiveness, must be > 0
alpha = p.dt / (p.tau_filter + p.dt);       % discrete low-pass filter coefficient
Umax = p.fan_tilt_max_deg / p.VIS_SCALE;    % actuator saturation on u
theta_limit = deg2rad(p.theta_limit_deg);   % mechanical stop angle

% Private random stream so runs are repeatable (same seed -> same noise)
% and so this function doesn't disturb the global rng.
rs = RandStream('twister', 'Seed', p.seed);

N = round(T / p.dt);
t          = (0:N)' * p.dt;
theta_h    = zeros(N+1, 1);
filt_h     = zeros(N+1, 1);
u_h        = zeros(N+1, 1);

% --- Initial state (same as rocketgimball.m) ---
theta = p.theta0;
theta_dot = 0.0;
theta_filt = 0.0;
integral_term = 0.0;
prev_error = [];
hit_stop = false;

% Disturbance kicks: rows of [time_s, delta_theta_rad], applied as an
% instantaneous bump in angle (same as the periodic kicks in the demo).
kick_steps = [];
if ~isempty(p.kicks)
    kick_steps = round(p.kicks(:,1) / p.dt);
end

theta_h(1) = theta;
for k = 1:N
    j = find(kick_steps == k);
    if ~isempty(j)
        theta = theta + sum(p.kicks(j,2));
    end

    % --- Simulated IMU measurement + low-pass filter ---
    measured_theta = theta + p.sigma_noise * randn(rs);
    theta_filt = theta_filt + alpha * (measured_theta - theta_filt);

    % --- PID controller ---
    err = p.ref - theta_filt;
    if isempty(prev_error)
        prev_error = err;   % avoid a derivative-kick on the very first sample
    end
    integral_term = integral_term + err * p.dt;
    derivative = (err - prev_error) / p.dt;
    prev_error = err;
    u = p.Kp*err + p.Ki*integral_term + p.Kd*derivative;

    % --- Actuator saturation ---
    u = max(-Umax, min(Umax, u));

    % --- Physics: semi-implicit Euler, full sin(theta) gravity torque ---
    theta_ddot = b*u + a*sin(theta);
    theta_dot  = theta_dot + theta_ddot * p.dt;
    theta      = theta + theta_dot * p.dt;

    % --- Mechanical stops: inelastic contact ---
    if abs(theta) >= theta_limit
        theta = sign(theta) * theta_limit;
        theta_dot = 0;
        hit_stop = true;
    end

    theta_h(k+1) = theta;
    filt_h(k+1)  = theta_filt;
    u_h(k+1)     = u;
end

out = struct('t', t, 'theta', theta_h, 'theta_filt', filt_h, 'u', u_h, ...
             'p', p, 'a', a, 'b', b, 'Umax', Umax, 'theta_limit', theta_limit, ...
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
    d.g = 9.81;             % [m/s^2]
    d.Kp = 0.42; d.Ki = 0.1; d.Kd = 0.015;
    d.sigma_noise = 0.02;   % [rad] IMU noise std dev
    d.tau_filter = 0.02;    % [s] low-pass filter time constant
    d.VIS_SCALE = 200;      % sets Umax = fan_tilt_max_deg / VIS_SCALE
    d.fan_tilt_max_deg = 35;
    d.theta_limit_deg = 30; % mechanical stop angle
    d.theta0 = 0.2;         % [rad] initial tip
    d.ref = 0.0;            % [rad] target angle
    d.kicks = [];           % [time_s, delta_theta_rad] rows
    d.seed = 1;             % noise seed
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

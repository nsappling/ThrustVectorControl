function r = tvc_linear_report(p, quiet)
%TVC_LINEAR_REPORT  Hand-calc style design numbers from the linearized model.
%
%   r = tvc_linear_report(p)        prints a summary and returns a struct
%   r = tvc_linear_report(p, true)  returns the struct without printing
%
%   No toolboxes needed: the closed loop with servo lag and loop latency is
%   checked by rooting the characteristic polynomial, with the latency
%   approximated by a 2nd-order Pade.

if nargin < 2, quiet = false; end

T  = edf_thrust(p, p.u_fixed);
a  = p.m*p.g*p.h_cg/p.I;           % theta_dd = a*theta + b*delta
b  = T*p.L/p.I;
r.thrust_N          = T;
r.open_loop_pole    = sqrt(a);     % [rad/s] unstable pole
r.time_to_double_s  = log(2)/sqrt(a);
r.fall_1deg_to_stop = acosh(p.theta_stop/deg2rad(1))/sqrt(a);

% ---- control authority -----------------------------------------------
tau_avail = T*p.L*sin(p.delta_cmd_max);
tau_grav  = p.m*p.g*p.h_cg*sin(p.theta_stop);
r.authority_ratio = tau_avail/tau_grav;      % >1 means it can pull back from the stop
r.max_recoverable_deg = rad2deg(asin(min(1, tau_avail/(p.m*p.g*p.h_cg))));
for k = [1 1.5]
    Treq = k*tau_grav/(p.L*sin(p.delta_cmd_max));
    r.(sprintf('min_throttle_ratio_%s', strrep(num2str(k),'.','p'))) = invert_thrust(p, Treq);
end

% ---- gains --------------------------------------------------------------
r.Kp = p.Kp;  r.Kd = p.Kd;
r.Kp_min = a/b;                              % below this it falls over
r.wn_cl  = sqrt(max(b*p.Kp - a, 0));
r.zeta_cl = b*p.Kd/(2*max(r.wn_cl, eps));
r.saturates_at_err_deg = rad2deg(p.delta_cmd_max/p.Kp);

% ---- latency budget ---------------------------------------------------
dtc = 1/p.loop_rate;
r.lat.sample_hold_ms   = 1e3*0.5*dtc;
r.lat.compute_ms       = 1e3*dtc*p.one_sample_delay;
r.lat.servo_frame_ms   = 1e3*0.5/p.servo_refresh;
r.lat.d_filter_ms      = 1e3/(2*pi*p.d_lpf_hz);
r.lat.total_delay_ms   = r.lat.sample_hold_ms + r.lat.compute_ms + ...
                         r.lat.servo_frame_ms + r.lat.d_filter_ms;
r.lat.servo_lag_ms     = 1e3*(1/p.servo_kpos + p.servo_tau);
r.gimbal_full_swing_ms = 1e3*2*p.delta_max/p.servo_rate;

tau_d = r.lat.total_delay_ms/1e3;
tau_s = r.lat.servo_lag_ms/1e3;
cl = @(extra) char_roots(a, b, p.Kp, p.Kd, tau_s, tau_d + extra);
rts = cl(0);
r.cl_poles = rts;
r.stable_with_latency = all(real(rts) < 0);
% extra pure delay the loop can absorb before going unstable
if r.stable_with_latency
    lo = 0; hi = 0.5;
    for i = 1:40
        mid = (lo + hi)/2;
        if all(real(cl(mid)) < 0), lo = mid; else, hi = mid; end
    end
    r.delay_margin_ms = 1e3*lo;
else
    r.delay_margin_ms = 0;
end

% ---- exhibit difficulty -----------------------------------------------
% For PD balancing of an inverted pendulum with reaction delay tau, no
% gains can stabilize it once sqrt(a)*tau > sqrt(2) (Stepan). A human
% pilot with delay p.human_delay is at this fraction of that limit:
r.human_delay_limit_s = sqrt(2)/sqrt(a);
r.human_difficulty    = p.human_delay/r.human_delay_limit_s;

if quiet, return; end
fprintf('\n=== TVC linear design report ===================================\n');
fprintf('Thrust at u=%.2f            : %.2f N\n', p.u_fixed, T);
fprintf('Open-loop unstable pole      : %.2f rad/s  (doubles every %.0f ms)\n', r.open_loop_pole, 1e3*r.time_to_double_s);
fprintf('Fall time 1 deg -> stop      : %.2f s\n', r.fall_1deg_to_stop);
fprintf('Authority ratio at the stop  : %.2f  (TVC torque / gravity torque, want >= 1.5)\n', r.authority_ratio);
fprintf('Max angle it can pull back from: %.1f deg\n', r.max_recoverable_deg);
fprintf('Min throttle for ratio 1.0 / 1.5: %.2f / %.2f\n', r.min_throttle_ratio_1, r.min_throttle_ratio_1p5);
fprintf('Kp = %.3f rad/rad (min %.3f),  Kd = %.3f s\n', p.Kp, r.Kp_min, p.Kd);
fprintf('Ideal closed loop            : wn = %.1f rad/s, zeta = %.2f\n', r.wn_cl, r.zeta_cl);
fprintf('Gimbal saturates at error    : %.1f deg\n', r.saturates_at_err_deg);
fprintf('Latency: hold %.1f + compute %.1f + servo frame %.1f + D-filter %.1f = %.1f ms\n', ...
    r.lat.sample_hold_ms, r.lat.compute_ms, r.lat.servo_frame_ms, r.lat.d_filter_ms, r.lat.total_delay_ms);
fprintf('Servo lag %.1f ms, full gimbal swing %.0f ms\n', r.lat.servo_lag_ms, r.gimbal_full_swing_ms);
if r.stable_with_latency
    fprintf('Linear loop with latency     : STABLE, delay margin %.0f ms\n', r.delay_margin_ms);
else
    fprintf('Linear loop with latency     : UNSTABLE - lower wn or cut latency\n');
end
fprintf('Human reaction-delay limit   : %.2f s  (pilot at %.0f%% of the limit)\n', ...
    r.human_delay_limit_s, 100*r.human_difficulty);
fprintf('================================================================\n\n');
end

function rts = char_roots(a, b, Kp, Kd, tau_s, tau_d)
% (s^2 - a)(tau_s s + 1) D(s) + b (Kd s + Kp) N(s) = 0, Pade(2) delay N/D
Np = [tau_d^2/12, -tau_d/2, 1];
Dp = [tau_d^2/12,  tau_d/2, 1];
lhs = conv(conv([1 0 -a], [tau_s 1]), Dp);
rhs = b*conv([Kd Kp], Np);
rhs = [zeros(1, numel(lhs) - numel(rhs)), rhs];
rts = roots(lhs + rhs);
end

function u = invert_thrust(p, Treq)
uu = linspace(0, 1, 2001);
TT = edf_thrust(p, uu);
i  = find(TT >= Treq, 1);
if isempty(i), u = NaN; else, u = uu(i); end
end

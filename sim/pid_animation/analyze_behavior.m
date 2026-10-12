%% analyze_behavior.m
% Maps out how the rocketgimball.m system behaves, with figures and a
% printed summary: how fast it falls with no control, closed-loop poles
% and time constants, overshoot / rise / settling times, stability
% margins, disturbance rejection, large-angle (saturated) behavior, and
% how the poles move with the gains.
%
% Uses the same plant, filter, PID, saturation and stops as
% rocketgimball.m (through tvc_sim.m, whose defaults mirror it), plus
% the linearized model for the textbook numbers. Safe to run headless:
%
%   matlab -batch "run('sim/pid_animation/analyze_behavior.m')"
%
% Figures are saved to sim/pid_animation/results/behavior/ and a text
% summary to summary.txt in the same folder. Needs the Control System
% Toolbox (tf, pid, damp, stepinfo, rlocus, allmargin).
%
% REMEMBER: F_t, R, m, r_0 are still GUESSES, so every number below is
% only as good as those. Re-run once they're measured.

clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(here);
outdir = fullfile(here, 'results', 'behavior');
if ~exist(outdir, 'dir'), mkdir(outdir); end
vis = 'on';
if batchStartupOptionUsed, vis = 'off'; end   % -batch: save figures, don't show them

% =====================================================================
% PARAMETERS (pulled from tvc_sim.m so they can't drift apart)
% =====================================================================

base = tvc_sim(struct(), 0.001);
p = base.p; a = base.a; b = base.b; Umax = base.Umax;
lim_deg = p.theta_limit_deg;
quiet = struct('sigma_noise', 0);   % noise-free runs, for clean textbook metrics

S = {};   % summary lines, printed and saved at the end
S{end+1} = sprintf('rocketgimball.m behavior summary   (%s)', string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm')));
S{end+1} = sprintf('Gains: Kp = %.3f, Ki = %.3f, Kd = %.4f;  filter tau = %.3f s;  IMU noise = %.3f rad', ...
                   p.Kp, p.Ki, p.Kd, p.tau_filter, p.sigma_noise);
S{end+1} = sprintf('Plant: a = g/r_0 = %.1f 1/s^2, b = F_t*R/(m*r_0^2) = %.1f, fan limit = +/-%g deg (Umax = %.3f)', ...
                   a, b, p.fan_tilt_max_deg, Umax);
S{end+1} = '  (F_t, R, m, r_0 are ESTIMATES -- all numbers below depend on them)';

% =====================================================================
% LINEAR MODEL (small angles: sin(theta) ~ theta)
%   Plant        theta'' = a*theta + b*u + d     G  = b/(s^2 - a),  Gd = 1/(s^2 - a)
%   IMU filter   1/(tau*s + 1)                    F
%   PID          Kp + Ki/s + Kd*s                 C  (acts on ref - filtered theta)
%   Loop gain    L = C*F*G
%   ref -> theta T = C*G/(1 + L)     disturbance -> theta  Td = Gd/(1 + L)
% =====================================================================

s = tf('s');
G  = b / (s^2 - a);
Gd = 1 / (s^2 - a);
F  = 1 / (p.tau_filter*s + 1);
C  = pid(p.Kp, p.Ki, p.Kd);
L  = C * F * G;
T  = minreal(C * G / (1 + L));
Td = minreal(Gd / (1 + L));
CL = feedback(L, 1);           % same poles as T and Td

% =====================================================================
% 1. OPEN LOOP: how fast does it fall with NO control?
% =====================================================================

lam = sqrt(a);                 % unstable pole of the plant, 1/s
tau_u = 1 / lam;               % instability time constant
t_double = log(2) / lam;
th0_ol = deg2rad(1);
t_stop_lin = acosh(deg2rad(lim_deg) / th0_ol) / lam;

ol = tvc_sim(struct('Kp', 0, 'Ki', 0, 'Kd', 0, 'sigma_noise', 0, 'theta0', th0_ol), 0.6);
i_stop = find(abs(ol.theta) >= deg2rad(lim_deg) - 1e-9, 1);
t_stop_nl = ol.t(i_stop);

f1 = figure('Visible', vis, 'Position', [50 50 800 450]);
plot(ol.t, rad2deg(ol.theta), 'LineWidth', 1.5); hold on; grid on;
plot(ol.t, rad2deg(th0_ol*cosh(lam*ol.t)), '--', 'LineWidth', 1.2);
yline(lim_deg, ':k', sprintf('stop (%g deg)', lim_deg));
xline(t_stop_nl, ':', sprintf('hits stop at %.0f ms', 1000*t_stop_nl));
ylim([0 lim_deg*1.15]); xlim([0 ol.t(i_stop)*1.6]);
xlabel('time [s]'); ylabel('\theta [deg]');
legend('nonlinear sim', 'linear: \theta_0 cosh(\surd{a}\,t)', 'Location', 'northwest');
title(sprintf('No control: unstable pole s = +%.1f 1/s,  time constant %.0f ms,  doubles every %.0f ms', ...
              lam, 1000*tau_u, 1000*t_double));
exportgraphics(f1, fullfile(outdir, '1_open_loop_fall.png'));

S{end+1} = '';
S{end+1} = '--- 1. No control (open loop) ---';
S{end+1} = sprintf('Unstable pole: +%.1f 1/s  ->  time constant %.1f ms, angle doubles every %.1f ms', lam, 1000*tau_u, 1000*t_double);
S{end+1} = sprintf('From a 1 deg tip it hits the %g deg stop in %.0f ms (linear model: %.0f ms).', lim_deg, 1000*t_stop_nl, 1000*t_stop_lin);
S{end+1} = sprintf('Human reaction time is ~200 ms = %.1f time constants -> why hand control (MANUAL mode) is so hard.', 0.2/tau_u);
S{end+1} = sprintf('Fan authority: fan torque beats gravity up to %.1f deg (stops at %g deg).', asind(min(1, b*Umax/a)), lim_deg);
S{end+1} = sprintf('Minimum Kp for stability: Kp > a/b = %.3f (current Kp = %.3f, %.1fx the minimum).', a/b, p.Kp, p.Kp/(a/b));

% =====================================================================
% 2. CLOSED-LOOP POLES AND TIME CONSTANTS
% =====================================================================

[wn, zeta, poles] = damp(CL);
tc = -1 ./ real(poles);        % time constant of each pole

f2 = figure('Visible', vis, 'Position', [50 50 800 550]);
pzplot(CL); grid on; hold on;
title('Closed-loop poles and zeros (with time constants)');
for k = 1:numel(poles)
    if imag(poles(k)) >= 0
        % Poles near the origin get their label on the left so it stays on the plot
        if real(poles(k)) > -0.15*max(abs(real(poles))), ha = 'right'; else, ha = 'left'; end
        text(real(poles(k)), imag(poles(k)), sprintf('  \\tau = %.0f ms, \\zeta = %.2f  ', 1000*tc(k), zeta(k)), ...
             'FontSize', 9, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', ha);
    end
end
xlim([-1.15*max(abs(real(poles))), 1.3*lam]);
plot(lam, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
text(lam, 0, 'open-loop unstable pole (no control)  ', 'Color', 'r', 'FontSize', 9, ...
     'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
exportgraphics(f2, fullfile(outdir, '2_closed_loop_poles.png'));

S{end+1} = '';
S{end+1} = '--- 2. Closed-loop poles (linear model) ---';
S{end+1} = sprintf('%-24s %12s %12s %8s', 'pole [1/s]', 'tau [ms]', 'wn [rad/s]', 'zeta');
[~, order] = sort(abs(real(poles)));
for k = order'
    if imag(poles(k)) < 0, continue; end
    if abs(imag(poles(k))) > 1e-6
        ps = sprintf('%.1f +/- %.1fj', real(poles(k)), imag(poles(k)));
    else
        ps = sprintf('%.1f', real(poles(k)));
    end
    S{end+1} = sprintf('%-24s %12.1f %12.1f %8.2f', ps, 1000*tc(k), wn(k), zeta(k)); %#ok<SAGROW>
end
S{end+1} = sprintf('Slowest pole sets the final tail: tau = %.0f ms (4*tau = %.2f s to die out).', 1000*max(tc), 4*max(tc));

% =====================================================================
% 3. STABILITY MARGINS (loop gain L = C*F*G)
% =====================================================================

mg = allmargin(L);
[~, ipm] = min(mg.PhaseMargin);
PM = mg.PhaseMargin(ipm); wc = mg.PMFrequency(ipm);
DM = deg2rad(PM) / wc;          % extra pure delay the loop can tolerate

f3 = figure('Visible', vis, 'Position', [50 50 800 600]);
opts = bodeoptions; opts.FreqUnits = 'rad/s'; opts.Grid = 'on';
bodeplot(L, opts, {0.5, 2000});
title(sprintf('Loop gain L = C F G:  phase margin %.1f deg at %.1f rad/s,  delay margin %.1f ms', PM, wc, 1000*DM));
exportgraphics(f3, fullfile(outdir, '3_loop_bode_margins.png'));

S{end+1} = '';
S{end+1} = '--- 3. Stability margins ---';
S{end+1} = sprintf('Phase margin: %.1f deg at %.1f rad/s   (rule of thumb: want 30-60 deg)', PM, wc);
S{end+1} = sprintf('Delay margin: %.1f ms of extra latency (servo lag, IMU delay, loop time) before it goes unstable', 1000*DM);
% The open loop is unstable, so stability needs ENOUGH gain: margins
% below 1 (negative dB) are how far all gains can shrink; above 1, how
% far they can grow.
gm_lo = mg.GainMargin(mg.GainMargin < 1); gm_hi = mg.GainMargin(mg.GainMargin > 1);
if ~isempty(gm_lo)
    S{end+1} = sprintf('Gain margin (lower): all gains can shrink to %.2fx (%.1f dB) before it goes unstable', max(gm_lo), 20*log10(max(gm_lo)));
end
if ~isempty(gm_hi)
    S{end+1} = sprintf('Gain margin (upper): all gains can grow to %.2fx (%.1f dB)', min(gm_hi), 20*log10(min(gm_hi)));
else
    S{end+1} = 'Gain margin (upper): none in the linear model -- in practice noise, servo speed and delay set the limit';
end

% =====================================================================
% 4. RECOVERY FROM A TIP (what the animation shows): noise off and on
% =====================================================================

th0 = p.theta0;
rec  = tvc_sim(quiet, 3);
recN = tvc_sim(struct(), 3);
mr = ic_metrics(rec.t, rec.theta, th0);
mN = ic_metrics(recN.t, recN.theta, th0);
last = recN.t >= recN.t(end) - 1;
jitter_deg = std(recN.u(last)) * p.VIS_SCALE;
rms_deg = rad2deg(rms(recN.theta(last)));

f4 = figure('Visible', vis, 'Position', [50 50 900 650]);
tl = tiledlayout(f4, 2, 1, 'TileSpacing', 'compact');
ax1 = nexttile(tl); hold on; grid on;
plot(recN.t, rad2deg(recN.theta), 'Color', [0.7 0.7 0.7], 'DisplayName', 'with IMU noise');
plot(rec.t, rad2deg(rec.theta), 'b', 'LineWidth', 1.5, 'DisplayName', 'noise-free');
yline([-2 2], ':k', 'HandleVisibility', 'off');
yline(0, 'k', 'HandleVisibility', 'off');
if mr.overshoot_deg > 0
    plot(mr.t_peak, -sign(th0)*mr.overshoot_deg, 'rv', 'MarkerFaceColor', 'r', 'DisplayName', 'overshoot peak');
end
if isfinite(mr.t_settle2)
    xline(mr.t_settle2, '--', 'settled (2%)', 'HandleVisibility', 'off');
end
ylabel('\theta [deg]'); legend('Location', 'northeast');
title(sprintf('Recovery from a %.1f deg tip', rad2deg(th0)));
text(ax1, 0.35, 0.95, sprintf(['Overshoot: %.1f deg (%.0f%%) at %.0f ms\n' ...
     'Recovery time (90%%->10%%): %.0f ms\nSettling: %.0f ms (2%%), %.0f ms (+/-2 deg)\n' ...
     'Decay time constant (envelope): %s'], ...
     mr.overshoot_deg, mr.overshoot_pct, 1000*mr.t_peak, 1000*mr.t_recover, ...
     1000*mr.t_settle2, 1000*mr.t_band2deg, fmt_ms(mr.tau_env)), ...
     'Units', 'normalized', 'VerticalAlignment', 'top', 'BackgroundColor', 'w', 'EdgeColor', [0.8 0.8 0.8]);
xlim([0 1.5]);
ax2 = nexttile(tl); hold on; grid on;
plot(recN.t, recN.u*p.VIS_SCALE, 'Color', [0.7 0.7 0.7]);
plot(rec.t, rec.u*p.VIS_SCALE, 'b', 'LineWidth', 1.5);
yline([-1 1]*p.fan_tilt_max_deg, 'r:', 'fan limit');
ylabel('fan tilt command [deg]'); xlabel('time [s]'); xlim([0 1.5]);
title(sprintf('Actuator: saturated %.0f%% of the time; with noise, steady jitter %.1f deg (std)', ...
              100*mean(abs(rec.u) >= 0.999*Umax), jitter_deg));
linkaxes([ax1 ax2], 'x');
exportgraphics(f4, fullfile(outdir, '4_tip_recovery.png'));

S{end+1} = '';
S{end+1} = sprintf('--- 4. Recovery from a %.1f deg tip (nonlinear sim, noise-free) ---', rad2deg(th0));
S{end+1} = sprintf('Overshoot:            %.2f deg (%.0f%% of the tip) at %.0f ms', mr.overshoot_deg, mr.overshoot_pct, 1000*mr.t_peak);
S{end+1} = sprintf('Recovery time 90->10%%: %.0f ms', 1000*mr.t_recover);
S{end+1} = sprintf('Settling time:        %.0f ms (within 2%% of the tip), %.0f ms (within +/-2 deg)', 1000*mr.t_settle2, 1000*mr.t_band2deg);
S{end+1} = sprintf('Decay time constant:  %s (fit to the oscillation envelope)', fmt_ms(mr.tau_env));
S{end+1} = sprintf('Steady-state error:   %.3f deg', mr.ss_deg);
S{end+1} = sprintf('With IMU noise:       settles (+/-2 deg) in %.0f ms; then %.2f deg RMS angle, %.1f deg RMS fan jitter', ...
                   1000*mN.t_band2deg, rms_deg, jitter_deg);

% =====================================================================
% 5. SETPOINT STEP: command a 5 deg lean (classic step-response metrics)
% =====================================================================

ref_deg = 5;
T_step = 10;   % long run: the slow integral pole takes seconds to finish the job
st = tvc_sim(struct('sigma_noise', 0, 'theta0', 0, 'ref', deg2rad(ref_deg)), T_step);
si_nl  = stepinfo(rad2deg(st.theta), st.t, ref_deg, 'SettlingTimeThreshold', 0.02);
[y_lin, t_lin] = step(T * ref_deg, T_step);
si_lin = stepinfo(T * ref_deg, 'SettlingTimeThreshold', 0.02);

f5 = figure('Visible', vis, 'Position', [50 50 800 450]);
plot(st.t, rad2deg(st.theta), 'b', 'LineWidth', 1.5); hold on; grid on;
plot(t_lin, y_lin, 'k--');
yline(ref_deg*[0.98 1.02], ':', 'HandleVisibility', 'off');
xline(si_nl.SettlingTime, '--', 'settled (2%)');
xlabel('time [s]'); ylabel('\theta [deg]'); xlim([0 min(T_step, max(1, 1.2*si_nl.SettlingTime))]);
legend('nonlinear sim', 'linear model', 'Location', 'southeast');
title(sprintf('Commanded %g deg lean:  rise %.0f ms,  overshoot %.0f%%,  peak at %.0f ms,  settling %.0f ms', ...
              ref_deg, 1000*si_nl.RiseTime, si_nl.Overshoot, 1000*si_nl.PeakTime, 1000*si_nl.SettlingTime));
exportgraphics(f5, fullfile(outdir, '5_setpoint_step.png'));

S{end+1} = '';
S{end+1} = sprintf('--- 5. Commanded %g deg lean (setpoint step) ---', ref_deg);
S{end+1} = sprintf('%-22s %12s %12s', '', 'nonlinear', 'linear');
S{end+1} = sprintf('%-22s %12.0f %12.0f', 'Rise time 10-90% [ms]', 1000*si_nl.RiseTime, 1000*si_lin.RiseTime);
S{end+1} = sprintf('%-22s %12.0f %12.0f', 'Peak time [ms]', 1000*si_nl.PeakTime, 1000*si_lin.PeakTime);
S{end+1} = sprintf('%-22s %12.1f %12.1f', 'Overshoot [%]', si_nl.Overshoot, si_lin.Overshoot);
S{end+1} = sprintf('%-22s %12.0f %12.0f', 'Settling 2% [ms]', 1000*si_nl.SettlingTime, 1000*si_lin.SettlingTime);
k_P = b*p.Kp / (b*p.Kp - a);   % where P alone would park it, as a multiple of the command
S{end+1} = sprintf(['Why the big overshoot: gravity helps the rocket lean further, so the P term alone would park it at\n' ...
                    'bKp/(bKp - a) = %.1fx the command (%.1f deg for a %g deg command). Only the slow integral pole\n' ...
                    '(tau = %.1f s) pulls it back to %g deg, so settling takes seconds. Fine for balancing at 0 deg; if the\n' ...
                    'joystick will command lean angles, add gravity feedforward (u_ff = -(a/b)*sin(ref)) or scale the command.'], ...
                   k_P, k_P*ref_deg, ref_deg, max(tc), ref_deg);

% =====================================================================
% 6. DISTURBANCE REJECTION: a steady push (e.g. someone leaning on it)
% =====================================================================

D = 0.2 * b * Umax;     % disturbance = 20% of the fan's full control authority
t_on = 0.5;
dist = @(t) D * (t >= t_on);
dPID = tvc_sim(struct('sigma_noise', 0, 'theta0', 0, 'dist_accel', dist), 4);
dPD  = tvc_sim(struct('sigma_noise', 0, 'theta0', 0, 'dist_accel', dist, 'Ki', 0), 4);
ss_PD_pred = rad2deg(D / (b*p.Kp - a));       % PD steady offset: b*Kp*th = a*th + D
peak_dev = max(abs(rad2deg(dPID.theta)));
i_back = find(abs(rad2deg(dPID.theta)) > 0.1*peak_dev, 1, 'last');

f6 = figure('Visible', vis, 'Position', [50 50 800 450]);
plot(dPID.t, rad2deg(dPID.theta), 'b', 'LineWidth', 1.5); hold on; grid on;
plot(dPD.t, rad2deg(dPD.theta), 'm', 'LineWidth', 1.2);
yline(ss_PD_pred, 'm:', sprintf('PD offset = %.2f deg', ss_PD_pred));
xline(t_on, ':', 'push starts');
xlabel('time [s]'); ylabel('\theta [deg]');
legend(sprintf('PID (Ki = %.2f)', p.Ki), 'PD only (Ki = 0)', 'Location', 'northeast');
title(sprintf('Steady push = 20%% of fan authority:  peak %.2f deg,  PID removes the offset (back within 10%% at %.1f s)', ...
              peak_dev, dPID.t(i_back)));
exportgraphics(f6, fullfile(outdir, '6_disturbance_rejection.png'));

S{end+1} = '';
S{end+1} = '--- 6. Steady push (20% of fan authority, starts at 0.5 s) ---';
S{end+1} = sprintf('Peak deviation: %.2f deg. PD alone would sit %.2f deg off forever; the integral pulls it back (within 10%% of peak by %.1f s).', ...
                   peak_dev, ss_PD_pred, dPID.t(i_back));
S{end+1} = sprintf('Final error with PID: %.3f deg;  with PD only: %.2f deg.', rad2deg(dPID.theta(end)), rad2deg(dPD.theta(end)));

% =====================================================================
% 7. HOW BEHAVIOR CHANGES WITH TIP SIZE (nonlinear: saturation, sin)
% =====================================================================

tips = [1 2 4 6 8 10 12 15 18 21 24 27 29.5];
sw = struct('os', [], 'ts', [], 'sat', [], 'peakfan', []);
for k = 1:numel(tips)
    r = tvc_sim(struct('sigma_noise', 0, 'theta0', deg2rad(tips(k))), 3);
    m = ic_metrics(r.t, r.theta, deg2rad(tips(k)));
    sw.os(k) = m.overshoot_pct;  sw.ts(k) = m.t_band2deg;  sw.ts2(k) = m.t_settle2;
    sw.sat(k) = 100*mean(abs(r.u) >= 0.999*Umax);
    sw.peakfan(k) = max(abs(r.u))*p.VIS_SCALE;
    sw.stop(k) = r.hit_stop;
end

f7 = figure('Visible', vis, 'Position', [50 50 900 600]);
tl = tiledlayout(f7, 2, 2, 'TileSpacing', 'compact');
nexttile(tl); plot(tips, sw.os, 'o-'); grid on; xlabel('initial tip [deg]'); ylabel('overshoot [%]'); title('Overshoot');
nexttile(tl); plot(tips, 1000*sw.ts2, 'o-'); grid on; xlabel('initial tip [deg]'); ylabel('[ms]'); title('Settling time (2% of tip)');
nexttile(tl); plot(tips, sw.peakfan, 'o-'); grid on; yline(p.fan_tilt_max_deg, 'r:', 'limit');
xlabel('initial tip [deg]'); ylabel('[deg]'); title('Peak fan tilt');
nexttile(tl); plot(tips, sw.sat, 'o-'); grid on; xlabel('initial tip [deg]'); ylabel('[% of time]'); title('Time at fan limit');
title(tl, 'Large tips saturate the fan, so the response stops being "linear"');
exportgraphics(f7, fullfile(outdir, '7_tip_size_sweep.png'));

k_sat = find(sw.peakfan >= 0.999*p.fan_tilt_max_deg, 1);
S{end+1} = '';
S{end+1} = '--- 7. Effect of tip size (nonlinear) ---';
if isempty(k_sat)
    S{end+1} = 'The fan never saturates over the tested range.';
else
    S{end+1} = sprintf('The fan hits its +/-%g deg limit for tips of %g deg and up; past that, overshoot and settling grow with tip size.', ...
                       p.fan_tilt_max_deg, tips(k_sat));
end
S{end+1} = sprintf('Overshoot ranges %.0f%%-%.0f%%; settling (2%%) %.0f-%.0f ms over %g-%g deg tips. Recovered from all: %s.', ...
                   min(sw.os), max(sw.os), 1000*min(sw.ts2), 1000*max(sw.ts2), tips(1), tips(end), string(~any(sw.stop)));

% =====================================================================
% 8. GAIN SENSITIVITY: root locus vs Kp, and overshoot/settling vs Kd
% =====================================================================

f8 = figure('Visible', vis, 'Position', [50 50 1000 450]);
tl = tiledlayout(f8, 1, 2, 'TileSpacing', 'compact');
nexttile(tl);
% Scale the whole PID by k: L(k) = k*L. k = 1 is the current design.
kk = logspace(-1, 1, 400);
R = rlocus(L, kk);
plot(real(R'), imag(R'), '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 4); hold on; grid on;
P1 = rlocus(L, 1);
plot(real(P1), imag(P1), 'bs', 'MarkerFaceColor', 'b');
xline(0, 'k');
xlim([-60 30]); ylim([-80 80]);
xlabel('real [1/s]'); ylabel('imag [rad/s]');
title('Root locus: all gains scaled by 0.1x-10x (blue = current)');
nexttile(tl);
Kd_list = linspace(0.008, 0.05, 15);
os = nan(size(Kd_list)); ts = os;
for k = 1:numel(Kd_list)
    r = tvc_sim(struct('sigma_noise', 0, 'Kd', Kd_list(k)), 3);
    m = ic_metrics(r.t, r.theta, th0);
    if ~r.hit_stop, os(k) = m.overshoot_pct; ts(k) = 1000*m.t_band2deg; end
end
yyaxis left;  plot(Kd_list, os, 'o-'); ylabel('overshoot [%]');
yyaxis right; plot(Kd_list, ts, 's-'); ylabel('settling +/-2 deg [ms]');
xline(p.Kd, ':', 'current Kd'); grid on; xlabel('K_d');
title('More derivative = more damping (until noise/jitter limits it)');
exportgraphics(f8, fullfile(outdir, '8_gain_sensitivity.png'));

% Range of overall gain scaling that stays stable
stable_k = arrayfun(@(k) all(real(rlocus(L, k)) < 0), kk);
S{end+1} = '';
S{end+1} = '--- 8. Gain sensitivity ---';
if any(stable_k)
    S{end+1} = sprintf('Scaling ALL gains together stays stable from %.2fx to %.2fx of the current values (tested 0.1x-10x).', ...
                       kk(find(stable_k, 1)), kk(find(stable_k, 1, 'last')));
end
[~, ib] = min(os);
S{end+1} = sprintf('Overshoot vs Kd: %.0f%% at Kd = %.3f down to %.0f%% at Kd = %.3f (current Kd = %.3f).', ...
                   os(find(~isnan(os), 1)), Kd_list(find(~isnan(os), 1)), os(ib), Kd_list(ib), p.Kd);

% =====================================================================
% SUMMARY
% =====================================================================

fprintf('%s\n', S{:});
fid = fopen(fullfile(outdir, 'summary.txt'), 'w');
fprintf(fid, '%s\n', S{:});
fclose(fid);
fprintf('\nFigures and summary.txt saved to %s\n', outdir);

% =====================================================================
% LOCAL FUNCTIONS
% =====================================================================

function m = ic_metrics(t, th, th0)
    % Metrics for recovering from an initial tip th0 back to 0.
    s0 = sign(th0); A = abs(th0);
    y = s0 * th;                         % positive = same side as the tip
    % Overshoot: how far it swings past upright to the OTHER side
    [mn, ip] = min(y);
    m.overshoot_deg = max(0, -rad2deg(mn));
    m.overshoot_pct = 100 * max(0, -mn) / A;
    m.t_peak = t(ip);
    % Recovery (fall) time: from 90% to 10% of the tip
    i90 = find(y <= 0.9*A, 1); i10 = find(y <= 0.1*A, 1);
    m.t_recover = t(i10) - t(i90);
    % Settling: stays within 2% of the tip, and within +/-2 deg
    m.t_settle2  = settle_time(t, th, 0.02*A);
    m.t_band2deg = settle_time(t, th, deg2rad(2));
    m.ss_deg = rad2deg(mean(th(t >= t(end) - 0.5)));
    % Decay time constant from the envelope: fit log|peaks| vs time
    [pk, locs] = findpeaks(abs(th), t, 'MinPeakHeight', 0.01*A);
    if numel(pk) >= 3
        c = polyfit(locs, log(pk), 1);
        m.tau_env = -1 / c(1);
    else
        m.tau_env = NaN;
    end
end

function ts = settle_time(t, th, band)
    out = find(abs(th) > band, 1, 'last');
    if isempty(out), ts = 0;
    elseif out == numel(th), ts = inf;
    else, ts = t(out + 1);
    end
end

function s = fmt_ms(x)
    if isnan(x) || x <= 0, s = 'n/a (not enough oscillation peaks)'; else, s = sprintf('%.0f ms', 1000*x); end
end

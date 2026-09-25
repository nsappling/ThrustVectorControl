%% tune_pid.m
% Headless PID test/tuning for the TVC demonstrator, built on tvc_sim.m.
% Safe to run with `matlab -batch` (no interactive windows; figures are
% saved as PNGs instead of shown).
%
%   matlab -batch "run('Alec Documents/tune_pid.m')"
%
% What it does:
%   1. Simulates the CURRENT gains from rocketgimball.m and prints
%      performance metrics (settling time, overshoot, noise jitter...).
%   2. Linear stability check with the Control System Toolbox: closed-
%      loop poles and phase margin of the linearized plant + filter + PID.
%   3. Sweeps Kp x Kd (Ki held fixed), scores every combination over
%      several noise seeds, and reports the best gains.
%   4. Saves plots to Alec Documents/results/.
%
% REMEMBER: F_t, R, m, r_0 are still guesses, so the "best" gains are
% only as good as those numbers. Re-run once they're measured.

clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(here);   % so tvc_sim.m is found no matter where this is run from
outdir = fullfile(here, 'results');
if ~exist(outdir, 'dir'), mkdir(outdir); end

T = 3;           % [s] length of each simulated run
seeds = 1:3;     % noise seeds averaged over during the sweep

% =====================================================================
% 1. BASELINE: current gains from rocketgimball.m
% =====================================================================

base = tvc_sim(struct(), T);
mb = tvc_metrics(base);
fprintf('=== Baseline gains: Kp=%.3f  Ki=%.3f  Kd=%.4f ===\n', base.p.Kp, base.p.Ki, base.p.Kd);
fprintf('  a = %.1f 1/s^2,  b = %.1f,  Umax = %.3f (= +/-%g deg fan tilt)\n', ...
        base.a, base.b, base.Umax, base.p.fan_tilt_max_deg);
fprintf('  Max recoverable angle = %.1f deg (stops at %g deg)\n', ...
        asind(min(1, base.b*base.Umax/base.a)), base.p.theta_limit_deg);
print_metrics(mb);

% =====================================================================
% 2. LINEAR STABILITY CHECK (small-angle: sin(theta) ~ theta)
% Plant:  theta'' = a*theta + b*u   ->  G(s) = b / (s^2 - a)
% Sensor: first-order low-pass      ->  F(s) = 1 / (tau*s + 1)
% PID acts on the filtered signal, so the loop is L = C*F*G.
% =====================================================================

linear_check(base.p, base.a, base.b);

% =====================================================================
% 3. GAIN SWEEP over Kp x Kd
% =====================================================================

Kp_list = linspace(0.30, 1.20, 19);
Kd_list = linspace(0.005, 0.060, 12);
cost = inf(numel(Kd_list), numel(Kp_list));
settle = inf(size(cost));

fprintf('\nSweeping %d gain combinations x %d seeds...\n', numel(cost), numel(seeds));
for i = 1:numel(Kd_list)
    for j = 1:numel(Kp_list)
        c = zeros(size(seeds)); ts = zeros(size(seeds));
        for s = 1:numel(seeds)
            r = tvc_sim(struct('Kp', Kp_list(j), 'Kd', Kd_list(i), 'seed', seeds(s)), T);
            m = tvc_metrics(r);
            ts(s) = m.settle_time;
            % Cost: fast settling, plus a penalty on servo jitter (a real
            % servo chattering at +/-several degrees wears out and looks
            % bad), plus overshoot. Failing runs get Inf.
            if r.hit_stop || isinf(m.settle_time)
                c(s) = inf;
            else
                c(s) = m.settle_time + 0.02*m.fan_jitter_deg + 0.01*m.overshoot_deg;
            end
        end
        cost(i,j) = mean(c);
        settle(i,j) = mean(ts);
    end
end

[best_cost, idx] = min(cost(:));
if isinf(best_cost)
    fprintf('No gain combination in the sweep stabilized the rocket.\n');
    [bi, bj] = deal(1);
else
    [bi, bj] = ind2sub(size(cost), idx);
    best = tvc_sim(struct('Kp', Kp_list(bj), 'Kd', Kd_list(bi)), T);
    mbest = tvc_metrics(best);
    fprintf('\n=== Best in sweep: Kp=%.3f  Ki=%.3f  Kd=%.4f ===\n', ...
            best.p.Kp, best.p.Ki, best.p.Kd);
    print_metrics(mbest);
    linear_check(best.p, best.a, best.b);
end

% =====================================================================
% 4. PLOTS (saved, not displayed -- works in -batch mode)
% =====================================================================

f1 = figure('Visible', 'off', 'Position', [100 100 900 600]);
subplot(2,1,1); hold on; grid on;
plot(base.t, rad2deg(base.theta), 'DisplayName', 'baseline');
if exist('best', 'var')
    plot(best.t, rad2deg(best.theta), 'DisplayName', 'best in sweep');
end
yline([-2 2], ':k', 'HandleVisibility', 'off');
ylabel('\theta [deg]'); title('Body angle'); legend('Location', 'best');
subplot(2,1,2); hold on; grid on;
plot(base.t, base.u * base.p.VIS_SCALE, 'DisplayName', 'baseline');
if exist('best', 'var')
    plot(best.t, best.u * best.p.VIS_SCALE, 'DisplayName', 'best in sweep');
end
ylabel('fan tilt command [deg]'); xlabel('time [s]'); title('Actuator');
exportgraphics(f1, fullfile(outdir, 'pid_response.png'));

f2 = figure('Visible', 'off', 'Position', [100 100 800 500]);
imagesc(Kp_list, Kd_list, settle); set(gca, 'YDir', 'normal'); colorbar;
hold on;
plot(base.p.Kp, base.p.Kd, 'wo', 'MarkerSize', 10, 'LineWidth', 2);
if exist('best', 'var')
    plot(Kp_list(bj), Kd_list(bi), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
end
xlabel('K_p'); ylabel('K_d');
title('Mean settling time [s] (white o = baseline, red star = best; blank = unstable)');
set(gca, 'Color', [0.9 0.9 0.9]);
set(findobj(gca, 'Type', 'Image'), 'AlphaData', isfinite(settle));
exportgraphics(f2, fullfile(outdir, 'gain_sweep.png'));

fprintf('\nPlots saved to %s\n', outdir);

% =====================================================================
% LOCAL FUNCTIONS
% =====================================================================

function m = tvc_metrics(r)
    % Performance numbers for one tvc_sim run.
    band = deg2rad(2);                        % "settled" = within +/-2 deg
    outside = find(abs(r.theta) > band, 1, 'last');
    if isempty(outside)
        m.settle_time = 0;
    elseif outside == numel(r.theta)
        m.settle_time = inf;                  % never settled
    else
        m.settle_time = r.t(outside + 1);
    end
    % Overshoot = how far it swings past upright to the OTHER side.
    m.overshoot_deg = max(0, max(-sign(r.p.theta0) * rad2deg(r.theta)));
    last = r.t >= r.t(end) - 1;               % final 1 s = "steady state"
    m.ss_rms_deg = rad2deg(rms(r.theta(last)));
    m.fan_jitter_deg = std(r.u(last)) * r.p.VIS_SCALE;
    m.sat_frac = mean(abs(r.u) >= 0.999 * r.Umax);
    m.hit_stop = r.hit_stop;
end

function print_metrics(m)
    fprintf('  Settling time (+/-2 deg): %s\n', fmt_time(m.settle_time));
    fprintf('  Overshoot:                %.2f deg\n', m.overshoot_deg);
    fprintf('  Steady-state RMS angle:   %.3f deg\n', m.ss_rms_deg);
    fprintf('  Steady-state fan jitter:  %.2f deg (std dev of fan tilt command)\n', m.fan_jitter_deg);
    fprintf('  Time at actuator limit:   %.1f %%\n', 100*m.sat_frac);
    fprintf('  Hit mechanical stops:     %s\n', string(m.hit_stop));
end

function s = fmt_time(t)
    if isinf(t), s = 'never'; else, s = sprintf('%.3f s', t); end
end

function linear_check(p, a, b)
    s = tf('s');
    G = b / (s^2 - a);
    F = 1 / (p.tau_filter*s + 1);
    C = pid(p.Kp, p.Ki, p.Kd);
    L = C * F * G;
    CL = feedback(L, 1);
    poles = pole(CL);
    fprintf('  Linear model: closed loop is %s\n', ...
            string(ifelse(all(real(poles) < 0), 'STABLE', 'UNSTABLE')));
    [wn, zeta] = damp(CL);
    [~, k] = min(zeta);
    fprintf('    least-damped mode: zeta = %.2f at %.1f rad/s\n', zeta(k), wn(k));
    mg = allmargin(L);
    if ~isempty(mg.PhaseMargin)
        fprintf('    phase margin: %.1f deg at %.1f rad/s\n', mg.PhaseMargin(1), mg.PMFrequency(1));
    end
end

function out = ifelse(cond, a, b)
    if cond, out = a; else, out = b; end
end

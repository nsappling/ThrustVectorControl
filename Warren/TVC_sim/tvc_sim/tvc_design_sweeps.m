%% TVC_DESIGN_SWEEPS  Trade studies for the design phase.
%
%   Each %% section is independent: put the cursor in one and press
%   Ctrl+Enter (Run Section). Every section starts from tvc_params(), so
%   edit that file first and the sweeps follow your current design.
%
%   1. Control authority vs geometry (L, h_cg)         -> mechanical layout
%   2. Minimum throttle vs lever arm                   -> noise limit / EDF choice
%   3. Kp-Kd map from nonlinear sims                    -> gain tuning starting point
%   4. Servo speed x PWM refresh rate                   -> servo selection
%   5. Control loop rate                                -> MCU / IMU interface
%   6. IMU location x filter time constant              -> IMU mounting + filtering
%   7. Fixed gains across throttle                      -> is gain scheduling needed?
%   8. Exhibit difficulty: human delay x CG height      -> how hard it is for visitors
%
%   Sweeps use a coarser plant step (p.dt = 5e-4) to run faster. Spot-check
%   any point you care about at the default step with tvc_main.

%% 1. Control authority vs geometry ------------------------------------
p = tvc_params();
Lv = linspace(0.10, 0.50, 81);         % pivot -> gimbal [m]
hv = linspace(0.02, 0.25, 81);         % pivot -> CG [m]
[LL, HH] = meshgrid(Lv, hv);
T = edf_thrust(p, p.u_fixed);
R = T*LL*sin(p.delta_cmd_max) ./ (p.m*p.g*HH*sin(p.theta_stop));

figure('Name', '1 Authority');
contourf(LL*100, HH*100, R, [0 0.5 1 1.5 2 3 4 6], 'ShowText', 'on'); hold on;
contour(LL*100, HH*100, R, [1.5 1.5], 'r', 'LineWidth', 2);
plot(p.L*100, p.h_cg*100, 'wp', 'MarkerSize', 14, 'MarkerFaceColor', 'k');
xlabel('lever arm L, pivot \rightarrow gimbal [cm]'); ylabel('CG height h_{cg} [cm]');
title(sprintf('Authority ratio at the %g° stop (T = %.1f N, \\delta_{max} = %g°). Red: 1.5', ...
    rad2deg(p.theta_stop), T, rad2deg(p.delta_cmd_max)));
colorbar;
% Note: the mass also changes when you move the EDF; re-check with real CAD numbers.

%% 2. Minimum throttle vs lever arm -------------------------------------
p = tvc_params();
Lv = linspace(0.15, 0.50, 50);
ratios = [1 1.5 2];
umin = nan(numel(ratios), numel(Lv));
uu = linspace(0, 1, 1001);
TT = edf_thrust(p, uu);
for j = 1:numel(Lv)
    for i = 1:numel(ratios)
        Treq = ratios(i)*p.m*p.g*p.h_cg*sin(p.theta_stop)/(Lv(j)*sin(p.delta_cmd_max));
        k = find(TT >= Treq, 1);
        if ~isempty(k), umin(i,j) = uu(k); end
    end
end
figure('Name', '2 Min throttle');
plot(Lv*100, umin', 'LineWidth', 1.5); grid on; hold on;
xline(p.L*100, 'k--', 'current L');
legend(compose('authority %.1f', ratios), 'Location', 'northeast');
xlabel('lever arm L [cm]'); ylabel('minimum throttle [-]');
title('Lower throttle = quieter (70-75 dB limit). Longer L buys that back.');

%% 3. Kp-Kd stability / performance map (nonlinear) ----------------------
p = tvc_params(); p.dt = 5e-4;
r = tvc_linear_report(p, true);
Kpv = linspace(0.5*r.Kp_min, 3*p.Kp, 18);
Kdv = linspace(0, 2.5*p.Kd, 18);
ST = nan(numel(Kdv), numel(Kpv));  OS = ST;
sc = tvc_scenario('pd', 'T_end', 3);
for i = 1:numel(Kdv)
    for j = 1:numel(Kpv)
        q = p;  q.Kp = Kpv(j);  q.Kd = Kdv(i);
        m = tvc_metrics(tvc_simulate(q, sc));
        ST(i,j) = m.settle_s;  OS(i,j) = m.overshoot_pct;
    end
end
figure('Name', '3 Gain map', 'Position', [100 100 1100 420]);
subplot(1,2,1); nan_image(Kpv, Kdv, ST); hold on;
xline(r.Kp_min, 'r--', 'K_p min (linear)', 'LineWidth', 1.5);
plot(p.Kp, p.Kd, 'wp', 'MarkerSize', 14, 'MarkerFaceColor', 'k');
xlabel('K_p [rad/rad]'); ylabel('K_d [s]'); title('settling time to 2° [s]  (white = never / fell)');
subplot(1,2,2); nan_image(Kpv, Kdv, min(OS, 100)); hold on;
plot(p.Kp, p.Kd, 'wp', 'MarkerSize', 14, 'MarkerFaceColor', 'k');
xlabel('K_p [rad/rad]'); ylabel('K_d [s]'); title('overshoot [%]');

%% 4. Servo speed x PWM refresh -----------------------------------------
p = tvc_params(); p.dt = 5e-4;
speeds   = [0.04 0.06 0.08 0.10 0.13 0.16 0.20 0.25];   % s/60deg (datasheet)
refresh  = [50 100 200 333];                            % Hz
ST = nan(numel(refresh), numel(speeds));  MD = ST;
sc = tvc_scenario('pd', 'T_end', 3);
for i = 1:numel(refresh)
    for j = 1:numel(speeds)
        q = p;  q.servo_refresh = refresh(i);  q.servo_speed = speeds(j);
        q.servo_rate = q.servo_derate*deg2rad(60)/q.servo_speed;
        m = tvc_metrics(tvc_simulate(q, sc));
        ST(i,j) = m.settle_s;  MD(i,j) = m.max_dev_deg + 100*m.fell;
    end
end
figure('Name', '4 Servo selection', 'Position', [100 100 1000 400]);
subplot(1,2,1); plot(speeds, ST', 'o-', 'LineWidth', 1.4); grid on;
xlabel('servo speed [s/60°]'); ylabel('settling time [s]');
legend(compose('%d Hz PWM', refresh), 'Location', 'northwest');
subplot(1,2,2); plot(speeds, min(MD, 25)', 'o-', 'LineWidth', 1.4); grid on;
xlabel('servo speed [s/60°]'); ylabel('max deviation [deg] (25 = fell)');
sgtitle('Servo choice: faster slew and faster frame rate both matter');

%% 5. Control loop rate --------------------------------------------------
p = tvc_params(); p.dt = 2.5e-4;
rates = [50 100 200 400 800 1000];
res = zeros(numel(rates), 3);
sc = tvc_scenario('pd', 'T_end', 3);
for i = 1:numel(rates)
    q = p;  q.loop_rate = rates(i);
    m = tvc_metrics(tvc_simulate(q, sc));
    res(i,:) = [m.settle_s, m.overshoot_pct, m.max_dev_deg + 100*m.fell];
end
figure('Name', '5 Loop rate');
yyaxis left;  semilogx(rates, res(:,1), 'o-', 'LineWidth', 1.4); ylabel('settling time [s]');
yyaxis right; semilogx(rates, res(:,2), 's-', 'LineWidth', 1.4); ylabel('overshoot [%]');
grid on; xlabel('control loop rate [Hz]');
title(sprintf('Loop rate (servo frame fixed at %d Hz)', p.servo_refresh));

%% 6. IMU location x complementary-filter time constant -----------------
% The accelerometer measures r*alpha and r*omega^2 on top of gravity, so an
% IMU far from the pivot reports a wrong angle exactly when the vehicle is
% being pushed. A fast filter (small tau) trusts that wrong angle more.
p = tvc_params(); p.dt = 5e-4;
rv   = [0 0.05 0.10 0.15 0.20 0.30];
tauv = [0.1 0.2 0.35 0.5 1 2 4];
sc = tvc_scenario('pd', 'T_end', 4);
OS = nan(numel(rv), numel(tauv), 2);  JT = OS;
for c = 1:2
    for i = 1:numel(rv)
        for j = 1:numel(tauv)
            q = p;  q.r_imu = rv(i);  q.cf_tau = tauv(j);  q.accel_comp = (c == 2);
            m = tvc_metrics(tvc_simulate(q, sc));
            OS(i,j,c) = m.overshoot_pct + 1000*m.fell;
            JT(i,j,c) = m.jitter_deg;
        end
    end
end
figure('Name', '6 IMU + filter', 'Position', [100 100 1100 750]);
lbl = {'no accel compensation', 'with r\alpha, r\omega^2 compensation'};
for c = 1:2
    subplot(2,2,c);   semilogx(tauv, min(OS(:,:,c), 150)', 'o-', 'LineWidth', 1.3); grid on;
    xlabel('filter \tau [s]'); ylabel('overshoot [%] (150 = bad)'); title(lbl{c});
    legend(compose('r_{imu} = %.2f m', rv), 'Location', 'northeast');
    subplot(2,2,c+2); semilogx(tauv, JT(:,:,c)', 'o-', 'LineWidth', 1.3); grid on;
    xlabel('filter \tau [s]'); ylabel('gimbal jitter [deg rms]');
end
sgtitle(sprintf('EDF vibration %.1f m/s^2, gyro bias %.2f °/s', p.accel_vib, rad2deg(p.gyro_bias)));
% Long tau -> less noise but gyro bias drifts the estimate (steady-state
% error ~ bias * tau). Check m.sse_deg if you push tau up.

%% 7. Fixed gains across throttle ---------------------------------------
p = tvc_params(); p.dt = 5e-4;               % gains designed at p.u_fixed
uv = 0.40:0.05:1.0;
res = nan(numel(uv), 4);
for i = 1:numel(uv)
    m = tvc_metrics(tvc_simulate(p, tvc_scenario('pd', 'T_end', 3, 'throttle', uv(i))));
    res(i,:) = [m.settle_s, m.overshoot_pct, m.max_dev_deg, m.fell];
end
figure('Name', '7 Throttle robustness');
subplot(2,1,1); plot(uv, res(:,1), 'o-', 'LineWidth', 1.4); grid on; ylabel('settling [s]');
xline(p.u_fixed, 'k--', 'design point');
title('Same gains, different throttle: loop gain scales with T, so response changes');
subplot(2,1,2); plot(uv, res(:,2), 'o-', 'LineWidth', 1.4); grid on; ylabel('overshoot [%]');
xlabel('throttle [-]'); xline(p.u_fixed, 'k--');
fell = uv(res(:,4) == 1);
if ~isempty(fell), fprintf('Section 7: falls at throttle %s\n', mat2str(fell)); end
% If visitors control throttle, schedule the gains: Kp, Kd ~ 1/T(u)
% (call tvc_design_gains(p, [], [], u) at each throttle).

%% 8. Exhibit difficulty: can a person do it? ---------------------------
% The project wants it "intentionally difficult" for a visitor but easy for
% the controller. The open-loop pole sqrt(m g h / I) sets how hard it is.
p = tvc_params(); p.dt = 5e-4;
hv  = [0.03 0.05 0.08 0.10 0.12 0.15];
dv  = [0.10 0.15 0.20 0.25 0.30 0.40];
nSeeds = 4;
S = zeros(numel(dv), numel(hv));
for j = 1:numel(hv)
    q = p;  q.h_cg = hv(j);
    q = tvc_design_gains(q);
    for i = 1:numel(dv)
        q.human_delay = dv(i);
        ok = 0;
        for s = 1:nSeeds
            m = tvc_metrics(tvc_simulate(q, tvc_scenario('human', 'T_end', 6, 'seed', s)));
            ok = ok + ~m.fell;
        end
        S(i,j) = ok/nSeeds;
    end
end
pole = sqrt(p.m*p.g*hv/p.I);
figure('Name', '8 Exhibit difficulty');
imagesc(hv*100, dv, S); axis xy; colorbar; caxis([0 1]); hold on;
plot(hv*100, sqrt(2)./pole, 'w--', 'LineWidth', 2);    % theoretical limit
xlabel('CG height above pivot [cm]'); ylabel('human reaction delay [s]');
title('Fraction of runs a person keeps it up (dashed = best-possible limit)');
% Kids react in ~0.25-0.4 s. Pick h_cg so they mostly fail and the PD never does.

%% -----------------------------------------------------------------------
function nan_image(x, y, Z)
imagesc(x, y, Z, 'AlphaData', ~isnan(Z)); axis xy; colorbar;
set(gca, 'Color', [1 1 1]);
end

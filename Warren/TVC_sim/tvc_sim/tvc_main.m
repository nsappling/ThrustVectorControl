%% TVC_MAIN  Run the final-demo tests in simulation.
%
%   Test 1 - controller OFF : same push, gimbal held at 0  -> falls to the stop
%   Test 2 - controller ON  : PD on the gimbal               -> recovers
%   Test 3 - human pilot    : joystick with reaction delay   -> why TVC needs a computer
%
%   Plots vehicle angle, angular velocity, and gimbal angle (the three
%   signals the project plan says to log) and prints the performance metrics.
%   Edit tvc_params.m first; nothing here needs changing for a new design.

clear; close all; clc;

p = tvc_params();
p = tvc_design_gains(p);          % re-run if you edit p below
tvc_linear_report(p);

tests = {'off', 'pd', 'human'};
names = {'Controller OFF', 'Controller ON (PD)', 'Human pilot'};
logs  = cell(size(tests));
for i = 1:numel(tests)
    logs{i} = tvc_simulate(p, tvc_scenario(tests{i}));
end

%% Metrics table
fprintf('%-20s %6s %10s %10s %10s %10s %10s\n', 'Test', 'fell', 'max dev', 'overshoot', 'settle', 'gimbal pk', 'jitter');
fprintf('%-20s %6s %10s %10s %10s %10s %10s\n', '', '', '[deg]', '[%]', '[s]', '[deg]', '[deg]');
for i = 1:numel(tests)
    m = tvc_metrics(logs{i});
    fprintf('%-20s %6d %10.2f %10.1f %10.3f %10.2f %10.2f\n', names{i}, m.fell, ...
        m.max_dev_deg, m.overshoot_pct, m.settle_s, m.delta_peak_deg, m.jitter_deg);
end

%% Plots
cols = lines(numel(tests));
h = gobjects(1, numel(tests));
figure('Name', 'TVC demo tests', 'Position', [100 100 900 750]);
ax(1) = subplot(3,1,1); hold on; grid on;
ax(2) = subplot(3,1,2); hold on; grid on;
ax(3) = subplot(3,1,3); hold on; grid on;
for i = 1:numel(tests)
    L = logs{i};
    h(i) = plot(ax(1), L.t, rad2deg(L.theta), 'Color', cols(i,:), 'LineWidth', 1.4);
    plot(ax(2), L.t, rad2deg(L.omega), 'Color', cols(i,:), 'LineWidth', 1.2);
    plot(ax(3), L.t, rad2deg(L.delta), 'Color', cols(i,:), 'LineWidth', 1.2);
end
yline(ax(1),  rad2deg(p.theta_stop), 'k--', 'hard stop');
yline(ax(1), -rad2deg(p.theta_stop), 'k--');
yline(ax(3),  rad2deg(p.delta_max), 'k:');
yline(ax(3), -rad2deg(p.delta_max), 'k:');
sc = logs{1}.sc;
for a = ax                         % shade the push
    yl = ylim(a);
    patch(a, sc.push_t0 + [0 1 1 0]*sc.push_dur, yl([1 1 2 2]), [1 0.8 0.3], ...
          'FaceAlpha', 0.3, 'EdgeColor', 'none');
    ylim(a, yl);
end
ylabel(ax(1), '\theta vehicle [deg]');
ylabel(ax(2), '\theta-dot [deg/s]');
ylabel(ax(3), '\delta gimbal [deg]');
xlabel(ax(3), 'time [s]');
legend(h, names, 'Location', 'best');
title(ax(1), sprintf('Push %.2f N m for %.0f ms (shaded), throttle %.2f', ...
    sc.push_tau, 1e3*sc.push_dur, p.u_fixed));
linkaxes(ax, 'x');

%% Estimator view: what the controller thinks vs the truth (controller ON)
L = logs{2};
figure('Name', 'Attitude estimate', 'Position', [1020 100 700 450]);
plot(L.t, rad2deg(L.theta_acc), 'Color', [0.8 0.8 0.8]); hold on; grid on;
plot(L.t, rad2deg(L.theta), 'k', 'LineWidth', 1.5);
plot(L.t, rad2deg(L.theta_hat), 'r', 'LineWidth', 1.2);
legend('accelerometer-only angle', 'true \theta', 'complementary filter');
xlabel('time [s]'); ylabel('angle [deg]');
title(sprintf('IMU %.2f m from pivot, filter \\tau = %.2f s, vibration %.1f m/s^2', ...
    p.r_imu, p.cf_tau, p.accel_vib));

%% Animation (uncomment to watch Test 2)
% tvc_animate(logs{2});

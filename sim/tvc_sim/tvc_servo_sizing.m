%% TVC_SERVO_SIZING  Size the gimbal servo from inertia and a motion spec.
%
%   This follows the procedure in our notes:
%     1. inertia of everything that swings with the gimbal (parallel axis)
%     2. explicit motion spec
%     3. required angular acceleration (bang-bang move)
%     4. torque = I*alpha + friction + gravity offset + thrust misalignment + wire drag
%     5. x2 safety margin
%     6. compare against each candidate's torque-speed line (stall -> no-load)
%
%   Replace the [EST] numbers with CAD mass properties once the gimbal is
%   modeled. Copy the resulting Ig into tvc_params.m.

clear; clc;

%% 1. Gimbal inertia about the gimbal axis --------------------------------
% EDF modeled as a solid cylinder, axis along thrust, gimbal axis transverse
edf.m  = 0.095;      % [kg] FMS 50mm 11-blade unit incl. motor     [SPEC - weigh it]
edf.r  = 0.028;      % [m]  outer radius of the duct                [SPEC]
edf.l  = 0.065;      % [m]  duct length                             [SPEC]
edf.d  = 0.005;      % [m]  EDF CG offset from the gimbal axis      [DES - keep near 0]
cr.m   = 0.030;      % [kg] cradle / ring                            [EST]
cr.I   = 2.0e-5;     % [kg m^2] cradle about its own axis (CAD)      [EST]
cr.d   = 0.004;      % [m]  cradle CG offset                         [EST]
wire.m = 0.010;      % [kg] ESC wire lump that swings with the EDF   [EST]
wire.d = 0.030;      % [m]                                           [EST]

I_edf = edf.m*(3*edf.r^2 + edf.l^2)/12 + edf.m*edf.d^2;
I_cr  = cr.I + cr.m*cr.d^2;
I_w   = wire.m*wire.d^2;
Ig    = I_edf + I_cr + I_w;
m_g   = edf.m + cr.m + wire.m;
d_cg  = (edf.m*edf.d + cr.m*cr.d + wire.m*wire.d)/m_g;   % combined CG offset

%% 2. Motion spec -------------------------------------------------------
spec.dtheta = deg2rad(15);    % [rad] move size                       [DES]
spec.t      = 0.06;           % [s]   in this time                    [DES]
% 15 deg in 60 ms ~ 0.24 s/60deg loaded. Check tvc_design_sweeps section 4
% to see how fast the servo actually has to be for your geometry.

%% 3. Kinematics (bang-bang: accelerate half, decelerate half) -------------
alpha_req = 4*spec.dtheta/spec.t^2;     % [rad/s^2]
w_peak    = 2*spec.dtheta/spec.t;       % [rad/s]

%% 4. Load torque at the gimbal -------------------------------------------
g = 9.81;
T_edf      = 6.5;           % [N]   full thrust                              [EST]
misalign   = 0.001;         % [m]   thrust line offset from gimbal axis      [EST]
tau_fric   = 0.005;         % [N m] gimbal bearing friction                  [EST]
tau_wire   = 0.008;         % [N m] wire bending stiffness at full deflection [EST - measure]
tau_inert  = Ig*alpha_req;
tau_grav   = m_g*g*d_cg;    % worst case: CG offset horizontal (vehicle at the stop adds little)
tau_thrust = T_edf*misalign;
tau_req    = tau_inert + tau_grav + tau_thrust + tau_fric + tau_wire;

%% 5. Margin -----------------------------------------------------------
margin   = 2.0;
tau_need = margin*tau_req;

%% 6. Candidates -------------------------------------------------------
% Gimbal-side numbers depend on linkage ratio n = servo angle / gimbal angle:
%   tau_gimbal = n * tau_servo,  w_gimbal = w_servo / n
n_link = 1.0;                                                     % [DES]
% Fill these from datasheets AT THE VOLTAGE YOU WILL RUN. Values below are
% typical ranges only - they are not real part numbers.
cand = struct( ...
  'name',  {'micro analog (9g class)', 'micro digital MG', 'mini digital HV', 'standard digital HV'}, ...
  'stall', {0.18, 0.25, 0.45, 1.20}, ...    % [N m] stall torque
  'speed', {0.10, 0.08, 0.06, 0.09}, ...    % [s/60deg] no-load
  'hz',    {50,   333,  333,  333});        % max PWM frame rate

kgcm = @(x) x/0.0980665;
fprintf('\n=== Gimbal servo sizing ===================================\n');
fprintf('Gimbal inertia Ig       : %.3g kg m^2 (EDF %.3g, cradle %.3g, wire %.3g)\n', Ig, I_edf, I_cr, I_w);
fprintf('Swinging mass / CG off  : %.0f g / %.1f mm\n', 1e3*m_g, 1e3*d_cg);
fprintf('Spec                    : %.0f deg in %.0f ms\n', rad2deg(spec.dtheta), 1e3*spec.t);
fprintf('Required alpha / w_peak : %.0f rad/s^2 / %.1f rad/s (%.3f s/60deg)\n', alpha_req, w_peak, deg2rad(60)/w_peak);
fprintf('Torque: inertia %.4f + gravity %.4f + thrust %.4f + friction %.4f + wire %.4f\n', ...
    tau_inert, tau_grav, tau_thrust, tau_fric, tau_wire);
fprintf('        = %.4f N m  -> x%.1f margin = %.4f N m (%.2f kg cm) at the gimbal\n', ...
    tau_req, margin, tau_need, kgcm(tau_need));
fprintf('Ig for tvc_params.m     : p.Ig = %.3g;\n\n', Ig);

fprintf('%-26s %10s %12s %10s  %s\n', 'candidate', 'stall', 'torque@w_pk', 'PWM', 'verdict');
w_s = linspace(0, 1, 100);
figure('Name', 'Servo torque-speed'); hold on; grid on;
for k = 1:numel(cand)
    c = cand(k);
    w0 = deg2rad(60)/c.speed/n_link;              % gimbal-side no-load speed
    t0 = c.stall*n_link;                          % gimbal-side stall torque
    tq = t0*(1 - w_peak/w0);                      % torque available at w_peak
    ok = tq >= tau_need;
    v = 'OK'; if ~ok, v = 'too weak/slow'; end
    if c.hz < 100, v = [v ', 50 Hz frame adds ~10 ms']; end
    fprintf('%-26s %8.2f Nm %10.3f Nm %7d Hz  %s\n', c.name, c.stall, tq, c.hz, v);
    plot(w_s*w0, t0*(1 - w_s), 'LineWidth', 1.5, 'DisplayName', c.name);
end
plot(w_peak, tau_need, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'requirement (with margin)');
xlabel('gimbal speed [rad/s]'); ylabel('gimbal torque [N m]');
title(sprintf('Linear torque-speed lines, linkage ratio %.1f', n_link));
legend('Location', 'northeast');
fprintf('===========================================================\n');

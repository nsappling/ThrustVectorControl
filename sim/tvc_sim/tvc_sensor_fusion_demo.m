%% TVC_SENSOR_FUSION_DEMO  How our sensors get fused into one angle estimate.
%
%   Standalone teaching script: run it, then read it top to bottom.
%   It walks through the same pipeline the Teensy will run every loop:
%
%     1. READ        raw integer counts from each sensor (simulated here)
%     2. CONVERT     counts -> physical units (deg/s, g, uT, deg)
%     3. CALIBRATE   remove gyro bias / magnetometer reference at startup
%     4. MEASURE     turn each sensor into something the filter can use
%                    (gyro -> rate, accel -> tilt angle, mag -> tilt angle)
%     5. KALMAN      PREDICT with the physics model, then UPDATE with
%                    whichever sensors have new data this tick
%     6. CONTROL     the PD controller uses the filter's estimate
%
%   Sensors simulated (and why each one is or isn't trusted):
%     Gyroscope      (in the IMU)  - fast and smooth, but has a slowly drifting bias
%     Accelerometer  (in the IMU)  - knows "which way is down", but is fooled
%                                    by vibration and by the vehicle's own motion
%     Magnetometer   (9-axis IMU)  - absolute reference too, but the EDF motor
%                                    and wires disturb it badly
%     Pivot encoder  (AS5600)      - angle measured directly at the pivot.
%                                    Used here as GROUND TRUTH to grade the filter
%                                    (set USE_ENCODER = true to fuse it too)
%
%   Kalman filter state:  x = [theta; omega; gyro_bias]
%       theta      vehicle tilt from vertical          [rad]
%       omega      tilt rate                           [rad/s]
%       gyro_bias  what the gyro reads when not moving [rad/s]
%
%   Timeline of the run:
%     0 - 1 s     vehicle held upright on its rest, EDF already running
%                 -> startup calibration (fan on, so its vibration and
%                    magnetic field are part of the calibration)
%     1 s         released, controller takes over
%     2 s, 7 s    someone pushes it (disturbance torques)
%     9 - 10.5 s  a visitor's phone / metal object near the magnetometer
%     all along   gyro bias drifts as the electronics warm up
%
%   No toolboxes needed.

clear; clc; close all;
rng(2);                                   % repeatable noise

%% ======================================================================
%  0. SETTINGS
%  ======================================================================
USE_ENCODER = false;    % true = also fuse the pivot encoder into the filter

dt        = 1/400;      % [s] control loop / IMU sample period (400 Hz)
T_end     = 12;         % [s] run length
t_release = 1.0;        % [s] end of the "held still" calibration window
push_t0   = [2.0 7.0];  % [s] disturbance start times
push_dur  = 0.10;       % [s] disturbance length
push_tau  = [0.30 -0.30]; % [N m] disturbance torques
magdist_t = [9.0 10.5]; % [s] window with a stray magnetic object nearby

N = round(T_end/dt);
t = (0:N-1)*dt;

% ---- vehicle physics (same numbers as tvc_params.m) --------------------
veh.m     = 0.55;           % [kg]     rotating mass
veh.h     = 0.12;           % [m]      pivot -> CG
veh.I     = 0.040;          % [kg m^2] inertia about pivot
veh.L     = 0.30;           % [m]      pivot -> gimbal (thrust lever arm)
veh.T     = 4.1;            % [N]      EDF thrust at our fixed throttle
veh.b     = 0.002;          % [N m s]  bearing friction
veh.g     = 9.81;
veh.r_imu = 0.20;           % [m]      pivot -> IMU (accel feels r*alpha, r*omega^2)
veh.stop  = deg2rad(20);    % [rad]    hard stops

a_lin = veh.m*veh.g*veh.h/veh.I;   % linear model: theta_dd = a*theta + b*delta
b_lin = veh.T*veh.L/veh.I;

% ---- controller + servo -----------------------------------------------
Kp        = 5.2;            % [rad/rad]
Kd        = 0.62;           % [s]
delta_max = deg2rad(14);    % [rad] gimbal command limit
tau_servo = 0.03;           % [s]   servo modeled as a first-order lag

%% ======================================================================
%  SENSOR DEFINITIONS - one block per instrument
%  Numbers are typical datasheet values; replace with your bench measurements.
%  ======================================================================

% ---- GYROSCOPE (inside the IMU, e.g. MPU-6050 at +/-500 deg/s range) ---
%  Measures angular RATE about the pivot axis.
%  Output: signed 16-bit integer counts.  65.5 counts = 1 deg/s.
gyro.lsb_per_dps = 65.5;      % [counts/(deg/s)] scale factor at +/-500 dps
gyro.noise_dps   = 0.05;      % [deg/s rms] white noise per sample
gyro.bias0_dps   = 1.2;       % [deg/s] turn-on bias (different every power-up)
gyro.bias_rw_dps = 0.05;      % [deg/s/sqrt(s)] bias random walk (slow drift)
gyro.warmup_dps  = 0.6;       % [deg/s] extra bias as the chip warms up after release
gyro.warmup_tau  = 5;         % [s] warm-up time constant
gyro.rate_hz     = 400;       % [Hz] sample rate

% ---- ACCELEROMETER (inside the IMU, +/-4 g range) ----------------------
%  Measures SPECIFIC FORCE (gravity + the vehicle's own acceleration) along
%  two body axes: x = tangential, z = along the body.
%  At rest, gravity alone tells us the tilt: theta = atan2(-ax, az).
%  Output: int16 counts. 8192 counts = 1 g.
acc.lsb_per_g = 8192;         % [counts/g]
acc.noise_g   = 0.004;        % [g rms] sensor noise
acc.vib_g     = 0.20;         % [g rms] EDF vibration at our throttle (MEASURE THIS)
acc.rate_hz   = 400;          % [Hz]

% ---- MAGNETOMETER (9-axis IMU such as ICM-20948; MPU-6050 has none) ----
%  Measures Earth's magnetic field in body axes. Because the field is fixed
%  in the room, its apparent direction in the body tells us the tilt too.
%  Assumes the pivot axis points East-West, so tilting rotates the field
%  within the body x-z plane.
%  Output: int16 counts. 1 count = 0.15 uT.
mag.uT_per_lsb = 0.15;        % [uT/count]
mag.field_uT   = [21; -48];   % [uT] Earth field in room frame [north/x; up/z] (~St. Louis)
mag.noise_uT   = 0.6;         % [uT rms]
mag.motor_uT   = [6; 4];      % [uT] offset from EDF motor magnets/current (body frame)
mag.ripple_uT  = 3;           % [uT rms] fluctuating motor interference
mag.stray_uT   = [15; 10];    % [uT] stray object near the exhibit (room frame), during magdist_t
mag.rate_hz    = 100;         % [Hz] magnetometers are usually slower

% ---- PIVOT ENCODER (AS5600 magnetic rotary encoder on the pivot shaft) --
%  Measures the angle directly with no drift and no motion error, but only
%  to 12-bit resolution. In a real rocket this sensor can't exist; on our
%  stand it is the perfect way to check the IMU filter.
enc.counts_per_rev = 4096;    % 12-bit
enc.offset_counts  = 1717;    % reading when upright (arbitrary mounting)
enc.rate_hz        = 400;

%% ======================================================================
%  KALMAN FILTER SETUP
%  ======================================================================
%  PREDICT model (linearized physics, discretized with step dt):
%     theta(k+1) = theta + dt*omega
%     omega(k+1) = omega + dt*(a*theta + b*delta)      <- uses the gimbal command
%     bias(k+1)  = bias                                 <- assumed constant...
%  ...and Q says how wrong each line may be per step.
F = [1      dt  0;
     a_lin*dt 1 0;
     0      0   1];
B = [0; b_lin*dt; 0];

sig_alpha = 4.0;                    % [rad/s^2] unmodeled angular accel (pushes, friction, sin vs linear)
sig_brw   = deg2rad(gyro.bias_rw_dps);
Q = diag([1e-10, sig_alpha^2*dt, sig_brw^2*dt]);

%  MEASUREMENT models  z = H*x + noise(R)
H_gyro = [0 1 1];       % gyro reads true rate PLUS bias
H_ang  = [1 0 0];       % accel / mag / encoder each give an angle

%  R = sensor noise variance, derived from the sensor specs above.
%  Small-angle rule: an error of e (in g) across the axis tilts the
%  accel angle by about e radians; for the mag it is e_uT/|field|.
R_gyro    = deg2rad(gyro.noise_dps)^2;
R_acc0    = acc.noise_g^2 + acc.vib_g^2;                       % ~11 deg rms per sample!
R_mag     = (mag.noise_uT^2 + mag.ripple_uT^2)/sum(mag.field_uT.^2);
R_enc     = (2*pi/enc.counts_per_rev)^2/12;   % quantization noise
gate_nis  = 9;                   % reject a measurement if it is >3 sigma off

%% ======================================================================
%  LOGS
%  ======================================================================
z  = nan(1, N);
L = struct('theta',z,'omega',z,'bias',z,'delta',z, ...
           'x_hat',nan(3,N),'P_diag',nan(3,N), ...
           'th_acc',z,'th_mag',z,'th_enc',z,'th_gyro',z,'gyro_rate',z, ...
           'R_acc',z,'mag_rejected',false(1,N),'acc_rejected',false(1,N));

%% ======================================================================
%  STATE OF THE "REAL WORLD" (truth) AND OF THE FLIGHT SOFTWARE
%  ======================================================================
theta = 0; omega = 0; delta = 0;              % truth
bias_true = deg2rad(gyro.bias0_dps);          % truth gyro bias (unknown to software)

x_hat   = [0; 0; 0];                          % filter estimate
P       = diag([deg2rad(2)^2, deg2rad(5)^2, deg2rad(0.5)^2]);
d_cmd   = 0;                                  % gimbal command
d_model = 0;                                  % filter's copy of the servo position
th_gyro = 0;                                  % gyro-only integration (for comparison)

cal.gyro_sum = 0; cal.mag_sum = [0;0]; cal.n = 0; cal.done = false;
cal.gyro_bias = 0; cal.mag_ref_ang = 0;

mag_div = round(gyro.rate_hz/mag.rate_hz);    % mag gets a new sample every 4th loop

%% ======================================================================
%  MAIN LOOP - each iteration is one 400 Hz tick on the Teensy
%  ======================================================================
for k = 1:N
    tk   = t(k);
    held = tk < t_release;                    % being held during calibration?
    edf_on = true;                            % fan runs the whole time

    % ------------------------------------------------------------------
    % (A) REAL WORLD: physics advances by dt (10 substeps for accuracy)
    % ------------------------------------------------------------------
    alpha = 0;
    if ~held
        tau_push = sum(push_tau.*(tk >= push_t0 & tk < push_t0 + push_dur));
        hsub = dt/10;
        for s = 1:10
            delta = delta + hsub*(d_cmd - delta)/tau_servo;          % servo lag
            alpha = (veh.T*veh.L*sin(delta) + veh.m*veh.g*veh.h*sin(theta) ...
                     - veh.b*omega + tau_push)/veh.I;
            omega = omega + hsub*alpha;
            theta = theta + hsub*omega;
            if abs(theta) > veh.stop                                  % hit the stop
                theta = sign(theta)*veh.stop;  omega = 0;
            end
        end
    end
    bias_true = bias_true + deg2rad(gyro.bias_rw_dps)*sqrt(dt)*randn;  % random drift
    if ~held                                                           % warm-up drift
        bias_true = bias_true + deg2rad(gyro.warmup_dps)/gyro.warmup_tau ...
                    *exp(-(tk - t_release)/gyro.warmup_tau)*dt;
    end

    % ------------------------------------------------------------------
    % (B) READ SENSORS: what the Teensy actually receives (raw counts)
    % ------------------------------------------------------------------
    gyro_raw = read_gyro(omega, bias_true, gyro);
    acc_raw  = read_accel(theta, omega, alpha, edf_on, veh, acc);
    new_mag  = mod(k-1, mag_div) == 0;
    if new_mag
        stray = tk >= magdist_t(1) && tk < magdist_t(2);
        mag_raw = read_mag(theta, edf_on, stray, mag);
    end
    enc_raw  = read_encoder(theta, enc);

    % ------------------------------------------------------------------
    % (C) CONVERT counts -> physical units
    % ------------------------------------------------------------------
    gyro_rads = deg2rad(double(gyro_raw)/gyro.lsb_per_dps);        % [rad/s]
    acc_g     = double(acc_raw)/acc.lsb_per_g;                     % [g], [x; z]
    if new_mag, mag_uT = double(mag_raw)*mag.uT_per_lsb; end       % [uT], [x; z]
    th_enc    = wrap_pi((double(enc_raw) - enc.offset_counts)*2*pi/enc.counts_per_rev);

    % ------------------------------------------------------------------
    % (D) STARTUP CALIBRATION while held still
    %     gyro: average reading = bias.  mag: remember the field direction.
    % ------------------------------------------------------------------
    if held
        cal.gyro_sum = cal.gyro_sum + gyro_rads;
        if new_mag, cal.mag_sum = cal.mag_sum + mag_uT; end
        cal.n = cal.n + 1;
    elseif ~cal.done
        cal.gyro_bias   = cal.gyro_sum/cal.n;
        cal.mag_ref_ang = atan2(cal.mag_sum(1), cal.mag_sum(2));
        x_hat(3) = cal.gyro_bias;            % filter starts from the calibrated bias
        cal.done = true;
        fprintf('Calibration: gyro bias %.3f deg/s (true %.3f), mag ref %.1f deg\n', ...
            rad2deg(cal.gyro_bias), rad2deg(bias_true), rad2deg(cal.mag_ref_ang));
    end

    if ~held     % ---------- everything below runs only after release ----------

        % ------------------------------------------------------------------
        % (E) TURN EACH SENSOR INTO A MEASUREMENT OF THE STATE
        % ------------------------------------------------------------------
        % Accelerometer -> tilt. First remove the vehicle's own motion using
        % the filter's estimate (r*alpha tangential, r*omega^2 centripetal),
        % which is a big help when the IMU is far from the pivot.
        alpha_hat = a_lin*x_hat(1) + b_lin*d_model;
        w_hat     = x_hat(2);
        ax = acc_g(1)*veh.g - veh.r_imu*alpha_hat;
        az = acc_g(2)*veh.g + veh.r_imu*w_hat^2;
        th_acc = atan2(-ax, az);
        % Trust the accel less when total |accel| is far from 1 g (shaking, bumps)
        g_err = abs(norm(acc_g) - 1);
        R_acc = R_acc0*(1 + (g_err/0.2)^2);

        % Magnetometer -> tilt (only on ticks with a new sample)
        if new_mag
            th_mag = wrap_pi(cal.mag_ref_ang - atan2(mag_uT(1), mag_uT(2)));
        end

        % Gyro-only angle, just to show why integration alone drifts
        th_gyro = th_gyro + (gyro_rads - cal.gyro_bias)*dt;

        % ------------------------------------------------------------------
        % (F) KALMAN FILTER
        % ------------------------------------------------------------------
        % PREDICT: push the estimate forward with physics + the gimbal command
        d_model = d_model + dt*(d_cmd - d_model)/tau_servo;
        x_hat = F*x_hat + B*d_model;
        P     = F*P*F' + Q;

        % UPDATE 1: gyro (every tick)
        [x_hat, P] = kf_update(x_hat, P, gyro_rads, H_gyro, R_gyro, inf);

        % UPDATE 2: accelerometer angle (every tick, gated)
        [x_hat, P, ok] = kf_update(x_hat, P, th_acc, H_ang, R_acc, gate_nis);
        L.acc_rejected(k) = ~ok;

        % UPDATE 3: magnetometer angle (only when a new sample arrived, gated)
        if new_mag
            [x_hat, P, ok] = kf_update(x_hat, P, th_mag, H_ang, R_mag, gate_nis);
            L.mag_rejected(k) = ~ok;
        end

        % UPDATE 4 (optional): pivot encoder
        if USE_ENCODER
            [x_hat, P] = kf_update(x_hat, P, th_enc, H_ang, R_enc, inf);
        end

        % ------------------------------------------------------------------
        % (G) CONTROL: PD on the filtered estimate (rate = gyro minus bias)
        % ------------------------------------------------------------------
        d_cmd = -Kp*x_hat(1) - Kd*x_hat(2);
        d_cmd = min(max(d_cmd, -delta_max), delta_max);

    end      % ~held

    % ------------------------------------------------------------------
    % (H) LOG for plotting
    % ------------------------------------------------------------------
    L.theta(k) = theta;   L.omega(k) = omega;   L.bias(k) = bias_true;
    L.delta(k) = delta;   L.x_hat(:,k) = x_hat; L.P_diag(:,k) = diag(P);
    L.gyro_rate(k) = gyro_rads;  L.th_enc(k) = th_enc;
    if ~held
        L.th_gyro(k) = th_gyro;  L.th_acc(k) = th_acc;  L.R_acc(k) = R_acc;
        if new_mag, L.th_mag(k) = th_mag; end
    end
end

%% ======================================================================
%  RESULTS
%  ======================================================================
after = t >= t_release;
err   = @(est) rad2deg(sqrt(mean((est(after) - L.theta(after)).^2, 'omitnan')));
fprintf('\nRMS angle error after release (truth = simulated physics):\n');
fprintf('  gyro only (integrated)  : %6.2f deg\n', err(L.th_gyro));
fprintf('  accelerometer only      : %6.2f deg\n', err(L.th_acc));
fprintf('  magnetometer only       : %6.2f deg\n', err(L.th_mag));
fprintf('  pivot encoder           : %6.2f deg\n', err(L.th_enc));
fprintf('  KALMAN FILTER           : %6.2f deg\n', err(L.x_hat(1,:)));
fprintf('  accel updates rejected  : %d,  mag updates rejected: %d of %d\n', ...
    sum(L.acc_rejected), sum(L.mag_rejected), sum(after & mod(0:N-1, mag_div) == 0));

d = @rad2deg;
figure('Name', 'Sensor fusion: angle', 'Position', [60 80 1000 700]);
subplot(2,1,1); hold on; grid on;
plot(t, d(L.th_acc), 'Color', [0.80 0.80 0.80]);
plot(t, d(L.th_mag), '.', 'Color', [0.95 0.70 0.40], 'MarkerSize', 4);
plot(t, d(L.th_gyro), 'Color', [0.55 0.75 1.00], 'LineWidth', 1.2);
plot(t, d(L.theta), 'k', 'LineWidth', 2);
plot(t, d(L.x_hat(1,:)), 'r', 'LineWidth', 1.3);
plot(t, d(L.th_enc), 'g--', 'LineWidth', 1);
xline(t_release, ':', 'release');
for tp = push_t0, xline(tp, ':', 'push'); end
xregion_patch(gca, magdist_t, [-15 15]);
rj = L.mag_rejected;
plot(t(rj), d(L.th_mag(rj)), 'mx', 'MarkerSize', 4);
ylabel('tilt \theta [deg]'); ylim([-15 15]);
legend('accelerometer only', 'magnetometer only', 'gyro integrated', ...
       'TRUE angle', 'Kalman estimate', 'pivot encoder', 'mag rejected by gate', ...
       'Location', 'southwest');
title('Each sensor alone vs. the fused Kalman estimate');

subplot(2,1,2); hold on; grid on;
e  = d(L.x_hat(1,:) - L.theta);
s3 = 3*d(sqrt(L.P_diag(1,:)));
fill([t fliplr(t)], [s3 fliplr(-s3)], [1 0.85 0.85], 'EdgeColor', 'none');
plot(t, e, 'r');
xline(t_release, ':');
for tp = push_t0, xline(tp, ':'); end
ylabel('estimate error [deg]'); xlabel('time [s]'); ylim([-3 3]);
legend('\pm3\sigma (filter''s own uncertainty)', 'actual error');
title('If the red line stays inside the band, the filter is "consistent"');

figure('Name', 'Sensor fusion: rate and bias', 'Position', [1080 80 700 700]);
subplot(3,1,1); hold on; grid on;
plot(t, d(L.gyro_rate), 'Color', [0.6 0.75 1]);
plot(t, d(L.omega), 'k', 'LineWidth', 1.5);
plot(t, d(L.x_hat(2,:)), 'r');
ylabel('\omega [deg/s]'); legend('raw gyro', 'true', 'Kalman');
subplot(3,1,2); hold on; grid on;
plot(t, d(L.bias), 'k', 'LineWidth', 1.5);
plot(t, d(L.x_hat(3,:)), 'r');
plot(t, d(L.x_hat(3,:) + 3*sqrt(L.P_diag(3,:))), 'r:');
plot(t, d(L.x_hat(3,:) - 3*sqrt(L.P_diag(3,:))), 'r:');
ylabel('gyro bias [deg/s]'); legend('true bias', 'estimated', '\pm3\sigma');
subplot(3,1,3); hold on; grid on;
plot(t, d(L.delta), 'b');
ylabel('gimbal \delta [deg]'); xlabel('time [s]');
title('Gimbal angle commanded from the Kalman estimate');

%% ======================================================================
%  LOCAL FUNCTIONS
%  ======================================================================
function raw = read_gyro(omega, bias, gyro)
    % GYROSCOPE: true rate + bias + white noise, quantized to int16 counts.
    % On hardware this is the 2-byte GYRO_ZOUT register.
    dps = rad2deg(omega + bias) + gyro.noise_dps*randn;
    raw = int16(round(dps*gyro.lsb_per_dps));          % int16() saturates like the chip
end

function raw = read_accel(theta, omega, alpha, edf_on, veh, acc)
    % ACCELEROMETER: specific force at the IMU location, body axes [x; z].
    %   x (tangential) = r*alpha   - g*sin(theta)
    %   z (along body) = -r*omega^2 + g*cos(theta)
    % plus sensor noise and EDF vibration (only when the fan is running).
    fx = veh.r_imu*alpha - veh.g*sin(theta);
    fz = -veh.r_imu*omega^2 + veh.g*cos(theta);
    sig = sqrt(acc.noise_g^2 + (edf_on*acc.vib_g)^2);
    g_units = [fx; fz]/veh.g + sig*randn(2,1);
    raw = int16(round(g_units*acc.lsb_per_g));
end

function raw = read_mag(theta, edf_on, stray, mag)
    % MAGNETOMETER: Earth's field rotated into body axes, plus the motor's
    % own field (fixed offset + ripple) while the EDF runs, plus any stray
    % object in the room.
    c = cos(theta); s = sin(theta);
    Rwb = [c -s; s c];                       % room -> body for our tilt convention
    b_body = Rwb*(mag.field_uT + stray*mag.stray_uT) ...
           + edf_on*(mag.motor_uT + mag.ripple_uT*randn(2,1)) ...
           + mag.noise_uT*randn(2,1);
    raw = int16(round(b_body/mag.uT_per_lsb));
end

function raw = read_encoder(theta, enc)
    % PIVOT ENCODER: absolute shaft angle, 12-bit, wraps at one revolution.
    raw = mod(round(enc.offset_counts + theta/(2*pi)*enc.counts_per_rev), enc.counts_per_rev);
end

function [x, P, accepted] = kf_update(x, P, z, H, R, gate)
    % One Kalman measurement update.
    %   y  innovation: what the sensor says minus what we expected
    %   S  how big y should typically be (predicted spread + sensor noise)
    %   K  Kalman gain: how far to move toward the sensor
    % Measurements whose normalized innovation y^2/S exceeds the gate are
    % thrown away (protects against magnetic disturbances, knocks, etc.).
    y = z - H*x;
    S = H*P*H' + R;
    accepted = (y'/S*y) <= gate;
    if ~accepted, return; end
    K = P*H'/S;
    x = x + K*y;
    IKH = eye(numel(x)) - K*H;
    P = IKH*P*IKH' + K*R*K';                 % Joseph form: stays symmetric/positive
end

function xregion_patch(ax, tw, yl)
    % shade a time window (stray magnetic object) without needing R2023a xregion
    h = patch(ax, tw([1 2 2 1]), yl([1 1 2 2]), [1 0.9 1], 'EdgeColor', 'none', ...
              'HandleVisibility', 'off');
    uistack(h, 'bottom');
end

function a = wrap_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end

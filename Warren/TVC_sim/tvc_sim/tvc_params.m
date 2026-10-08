function p = tvc_params()
%TVC_PARAMS  Baseline parameters for the 1-DOF gimbaled-EDF TVC demonstrator.
%
%   p = tvc_params() returns a struct with every number the simulation uses.
%
%   Tags on each line tell you where the number should eventually come from:
%     [EST]  placeholder estimate  -> replace with CAD / bench measurement
%     [SPEC] datasheet value       -> confirm against the part you actually buy
%     [DES]  design choice         -> this is a knob you are free to turn
%
%   Sign convention: theta > 0 is the vehicle tipped toward +x, measured from
%   vertical. delta > 0 is a gimbal deflection that produces a +theta torque.
%   The controller therefore commands negative delta to fight a positive tilt.
%
%   Gains are designed at the bottom of this file from the physical numbers.
%   If you change m, h_cg, I, L, or thrust later, call tvc_design_gains again.

p.g = 9.81;

%% Vehicle (everything that rotates about the pivot) ----------------------
p.m          = 0.55;            % [kg]      total rotating mass (EDF, gimbal, servo, tube, IMU)   [EST]
p.h_cg       = 0.12;            % [m]       pivot -> CG along body axis (+ = above pivot, unstable) [EST]
p.I          = 0.040;           % [kg m^2]  inertia about the pivot axis (CAD mass properties)    [EST]
p.L          = 0.30;            % [m]       pivot -> gimbal axis distance (TVC lever arm)          [EST]
p.b_visc     = 0.002;           % [N m s/rad] viscous bearing drag                                [EST]
p.tau_coul   = 0.003;           % [N m]     Coulomb friction: bearings + wire drag across pivot   [EST]
p.theta_stop = deg2rad(20);     % [rad]     mechanical hard stops                                 [DES]
p.k_stop     = 30;              % [N m/rad] bumper stiffness at the stops                         [EST]
p.c_stop     = 0.6;             % [N m s/rad] bumper damping                                      [EST]

%% Gimbal + servo ---------------------------------------------------------
p.delta_max     = deg2rad(15);           % [rad]  gimbal mechanical limit                        [DES]
p.delta_cmd_max = deg2rad(14);           % [rad]  software command clamp (keep inside mech limit) [DES]
p.Ig            = 1.2e-4;                % [kg m^2] EDF + cradle inertia about gimbal axis (see tvc_servo_sizing) [EST]
p.servo_speed   = 0.10;                  % [s/60deg] no-load speed at your supply voltage        [SPEC]
p.servo_derate  = 0.75;                  % [-]    loaded speed / no-load speed                   [EST]
p.servo_rate    = p.servo_derate*deg2rad(60)/p.servo_speed;  % [rad/s] max slew used in the model
p.servo_kpos    = 40;                    % [1/s]  servo internal position-loop gain              [EST - step test it]
p.servo_tau     = 0.012;                 % [s]    servo velocity/mechanical time constant        [EST]
p.servo_torque  = 0.20;                  % [N m]  usable torque at the gimbal (after linkage)    [SPEC/2 margin]
p.servo_acc     = p.servo_torque/p.Ig;   % [rad/s^2] torque-limited gimbal acceleration
p.servo_refresh = 50;                    % [Hz]   PWM frame rate: 50 analog, 200-333 digital     [SPEC]
p.servo_deadband= deg2rad(0.2);          % [rad]  servo deadband (~2-4 us of pulse width)        [SPEC]

%% EDF thrust -------------------------------------------------------------
% Static map T = T_max * u^thrust_exp, unless you supply measured data in
% p.thrust_table = [u_1 T_1; u_2 T_2; ...] from the load-cell fixture,
% in which case edf_thrust() interpolates it instead.
p.T_max        = 6.5;           % [N]   full-throttle static thrust, FMS 50mm 11-blade on 4S  [EST - MEASURE]
p.thrust_exp   = 1.6;           % [-]   shape of thrust vs throttle curve                     [EST - MEASURE]
p.thrust_table = [];            % [u T] measured throttle/thrust pairs (overrides the power law)
p.tau_edf      = 0.08;          % [s]   spool-up time constant                                 [EST - MEASURE]
p.u_fixed      = 0.75;          % [-]   baseline fixed throttle (keep low for the 70-75 dB limit) [DES]

%% IMU (defaults are MPU-6050-class numbers) ------------------------------
p.r_imu      = 0.20;               % [m]  pivot -> IMU distance; nearer the pivot = less tangential accel error [DES]
p.gyro_nd    = deg2rad(0.005);     % [rad/s/sqrt(Hz)] gyro noise density                       [SPEC]
p.gyro_bw    = 98;                 % [Hz] gyro DLPF bandwidth setting                          [DES]
p.gyro_bias  = deg2rad(0.3);       % [rad/s] residual bias after startup calibration          [EST]
p.accel_nd   = 400e-6*p.g;         % [m/s^2/sqrt(Hz)] accel noise density                      [SPEC]
p.accel_bw   = 98;                 % [Hz] accel DLPF bandwidth                                  [DES]
p.accel_vib  = 2.0;                % [m/s^2 rms] EDF vibration seen by accel at full thrust    [EST - MEASURE, matters a lot]
p.accel_comp    = false;           % subtract r*alpha and r*omega^2 (from gyro) before atan2
p.ideal_sensors = false;           % true = controller sees the true state (no noise/filter)

%% Estimator + controller -------------------------------------------------
p.loop_rate  = 400;             % [Hz] control loop rate                                       [DES]
p.cf_tau     = 1.0;             % [s]  complementary-filter crossover time constant             [DES]
p.d_lpf_hz   = 40;              % [Hz] low-pass on gyro rate used by the D-term                 [DES]
p.one_sample_delay = true;      % output computed this tick is applied next tick (conservative)
p.Ki         = 0;               % [1/s] integral gain (baseline is PD)                          [DES]
p.delta_trim = 0;               % [rad] gimbal trim (fixes thrust-axis misalignment)           [DES]

%% Human pilot model (joystick -> gimbal, for the "kids try it" comparison)
p.human_delay       = 0.25;           % [s]   visual-motor reaction delay (kids ~0.25-0.4 s)
p.human_angle_noise = deg2rad(1.5);   % [rad] how precisely someone can see the tilt
p.human_tremor      = deg2rad(2);     % [rad] random stick motion

%% Simulation -------------------------------------------------------------
p.dt = 2.5e-4;                  % [s] plant integration step (RK4)

%% Gains ------------------------------------------------------------------
p.design_wn   = 12;             % [rad/s] desired closed-loop natural frequency [DES]
p.design_zeta = 0.8;            % [-]     desired closed-loop damping           [DES]
p = tvc_design_gains(p);
end

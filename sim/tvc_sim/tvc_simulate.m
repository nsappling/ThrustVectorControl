function log = tvc_simulate(p, sc)
%TVC_SIMULATE  Hybrid continuous/discrete simulation of the 1-DOF TVC stand.
%
%   log = tvc_simulate(p, sc)
%     p  : parameters from tvc_params
%     sc : scenario from tvc_scenario
%
%   Signal chain modeled (matches the project plan's core loop):
%
%     IMU -> complementary filter -> PD controller -> servo PWM frame ->
%     servo dynamics -> gimbaled EDF -> TVC torque -> vehicle motion
%
%   Continuous plant (RK4 at p.dt), state x = [theta; omega; delta; delta_dot; T]
%     I*theta_dd = T L sin(delta)          TVC torque
%                + m g h sin(theta)        gravity (destabilizing, CG above pivot)
%                - b*omega - tau_c*sgn     bearing + wire friction
%                + tau_stop                rubber hard stops at +/- theta_stop
%                + tau_push                disturbance pulse
%                - Ig*delta_dd             reaction from swinging the EDF
%   Servo: position loop -> rate limit -> velocity lag -> torque limit,
%          with gimbal hard limit at +/- delta_max.
%   EDF:   first-order spool-up toward the static thrust map.
%
%   Discrete parts on their own clocks:
%     p.loop_rate      IMU sample, filter, controller
%     p.servo_refresh  servo latches a new target once per PWM frame
%
%   Not modeled (see README): fan roll torque and gyroscopic torque from the
%   spinning rotor - both act about axes the 1-DOF pivot reacts, so they load
%   the bearings but do not move theta.

if ~isempty(sc.seed), rng(sc.seed); end
% throttle: [] -> p.u_fixed, scalar, or @(t) for throttle-step tests.
% The ESC sees a new throttle once per controller tick.
u = sc.throttle;
if isempty(u), u = p.u_fixed; end
if isa(u, 'function_handle'), Tcmd = edf_thrust(p, u(0)); else, Tcmd = edf_thrust(p, u); end

dt  = p.dt;
dtc = 1/p.loop_rate;
N   = round(sc.T_end/dt);

x = [sc.theta0; 0; 0; 0; Tcmd];

% discrete-side state
t_ctrl   = 0;
t_servo  = 0;
th_hat   = sc.theta0;
w_filt   = 0;
w_prev   = 0;
al_est   = 0;           % angular-accel estimate for accel compensation
e_int    = 0;
d_prev   = 0;           % command computed last tick (for one-sample delay)
d_out    = 0;           % command currently on the PWM line
d_target = 0;           % target latched by the servo

a_cf  = p.cf_tau/(p.cf_tau + dtc);
b_lpf = dtc/(dtc + 1/(2*pi*p.d_lpf_hz));
sg    = p.gyro_nd*sqrt(p.gyro_bw);
sa    = p.accel_nd*sqrt(p.accel_bw);

nDelay = max(1, round(p.human_delay/dtc));
hbuf   = repmat([sc.theta0; 0], 1, nDelay);
hidx   = 1;

nLog = floor(sc.T_end/dtc) + 2;
z = zeros(nLog, 1);
log = struct('t',z,'theta',z,'omega',z,'theta_hat',z,'theta_acc',z, ...
             'omega_meas',z,'delta_cmd',z,'delta',z,'thrust',z, ...
             'tau_push',z,'ref',z);
kl = 0;

for k = 0:N-1
    t = k*dt;

    % ---------------- controller tick ----------------------------------
    if t >= t_ctrl - 1e-9
        t_ctrl = t_ctrl + dtc;
        if isa(u, 'function_handle'), Tcmd = edf_thrust(p, u(t)); end
        [~, alpha] = plant_deriv(x, d_target, Tcmd, p, sc, t);
        th = x(1); w = x(2);

        % IMU: gyro + accelerometer specific force at r_imu on the body.
        % Body-frame components: tangential (ft) and along-axis (fb).
        % ft picks up r*alpha and fb picks up -r*omega^2, which is why the
        % accel-only angle is wrong whenever the vehicle is accelerating.
        gyro = w + p.gyro_bias + sg*randn;
        sv   = sqrt(sa^2 + (p.accel_vib*x(5)/p.T_max)^2);
        ft   = p.r_imu*alpha - p.g*sin(th) + sv*randn;
        fb   = -p.r_imu*w^2  + p.g*cos(th) + sv*randn;
        if p.accel_comp
            % remove the motion terms using the gyro's own estimate of
            % omega and alpha, leaving (ideally) just gravity
            ft = ft - p.r_imu*al_est;
            fb = fb + p.r_imu*w_filt^2;
        end
        th_acc = atan2(-ft, fb);

        th_hat = a_cf*(th_hat + gyro*dtc) + (1 - a_cf)*th_acc;
        w_filt = w_filt + b_lpf*(gyro - w_filt);
        al_est = al_est + b_lpf*((w_filt - w_prev)/dtc - al_est);
        w_prev = w_filt;

        if p.ideal_sensors
            th_meas = th;  w_meas = w;
        else
            th_meas = th_hat;  w_meas = w_filt;
        end

        ref = sc.theta_ref;
        if isa(ref, 'function_handle'), ref = ref(t); end

        switch sc.mode
            case 'off'
                d_new = 0;
            case 'pd'
                e = ref - th_meas;
                if p.Ki > 0
                    e_int = e_int + e*dtc;
                    lim   = p.delta_cmd_max/p.Ki;     % anti-windup
                    e_int = min(max(e_int, -lim), lim);
                end
                d_new = p.Kp*e - p.Kd*w_meas + p.Ki*e_int + p.delta_trim;
            case 'human'
                % person sees the true tilt (with limited precision),
                % reacts after human_delay, and has some tremor
                old = hbuf(:, hidx);
                hbuf(:, hidx) = [th + p.human_angle_noise*randn; w];
                hidx = mod(hidx, nDelay) + 1;
                d_new = p.human_Kp*(ref - old(1)) - p.human_Kd*old(2) ...
                        + p.human_tremor*randn;
            otherwise
                error('tvc_simulate: unknown mode "%s"', sc.mode);
        end
        d_new = min(max(d_new, -p.delta_cmd_max), p.delta_cmd_max);

        if p.one_sample_delay
            d_out = d_prev;  d_prev = d_new;
        else
            d_out = d_new;
        end

        kl = kl + 1;
        log.t(kl) = t;            log.theta(kl) = th;     log.omega(kl) = w;
        log.theta_hat(kl) = th_hat; log.theta_acc(kl) = th_acc;
        log.omega_meas(kl) = w_filt; log.delta_cmd(kl) = d_out;
        log.delta(kl) = x(3);     log.thrust(kl) = x(5);
        log.tau_push(kl) = push_torque(sc, t);  log.ref(kl) = ref;
    end

    % ---------------- servo PWM frame ----------------------------------
    if t >= t_servo - 1e-9
        t_servo = t_servo + 1/p.servo_refresh;
        if abs(d_out - d_target) > p.servo_deadband
            d_target = d_out;
        end
    end

    % ---------------- integrate plant (RK4) ----------------------------
    k1 = plant_deriv(x,           d_target, Tcmd, p, sc, t);
    k2 = plant_deriv(x + dt/2*k1, d_target, Tcmd, p, sc, t + dt/2);
    k3 = plant_deriv(x + dt/2*k2, d_target, Tcmd, p, sc, t + dt/2);
    k4 = plant_deriv(x + dt*k3,   d_target, Tcmd, p, sc, t + dt);
    x  = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    % gimbal hard limit
    if x(3) > p.delta_max
        x(3) = p.delta_max;  x(4) = min(x(4), 0);
    elseif x(3) < -p.delta_max
        x(3) = -p.delta_max; x(4) = max(x(4), 0);
    end
end

f = fieldnames(log);
for i = 1:numel(f), log.(f{i}) = log.(f{i})(1:kl); end
log.p  = p;
log.sc = sc;
end

% =======================================================================
function [dx, alpha] = plant_deriv(x, d_tgt, Tcmd, p, sc, t)
th = x(1); w = x(2); d = x(3); dd = x(4); T = x(5);

% servo: position loop -> slew limit -> velocity lag -> torque limit
dd_des = min(max(p.servo_kpos*(d_tgt - d), -p.servo_rate), p.servo_rate);
dd_acc = min(max((dd_des - dd)/p.servo_tau, -p.servo_acc), p.servo_acc);

tau_tvc  = T*p.L*sin(d);
tau_grav = p.m*p.g*p.h_cg*sin(th);
tau_fric = p.b_visc*w + p.tau_coul*tanh(w/0.01);
tau_stop = 0;
if th > p.theta_stop
    tau_stop = min(0, -p.k_stop*(th - p.theta_stop) - p.c_stop*w);
elseif th < -p.theta_stop
    tau_stop = max(0, -p.k_stop*(th + p.theta_stop) - p.c_stop*w);
end
tau_react = -p.Ig*dd_acc;

alpha = (tau_tvc + tau_grav - tau_fric + tau_stop + push_torque(sc, t) + tau_react)/p.I;
dx = [w; alpha; dd; dd_acc; (Tcmd - T)/p.tau_edf];
end

function tau = push_torque(sc, t)
if t >= sc.push_t0 && t < sc.push_t0 + sc.push_dur
    tau = sc.push_tau;
else
    tau = 0;
end
end

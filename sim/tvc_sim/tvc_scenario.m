function sc = tvc_scenario(mode, varargin)
%TVC_SCENARIO  Build a test scenario for tvc_simulate.
%
%   sc = tvc_scenario('pd')     controller ON  (Test 2 in the project plan)
%   sc = tvc_scenario('off')    controller OFF, gimbal held at 0 (Test 1)
%   sc = tvc_scenario('human')  a person flying the gimbal with a joystick
%
%   Name/value overrides, e.g.
%   sc = tvc_scenario('pd', 'push_tau', 0.4, 'theta_ref', deg2rad(5));
%
%   Fields:
%     T_end      [s]    simulation length
%     theta0     [rad]  initial tilt
%     theta_ref  [rad]  commanded angle, scalar or @(t) function handle
%     throttle   [-]    fixed throttle; [] means p.u_fixed
%     push_tau   [N m]  disturbance torque pulse ("the flick")
%     push_t0    [s]    pulse start
%     push_dur   [s]    pulse length
%     seed       rng seed so noisy runs are repeatable

sc.mode      = mode;
sc.T_end     = 4;
sc.theta0    = 0;
sc.theta_ref = 0;
sc.throttle  = [];
sc.push_tau  = 0.30;
sc.push_t0   = 0.5;
sc.push_dur  = 0.10;
sc.seed      = 1;

for i = 1:2:numel(varargin)
    if ~isfield(sc, varargin{i})
        error('tvc_scenario: unknown field "%s"', varargin{i});
    end
    sc.(varargin{i}) = varargin{i+1};
end
end

function p = tvc_design_gains(p, wn, zeta, u)
%TVC_DESIGN_GAINS  PD gains by pole placement on the linearized plant.
%
%   p = tvc_design_gains(p)               uses p.design_wn, p.design_zeta, p.u_fixed
%   p = tvc_design_gains(p, wn, zeta)     overrides the targets
%   p = tvc_design_gains(p, wn, zeta, u)  designs at throttle u
%
%   Linearized about upright, small angles, fixed thrust T:
%       I*theta_dd = m g h * theta + T L * delta
%       theta_dd   =     a * theta +     b * delta,   a = mgh/I,  b = TL/I
%   With delta = -Kp*theta - Kd*theta_d the closed loop is
%       s^2 + b Kd s + (b Kp - a) = 0
%   so matching s^2 + 2 zeta wn s + wn^2 gives
%       Kp = (wn^2 + a)/b,   Kd = 2 zeta wn / b
%   Kp must exceed a/b just to stop the vehicle falling over.
%
%   Also sets "best case human" gains for the joystick model: a skilled
%   person would try for a slow, well-damped response.

if nargin < 2 || isempty(wn),   wn   = p.design_wn;   end
if nargin < 3 || isempty(zeta), zeta = p.design_zeta; end
if nargin < 4 || isempty(u),    u    = p.u_fixed;     end

T = edf_thrust(p, u);
a = p.m*p.g*p.h_cg/p.I;
b = T*p.L/p.I;

p.Kp = (wn^2 + a)/b;
p.Kd = 2*zeta*wn/b;

wh = 1.3*sqrt(a);
p.human_Kp = (wh^2 + a)/b;
p.human_Kd = 2*0.7*wh/b;
end

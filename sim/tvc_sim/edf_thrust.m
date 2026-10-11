function T = edf_thrust(p, u)
%EDF_THRUST  Static EDF thrust [N] at throttle u in [0,1].
%   Uses measured load-cell data in p.thrust_table = [u T] if present,
%   otherwise the power law T = T_max * u^thrust_exp.

u = min(max(u, 0), 1);
if ~isempty(p.thrust_table)
    T = interp1(p.thrust_table(:,1), p.thrust_table(:,2), u, 'linear', 'extrap');
    T = max(T, 0);
else
    T = p.T_max * u.^p.thrust_exp;
end
end

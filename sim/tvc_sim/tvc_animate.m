function tvc_animate(log, speed)
%TVC_ANIMATE  Side-view animation of a run (for presentations / sanity checks).
%   tvc_animate(log)          real time
%   tvc_animate(log, 0.25)    quarter speed

if nargin < 2, speed = 1; end
p = log.p;
Lb  = max(p.L, p.h_cg)*1.25;     % body length drawn
fl  = 0.10;                      % flame/thrust arrow length per full thrust

figure('Name', 'TVC animation', 'Color', 'w');
ax = axes; hold(ax, 'on'); axis(ax, 'equal'); axis(ax, [-0.4 0.4 -0.1 0.6]); grid(ax, 'on');
plot(ax, [-0.4 0.4], [0 0], 'k', 'LineWidth', 2);
plot(ax, 0, 0, 'ko', 'MarkerFaceColor', 'k');
for s = [-1 1]
    plot(ax, [0 Lb*sin(s*p.theta_stop)], [0 Lb*cos(p.theta_stop)], 'r:');
end
hb = plot(ax, [0 0], [0 Lb], 'Color', [0.2 0.3 0.6], 'LineWidth', 8);
hc = plot(ax, 0, p.h_cg, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 9);
hg = plot(ax, [0 0], [0 0], 'k', 'LineWidth', 5);
hf = plot(ax, [0 0], [0 0], 'Color', [1 0.5 0], 'LineWidth', 4);
ht = title(ax, '');

tic;
k = 1;
while k <= numel(log.t)
    th = log.theta(k);  d = log.delta(k);  T = log.thrust(k);
    bx = [sin(th) cos(th)];                       % body axis
    gp = p.L*bx;                                  % gimbal point
    tdir = [sin(th + d) cos(th + d)];             % thrust direction
    set(hb, 'XData', [0 Lb*bx(1)], 'YData', [0 Lb*bx(2)]);
    set(hc, 'XData', p.h_cg*bx(1), 'YData', p.h_cg*bx(2));
    set(hg, 'XData', gp(1) + [-1 1]*0.03*tdir(1), 'YData', gp(2) + [-1 1]*0.03*tdir(2));
    % exhaust points opposite to thrust
    ex = gp - fl*T/p.T_max*tdir;
    set(hf, 'XData', [gp(1) ex(1)], 'YData', [gp(2) ex(2)]);
    set(ht, 'String', sprintf('%s   t = %.2f s   \\theta = %+5.1f°   \\delta = %+5.1f°', ...
        log.sc.mode, log.t(k), rad2deg(th), rad2deg(d)));
    drawnow limitrate;
    k = find(log.t >= toc*speed, 1);
    if isempty(k), break; end
end
end

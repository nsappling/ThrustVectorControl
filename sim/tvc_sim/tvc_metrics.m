function m = tvc_metrics(log, band)
%TVC_METRICS  Performance numbers for one run (the ones the final report asks for).
%
%   m = tvc_metrics(log)          settling band = 2 deg
%   m = tvc_metrics(log, band)    band in radians
%
%   Measured from the start of the disturbance pulse:
%     fell          vehicle reached the hard stop
%     max_dev_deg   largest |theta - ref|
%     overshoot_pct largest swing to the other side, as % of max_dev
%     settle_s      time until |theta - ref| stays inside the band (NaN if never)
%     sse_deg       mean error over the last 0.5 s
%     delta_peak_deg, sat_frac   largest gimbal command, fraction of time at the clamp
%     jitter_deg    std of the gimbal command over the last 1 s (sensor noise
%                   leaking into the servo -> wear, heat, and fan noise)

if nargin < 2, band = deg2rad(2); end
p = log.p;  sc = log.sc;
t = log.t;  e = log.theta - log.ref;

i0 = find(t >= sc.push_t0, 1);
if isempty(i0), i0 = 1; end
ee = e(i0:end);  tt = t(i0:end);

m.fell = any(abs(log.theta) >= p.theta_stop - 1e-3);
[mx, imx] = max(abs(ee));
m.max_dev_deg = rad2deg(mx);
s = sign(ee(imx));
after = ee(imx:end);
m.overshoot_pct = 100*max(0, max(-s*after))/max(mx, eps);

out = find(abs(ee) > band, 1, 'last');
if m.fell
    m.settle_s = NaN;
elseif isempty(out)
    m.settle_s = 0;
elseif out == numel(ee)
    m.settle_s = NaN;
else
    m.settle_s = tt(out+1) - sc.push_t0;
end

last = t >= t(end) - 0.5;
m.sse_deg = rad2deg(mean(e(last)));
m.delta_peak_deg = rad2deg(max(abs(log.delta_cmd)));
m.sat_frac = mean(abs(log.delta_cmd) >= p.delta_cmd_max - 1e-6);
last1 = t >= t(end) - 1.0;
m.jitter_deg = rad2deg(std(log.delta_cmd(last1)));
end

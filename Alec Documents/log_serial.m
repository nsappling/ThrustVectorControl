function fname = log_serial(port, duration_s)
%% log_serial.m
% Record the CSV data the Arduino prints over USB (hardware_test.ino or
% tvc_controller.ino) straight into a .csv file, with a live plot.
%
% Usage (from the MATLAB command window):
%   log_serial                  % auto-find the Arduino, log until you close the plot
%   log_serial([], 30)          % auto-find, log for 30 seconds
%   log_serial("/dev/cu.usbserial-140")
%
% IMPORTANT: close the Arduino IDE's Serial Monitor first -- only one
% program can use the port at a time.
%
% Files are saved to Alec Documents/data/log_YYYYMMDD_HHMMSS.csv. Every
% line is written to disk as it arrives, so nothing is lost if you stop
% early (close the plot window, or Ctrl+C). Load a log afterwards with:
%   d = readtable('data/log_....csv');  plot(d.t_ms/1000, d.angle_deg)
%
% What gets logged: any line the Arduino prints that contains commas.
% The first comma line with letters in it is treated as the column
% header. Lines starting with '#' are status messages -- shown in the
% command window, not saved.

if nargin < 1 || isempty(port), port = find_arduino(); end
if nargin < 2 || isempty(duration_s), duration_s = inf; end

outdir = fullfile(fileparts(mfilename('fullpath')), 'data');
if ~exist(outdir, 'dir'), mkdir(outdir); end
fname = fullfile(outdir, "log_" + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + ".csv");

s = serialport(port, 115200, 'Timeout', 5);
configureTerminator(s, "CR/LF");   % Arduino println ends lines with \r\n
flush(s);
% Opening the port resets the Arduino, so its setup() runs again and it
% re-prints the header -- we just wait for it below.

fid = fopen(fname, 'w');
cleanup = onCleanup(@() finish(fid, fname));   % runs even on Ctrl+C

fprintf('Logging from %s to\n  %s\nClose the plot window (or Ctrl+C) to stop.\n', port, fname);

header = [];
lines = gobjects(0);
fig = [];
nrows = 0;
t_start = tic;

while toc(t_start) < duration_s && (isempty(fig) || isvalid(fig))
    line = readline(s);
    if isempty(line)
        fprintf('(no data for 5 s -- is the Arduino running and the Serial Monitor closed?)\n');
        continue
    end
    line = strtrim(line);

    if startsWith(line, "#")
        fprintf('%s\n', line);            % status message from the Arduino
        continue
    end
    if ~contains(line, ",")
        continue
    end

    if isempty(header)
        if ~isempty(regexp(line, '[A-Za-z]', 'once'))
            header = split(line, ",")';
            fprintf(fid, '%s\n', line);
            [fig, lines] = make_plot(header);
        end
        continue                          % skip data that arrives before a header
    end
    if ~isempty(regexp(line, '[A-Za-z]', 'once'))
        continue                          % header re-printed (e.g. board reset)
    end

    vals = str2double(split(line, ","))';
    if numel(vals) ~= numel(header) || any(isnan(vals))
        continue                          % garbled/partial line
    end
    fprintf(fid, '%s\n', line);
    nrows = nrows + 1;

    % Live plot: column 1 (time) on x, one strip per other column
    t = vals(1) / 1000;
    for k = 1:numel(lines)
        addpoints(lines(k), t, vals(k+1));
    end
    if mod(nrows, 5) == 0
        drawnow limitrate;
    end
end

clear s   % release the serial port
fprintf('Recorded %d rows.\n', nrows);
end

% =====================================================================
% LOCAL FUNCTIONS
% =====================================================================

function port = find_arduino()
    ports = serialportlist("available");
    hit = ports(contains(ports, "usbserial") | contains(ports, "usbmodem") ...
              | contains(ports, "ttyUSB") | contains(ports, "ttyACM") | startsWith(ports, "COM"));
    hit = hit(~contains(hit, "Bluetooth"));
    if isempty(hit)
        error('log_serial:noPort', ['No Arduino port found (is it plugged in, and the ' ...
              'Serial Monitor closed?). Available: %s'], strjoin(ports, ', '));
    end
    port = hit(1);
end

function [fig, lines] = make_plot(header)
    n = numel(header) - 1;
    fig = figure('Name', 'Serial log (close to stop)', 'NumberTitle', 'off');
    tl = tiledlayout(fig, n, 1, 'TileSpacing', 'tight');
    lines = gobjects(1, n);
    for k = 1:n
        ax = nexttile(tl);
        lines(k) = animatedline(ax, 'MaximumNumPoints', 600);   % last ~60 s at 10 Hz
        ylabel(ax, header(k+1), 'Interpreter', 'none');
        grid(ax, 'on');
        if k < n, ax.XTickLabel = []; end
    end
    xlabel(nexttile(tl, n), 'time [s]');
end

function finish(fid, fname)
    fclose(fid);
    fprintf('Saved %s\n', fname);
end

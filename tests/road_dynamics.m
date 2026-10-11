%% TUKZIE V2X VANET (TVV) - ROAD DYNAMICS
% Speed and IMU readings from a node's sensing log (sensing.txt on its SD Card)
%
% Input (in the "logs" folder next to this script):
%   sensing.txt
%
% Log line (formatSensing in diagnostics.cpp), 16 fields, every SENSE_LOG_INTVL (1 s):
%   ms,date,time,lat,lon,alt,speed,course,sats,hdop,ax,ay,az,gx,gy,gz
%   speed: fused GPS + IMU (kmph) | ax, ay, az: raw acceleration (g, gravity included) | gx, gy, gz: angular rate (dps)
%
% Figures (saved as .png and .fig in the "figures" folder):
%   1. Speed
%   2. IMU acceleration
%   3. IMU gyroscope

clear; close all; clc;

%% Settings
scriptDir = fileparts(mfilename('fullpath'));
logFile = fullfile(scriptDir, 'logs', 'sensing.txt');
figDir = fullfile(scriptDir, 'figures');
if ~isfolder(figDir)
    mkdir(figDir);
end

%% Read the log
S = readSensingLog(logFile);
t = (S.ms - S.ms(1)) / 1000;    % s since the first line

fprintf('%d samples over %.1f s\n', height(S), t(end));

%% Figure 1: Speed
fig = figure('Name', 'Speed', 'Color', 'w');
plot(t, S.speed, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Speed (km/h)');
title('Speed (GPS + IMU fusion)');
saveFigure(fig, figDir, 'road_speed');

%% Figure 2: IMU acceleration
fig = figure('Name', 'IMU acceleration', 'Color', 'w');
hold on; grid on;
plot(t, S.ax, 'LineWidth', 1);
plot(t, S.ay, 'LineWidth', 1);
plot(t, S.az, 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('IMU acceleration (gravity included)');
legend('x', 'y', 'z', 'Location', 'best');
saveFigure(fig, figDir, 'road_acceleration');

%% Figure 3: IMU gyroscope
fig = figure('Name', 'IMU gyroscope', 'Color', 'w');
hold on; grid on;
plot(t, S.gx, 'LineWidth', 1);
plot(t, S.gy, 'LineWidth', 1);
plot(t, S.gz, 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('IMU gyroscope');
legend('x', 'y', 'z', 'Location', 'best');
saveFigure(fig, figDir, 'road_gyroscope');

fprintf('Figures saved to %s\n', figDir);

%% ------------------------------------------------------------------------------------------------
%% Local functions
%% ------------------------------------------------------------------------------------------------

function S = readSensingLog(file)
% Reads a sensing log into a table (lines without exactly 16 fields are skipped)

    if ~isfile(file)
        error('road_dynamics:missingLog', 'Log file not found: %s', file);
    end

    lines = strtrim(readlines(file));
    lines = lines(strlength(lines) > 0);
    lines = lines(count(lines, ",") == 15);     % 16 fields: skips partial lines
    if isempty(lines)
        error('road_dynamics:emptyLog', 'No valid log lines in %s (expected 16 fields per line)', file);
    end

    parts = split(lines, ",");
    if isscalar(lines)
        parts = parts.';                         % split returns a column for a single line
    end

    S = array2table(double(parts), 'VariableNames', ...
        {'ms', 'date', 'time', 'lat', 'lon', 'alt', 'speed', 'course', 'sats', 'hdop', ...
         'ax', 'ay', 'az', 'gx', 'gy', 'gz'});
end

function saveFigure(fig, figDir, name)
% Saves a figure as .png (200 dpi) and .fig

    set(findall(fig, 'Type', 'axes'), 'Toolbar', []);     % keep the axes toolbar out of the image
    exportgraphics(fig, fullfile(figDir, [name '.png']), 'Resolution', 200);
    savefig(fig, fullfile(figDir, [name '.fig']));
end

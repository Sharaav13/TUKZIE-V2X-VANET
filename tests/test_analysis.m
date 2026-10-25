%% TUKZIE V2X VANET (TVV) - TEST ANALYSIS
% Plots and data from the field tests in the folders next to this script
%
% Output:
%   plots/<title>.png   every figure, named after its title
%   test_results.txt    everything printed to the Command Window
%
% Logs (one set per node, n = 1 Alice, n = 2 Bob):
%   sensor_log<n>.txt   ms,date,time,lat,lon,alt,speed,course,sats,hdop,ax,ay,az,gx,gy,gz,message
%   myself_log<n>.txt   ms,id,date,time,lat,lon,alt,speed,course,status,tasks,battery,message,seq,srcRssi,pktRssi
%   vanet_log<n>.txt    ms,event,src,date,time,lat,lon,alt,speed,course,status,tasks,battery,message,seq,srcRssi,
%                       pktRssi,snr,freqErr,utc,airUs,sync,ttl
%   In RX lines date..seq are the sender's (copied from the packet), utc and sync are the receiver's (RxDone)
%   Times are SAST (HHMMSSCC), utc is us since UTC midnight (PPS time base, sync 1 = locked, 2 = holdover)
%
% A packet's time is its TX start from the PPS time base when synced, otherwise the GPS time in its TX line
%
% Message codes are those of the firmware used for the tests (committed 19 Oct 2026):
%   1 accident, 2 low battery, 3 rough terrain, 4 potholes, 5 host message

clear; close all; clc;

%% User inputs ------------------------------------------------------------------------------------------------------
% Black lines: one row per line, {time, label}. Times are SAST 'HH:MM:SS' (or 'HH:MM'), '' = no label, e.g.
%   {'13:54:12', 'Speed bump'
%    '13:55:40', 'Pothole'}

% gw parking lot (run 1): bumps you drove over
GW_BUMPS = {};

% howard centre (undercover laps, outside laps) and tukzie test: lines on the RSSI/SNR/delivery figures
HOWARD_UNDERCOVER_MARKS = {'13:10', 'Lap 1' 
                            '13:14', 'Lap 2'
                            '13:18', 'Lap 3'
                            '13:21', 'End'};
HOWARD_OUTSIDE_MARKS = {'13:39', 'Lap 1'
                        '13:43', 'Lap 2'
                        '13:46', 'Lap 3'
                        '13:50', 'End'};
TUKZIE_MARKS = {'18:26', 'At CME'
                '18:41', 'Ascending Madiba Cir.'
                '18:46', 'Descending Madiba Cir.'
                '18:52', 'Back at CME'};
TUKZIE_EXCLUDE = {'18:41', '18:52'};    % tukzie test: also give the PDR without the packets sent in this interval

% baseline: lines on its RSSI/SNR/delivery figure
BASELINE_MARKS = {};

% howard centre: true location of Bob [lat lon] (degrees), NaN = not supplied
HOWARD_TRUE_UNDERCOVER = [-33.9339027 18.5094547];
HOWARD_TRUE_OUTSIDE    = [-33.9336425 18.5103687];

%% Settings ---------------------------------------------------------------------------------------------------------
ALICE = 1;                  % node IDs (log1 = Alice, log2 = Bob)
BOB = 2;
NAMES = containers.Map({ALICE, BOB}, {'Alice', 'Bob'});

ACCIDENT_CODES = 1;         % message codes drawn as red dashes
TERRAIN_CODES = [3 4];      % rough terrain, potholes: orange dashes

TTL_INIT = 2;               % ttl of a packet heard directly
PDR_BIN = 25;               % packets per PDR bin (uct rugby field)
START_LEVEL = 2;            % menzies staircase: level the climb started from
GW_RUNS = {'run 1'};        % gw parking lot: run folders plotted
MAX_POS_AGE_S = 5;          % s, largest gap to the receiver's nearest GPS fix when finding the distance

C_AB = [0.00 0.45 0.74];    % Alice -> Bob
C_BA = [0.85 0.33 0.10];    % Bob -> Alice
C_ACC = [0.49 0.18 0.56; 0.00 0.45 0.74; 0.47 0.67 0.19];   % x purple, y blue, z green
C_ORANGE = [0.93 0.55 0.10];

testsDir = fileparts(mfilename('fullpath'));
plotDir = fullfile(testsDir, 'plots');
if ~isfolder(plotDir)
    mkdir(plotDir);
end

logFile = fullfile(testsDir, 'test_results.txt');
if isfile(logFile)
    delete(logFile);
end
diary(logFile);
cleanup = onCleanup(@() diary('off'));

fprintf('TVV test analysis, run %s\n', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
fprintf('Alice = node %d (log1), Bob = node %d (log2)\n', ALICE, BOB);

%% gw parking lot ---------------------------------------------------------------------------------------------------
section('GrandWest Casino parking lot');
try
    runs = GW_RUNS;
    for r = 1:numel(runs)
        dirR = fullfile(testsDir, 'gw parking lot', runs{r});
        S = readSensor(fullfile(dirR, 'sensor_log1.txt'));
        V = readVanet(fullfile(dirR, 'vanet_log1.txt'));
        ok = ~isnan(S.t);

        % Detections: beacons that carried the code (TX time), or the sensor log's message column without a vanet log
        tx = V(V.event == "TX", :);
        if ~isempty(tx)
            tAcc = tx.t(ismember(tx.msg, ACCIDENT_CODES));
            tTer = tx.t(ismember(tx.msg, TERRAIN_CODES));
        else
            onset = [true; diff(S.msg) ~= 0];
            tAcc = S.t(onset & ismember(S.msg, ACCIDENT_CODES));
            tTer = S.t(onset & ismember(S.msg, TERRAIN_CODES));
        end
        [bumps, bumpLabels] = marks(GW_BUMPS);

        name = sprintf('GrandWest Casino parking lot %s - acceleration', runs{r});
        fig = newFigure(name, [100 100 1200 500]);
        hold on; grid on;
        h = gobjects(0);
        h(end + 1) = plot(seconds(S.t(ok)), S.ax(ok), 'Color', C_ACC(1, :), 'LineWidth', 1);
        h(end + 1) = plot(seconds(S.t(ok)), S.ay(ok), 'Color', C_ACC(2, :), 'LineWidth', 1);
        h(end + 1) = plot(seconds(S.t(ok)), S.az(ok), 'Color', C_ACC(3, :), 'LineWidth', 1);
        labels = {'a_x', 'a_y', 'a_z'};
        [h, labels] = addLines(h, labels, tAcc, 'r', '--', 'Accident detected');
        [h, labels] = addLines(h, labels, tTer, C_ORANGE, '--', 'Terrain detected');
        [h, labels] = addLines(h, labels, bumps, 'k', '-', 'Bump (recorded)', bumpLabels);
        xtickformat('hh:mm:ss');
        xlabel('Time (SAST)');
        ylabel('Acceleration (g)');
        title(name);
        legend(h, labels, 'Location', 'eastoutside');
        saveFigure(fig, plotDir, name);

        fprintf('%s: %s to %s, %d accident and %d terrain detections, %d bumps marked\n', runs{r}, ...
            hmsStr(min(S.t)), hmsStr(max(S.t)), numel(tAcc), numel(tTer), numel(bumps));
        for i = 1:numel(tTer)
            fprintf('  terrain detected (beacon) at %s\n', hmsStr(tTer(i)));
        end
        for i = 1:numel(tAcc)
            fprintf('  accident detected (beacon) at %s\n', hmsStr(tAcc(i)));
        end
    end
catch err
    report(err);
end

%% heldersig straight -----------------------------------------------------------------------------------------------
section('heldersig straight');
try
    runs = subfolders(fullfile(testsDir, 'heldersig straight'), '^\d+kmph');
    trueSpd = cellfun(@(s) str2double(regexp(s, '^\d+', 'match', 'once')), runs);
    [trueSpd, order] = sort(trueSpd);
    runs = runs(order);

    fprintf('%-14s %10s %12s %14s %16s %8s\n', 'Run', 'True', 'Mean', 'Variance', 'RMS vs true', 'Samples');
    fprintf('%-14s %10s %12s %14s %16s %8s\n', '', '(km/h)', '(km/h)', '(km/h)^2', '(km/h)', '');
    spd = cell(size(runs));
    for r = 1:numel(runs)
        S = readSensor(fullfile(testsDir, 'heldersig straight', runs{r}, 'sensor_log1.txt'));
        spd{r} = S.speed;
        fprintf('%-14s %10d %12.2f %14.2f %16.2f %8d\n', runs{r}, trueSpd(r), mean(spd{r}), var(spd{r}), ...
            sqrt(mean((spd{r} - trueSpd(r)) .^ 2)), numel(spd{r}));
    end

    fprintf('\nOverall per true speed (all samples of its runs):\n');
    for v = unique(trueSpd)
        pool = vertcat(spd{trueSpd == v});
        fprintf('  %3d km/h: mean %.2f km/h, variance %.2f (km/h)^2, RMS vs true %.2f km/h, %d runs, %d samples\n', ...
            v, mean(pool), var(pool), sqrt(mean((pool - v) .^ 2)), sum(trueSpd == v), numel(pool));
    end
catch err
    report(err);
end

%% howard centre ----------------------------------------------------------------------------------------------------
section('howard centre');
try
    dirH = fullfile(testsDir, 'howard centre');

    % Full logs: each lap's packets are matched against them, so a packet sent at the end of a lap is still found
    full = loadNodes(dirH, [ALICE BOB]);

    envs = {'undercover', 'outside'};
    trueLoc = {HOWARD_TRUE_UNDERCOVER, HOWARD_TRUE_OUTSIDE};
    envMarks = {HOWARD_UNDERCOVER_MARKS, HOWARD_OUTSIDE_MARKS};
    for e = 1:numel(envs)
        laps = subfolders(dirH, ['^' envs{e} ' lap \d+$']);
        fprintf('\n%s (%s):\n', upper(envs{e}(1)) + string(envs{e}(2:end)), strjoin(laps, ', '));

        % RSSI, SNR and delivery over the laps joined together
        linkFigure(concatNodes(dirH, laps, [ALICE BOB]), sprintf('Howard centre - Bob %s - RSSI, SNR and packet delivery', ...
            envs{e}), envMarks{e}, ALICE, BOB, NAMES, C_AB, C_BA, plotDir);

        % Bob's GPS location across the laps
        S = cellfun(@(l) readSensor(fullfile(dirH, l, 'sensor_log2.txt')), laps, 'UniformOutput', false);
        S = vertcat(S{:});
        locationStats(S, trueLoc{e}, 'Bob');

        % Latency and time on air of the packets received directly, from the PPS time base
        pairs = [];
        for l = 1:numel(laps)
            lap = loadNodes(fullfile(dirH, laps{l}), [ALICE BOB]);
            for d = [ALICE BOB; BOB ALICE]
                L = link(lap.(nodeKey(d(1))).V, full.(nodeKey(d(2))).V, d(1), [], true);
                pairs = [pairs; L(L.delivered & L.rxTtl == TTL_INIT, :)]; %#ok<AGROW>
            end
        end
        latencyStats(pairs);
    end
catch err
    report(err);
end

%% menzies staircase ------------------------------------------------------------------------------------------------
section('menzies staircase');
try
    dirM = fullfile(testsDir, 'menzies staircase');
    lvls = subfolders(dirM, '^lvl\d+\+?$');
    key = cellfun(@(s) str2double(regexp(s, '\d+', 'match', 'once')) + 0.5 * endsWith(s, '+'), lvls);
    [~, order] = sort(key);
    lvls = lvls(order);

    % Levels climbed from START_LEVEL: lvl7 -> 5, a "+" folder is one more with an asterisk (lvl7+ -> 6*)
    climbed = cellfun(@(s) sprintf('%d%s', str2double(regexp(s, '\d+', 'match', 'once')) - START_LEVEL + endsWith(s, '+'), ...
        repmat('*', 1, endsWith(s, '+'))), lvls, 'UniformOutput', false);

    % PDR from the sender's seq numbers: column 1 Alice's beacons at Bob, column 2 Bob's beacons at Alice
    full = loadNodes(dirM, [ALICE BOB]);
    pdr = nan(numel(lvls), 2);
    rssi = pdr;
    snr = pdr;
    dirs = [ALICE BOB; BOB ALICE];
    for k = 1:2
        fprintf('%s -> %s (from %s''s seq numbers):\n', NAMES(dirs(k, 1)), NAMES(dirs(k, 2)), NAMES(dirs(k, 1)));
        fprintf('  %-8s %-8s %10s %8s %10s %12s %10s\n', 'Climbed', 'Folder', 'Received', 'Sent', 'PDR (%)', ...
            'RSSI (dBm)', 'SNR (dB)');
        for i = 1:numel(lvls)
            lvlLogs = loadNodes(fullfile(dirM, lvls{i}), dirs(k, 1));
            L = link(lvlLogs.(nodeKey(dirs(k, 1))).V, full.(nodeKey(dirs(k, 2))).V, dirs(k, 1), [], true);
            pdr(i, k) = 100 * mean(L.delivered);
            rssi(i, k) = mean(L.rssi(L.delivered));
            snr(i, k) = mean(L.snr(L.delivered));
            fprintf('  %-8s %-8s %10d %8d %10.1f %12.1f %10.2f\n', climbed{i}, lvls{i}, sum(L.delivered), height(L), pdr(i, k), ...
                rssi(i, k), snr(i, k));
        end
    end

    name = 'Menzies staircase - PDR, RSSI and SNR per level';
    fig = newFigure(name, [100 100 1000 900]);
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact');
    cols = [C_AB; C_BA];
    nexttile; barLabelled(climbed, pdr, cols, '%.0f'); ylabel('PDR (%)'); ylim([0 115]); grid on;
    legend('Alice -> Bob', 'Bob -> Alice', 'Location', 'northoutside', ...
        'Orientation', 'horizontal', 'Interpreter', 'none');
    nexttile; barLabelled(climbed, rssi, cols, '%.1f'); ylabel('Average RSSI (dBm)'); grid on;
    nexttile; barLabelled(climbed, snr, cols, '%.1f'); ylabel('Average SNR (dB)'); grid on;
    xlabel(tl, sprintf('Levels climbed from level %d (no bar = nothing received)', START_LEVEL));
    title(tl, name);
    saveFigure(fig, plotDir, name);
catch err
    report(err);
end

%% shd test ---------------------------------------------------------------------------------------------------------
section('shd test');
try
    shdAnalysis(fullfile(testsDir, 'shd test'), [ALICE BOB], NAMES, TTL_INIT);
catch err
    report(err);
end

%% tukzie test ------------------------------------------------------------------------------------------------------
section('tukzie test');
try
    [Lab, Lba] = linkFigure(loadNodes(fullfile(testsDir, 'tukzie test'), [ALICE BOB]), ...
        'Tukzie test - RSSI, SNR and packet delivery', TUKZIE_MARKS, ALICE, BOB, NAMES, C_AB, C_BA, plotDir);

    % PDR without the packets sent between TUKZIE_EXCLUDE(1) and TUKZIE_EXCLUDE(2)
    ex = toSec(TUKZIE_EXCLUDE);
    fprintf('\nExcluding packets sent from %s to %s:\n', TUKZIE_EXCLUDE{1}, TUKZIE_EXCLUDE{2});
    kept = {Lab(~(Lab.t >= ex(1) & Lab.t < ex(2)), :), Lba(~(Lba.t >= ex(1) & Lba.t < ex(2)), :)};
    labels = {sprintf('%s -> %s', NAMES(ALICE), NAMES(BOB)), sprintf('%s -> %s', NAMES(BOB), NAMES(ALICE))};
    for k = 1:2
        fprintf('%s: %d of %d received, PDR %.1f %%\n', labels{k}, sum(kept{k}.delivered), height(kept{k}), ...
            100 * mean(kept{k}.delivered));
    end
    both = [kept{1}.delivered; kept{2}.delivered];
    fprintf('Both directions: %d of %d received, PDR %.1f %%\n', sum(both), numel(both), 100 * mean(both));
catch err
    report(err);
end

%% baseline ---------------------------------------------------------------------------------------------------------
section('baseline');
try
    linkFigure(loadNodes(fullfile(testsDir, 'baseline'), [ALICE BOB]), 'Baseline - RSSI, SNR and packet delivery', ...
        BASELINE_MARKS, ALICE, BOB, NAMES, C_AB, C_BA, plotDir);
catch err
    report(err);
end

%% uct rugby field --------------------------------------------------------------------------------------------------
section('uct rugby field');
try
    dirU = fullfile(testsDir, 'uct rugby field');
    runs = subfolders(dirU, '^run \d+$');
    full = loadNodes(dirU, [ALICE BOB]);

    merged = [];
    fprintf('%-8s %12s %12s %12s %16s %18s\n', 'Run', 'PDR A->B', 'PDR B->A', 'PDR both', 'Mean distance', 'Max received at');
    for r = 1:numel(runs)
        runLogs = loadNodes(fullfile(dirU, runs{r}), [ALICE BOB]);
        pk = [];
        for d = [ALICE BOB; BOB ALICE]
            L = link(runLogs.(nodeKey(d(1))).V, full.(nodeKey(d(2))).V, d(1), [], true);
            L.dist = distanceTo(L, full.(nodeKey(d(2))).S, MAX_POS_AGE_S);
            L.dir = repmat(d(1), height(L), 1);
            pk = [pk; L]; %#ok<AGROW>
        end
        pk.run = repmat(r, height(pk), 1);
        merged = [merged; pk]; %#ok<AGROW>
        fprintf('%-8s %11.1f%% %11.1f%% %11.1f%% %14.1f m %16.1f m\n', runs{r}, ...
            100 * mean(pk.delivered(pk.dir == ALICE)), 100 * mean(pk.delivered(pk.dir == BOB)), 100 * mean(pk.delivered), ...
            mean(pk.dist, 'omitnan'), max(pk.dist(pk.delivered), [], 'omitnan'));
    end
    fprintf('%-8s %11.1f%% %11.1f%% %11.1f%% %14.1f m %16.1f m\n', 'Overall', ...
        100 * mean(merged.delivered(merged.dir == ALICE)), 100 * mean(merged.delivered(merged.dir == BOB)), 100 * mean(merged.delivered), ...
        mean(merged.dist, 'omitnan'), max(merged.dist(merged.delivered), [], 'omitnan'));
    fprintf(['Mean distance: average Alice-Bob separation when each packet was sent (range distance).\n' ...
             'Max received at: furthest separation at which a packet was still received.\n']);
    fprintf('Packets without a distance (no GPS fix on either node): %d of %d\n', sum(isnan(merged.dist)), height(merged));

    % PDR in bins of PDR_BIN packets, sorted by distance
    k = merged(~isnan(merged.dist), :);
    [~, order] = sort(k.dist);
    k = k(order, :);
    nb = floor(height(k) / PDR_BIN);
    binDist = zeros(nb, 1);
    binPdr = zeros(nb, 1);
    for b = 1:nb
        idx = (b - 1) * PDR_BIN + (1:PDR_BIN);
        binDist(b) = mean(k.dist(idx));
        binPdr(b) = 100 * mean(k.delivered(idx));
    end
    fprintf('\nPDR bins: %d bins of %d packets (%d left over)\n', nb, PDR_BIN, height(k) - nb * PDR_BIN);

    rx = k(k.delivered, :);
    scatterFit(binDist, binPdr, [], 'UCT rugby field - PDR vs distance', sprintf('PDR (%%, %d-packet bins)', PDR_BIN), ...
        C_AB, plotDir, 'best');
    scatterFit(rx.dist, rx.rssi, rx.dir == ALICE, 'UCT rugby field - RSSI vs distance', 'RSSI (dBm)', [C_AB; C_BA], ...
        plotDir, 'exp');
    scatterFit(rx.dist, rx.snr, rx.dir == ALICE, 'UCT rugby field - SNR vs distance', 'SNR (dB)', [C_AB; C_BA], ...
        plotDir, 'exp');
catch err
    report(err);
end

fprintf('\nPlots saved to %s\n', plotDir);
diary('off');

%% -----------------------------------------------------------------------------------------------------------------
%% Local functions
%% -----------------------------------------------------------------------------------------------------------------

function section(name)
    fprintf('\n==================== %s ====================\n', upper(name));
end

function report(err)
% Prints a failed section's error and carries on with the next one
    fprintf('ERROR: %s\n', err.message);
    for i = 1:min(3, numel(err.stack))
        fprintf('  at %s line %d\n', err.stack(i).name, err.stack(i).line);
    end
end

function names = subfolders(dirP, pattern)
% Subfolder names matching the regular expression, in name order
    d = dir(dirP);
    d = d([d.isdir] & ~startsWith({d.name}, '.'));
    names = {d.name};
    names = sort(names(~cellfun(@isempty, regexp(names, pattern, 'once'))));
    if isempty(names)
        error('test_analysis:noFolders', 'No folders matching "%s" in %s', pattern, dirP);
    end
end

function x = pick(c, i)
    if i <= numel(c)
        x = c{i};
    else
        x = {};
    end
end

% ---- Reading logs --------------------------------------------------------------------------------------------------

function t = tod(hhmmsscc)
% HHMMSSCC to seconds since midnight, NaN for no time (02000000 is the GPS default with no time)
    t = floor(hhmmsscc / 1e6) * 3600 + mod(floor(hhmmsscc / 1e4), 100) * 60 + mod(hhmmsscc, 1e4) / 100;
    t(hhmmsscc == 2000000 | isnan(hhmmsscc)) = NaN;
end

function s = toSec(times)
% 'HH:MM[:SS]' strings to seconds since midnight
    s = zeros(numel(times), 1);
    for i = 1:numel(times)
        p = str2double(split(string(times{i}), ':'));
        p(end + 1:3) = 0;
        s(i) = p(1) * 3600 + p(2) * 60 + p(3);
    end
end

function [t, labels] = marks(rows)
% {time, label} rows to seconds since midnight and labels
    if isempty(rows)
        t = zeros(0, 1);
        labels = {};
    else
        t = toSec(rows(:, 1));
        labels = rows(:, 2);
    end
end

function str = hmsStr(t)
    if isnan(t)
        str = '--:--:--';
    else
        str = char(duration(0, 0, t, 'Format', 'hh:mm:ss'));
    end
end

function S = readSensor(file)
% Sensor log as a table, t = SAST seconds since midnight
    names = {'ms', 'date', 'time', 'lat', 'lon', 'alt', 'speed', 'course', 'sats', 'hdop', ...
             'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'msg'};
    S = array2table(readNumeric(file, numel(names)), 'VariableNames', names);
    S.t = tod(S.time);
    S.lat(S.lat == 0 & S.lon == 0) = NaN;
    S.lon(isnan(S.lat)) = NaN;
end

function M = readMyself(file)
% Myself log as a table, t = SAST seconds since midnight
    names = {'ms', 'id', 'date', 'time', 'lat', 'lon', 'alt', 'speed', 'course', 'status', 'tasks', 'battery', ...
             'msg', 'seq', 'srcRssi', 'pktRssi'};
    M = array2table(readNumeric(file, numel(names)), 'VariableNames', names);
    M.t = tod(M.time);
end

function M = readNumeric(file, n)
    if ~isfile(file)
        error('test_analysis:missingLog', 'Log file not found: %s', file);
    end
    fid = fopen(file);
    c = textscan(fid, repmat('%f', 1, n), 'Delimiter', ',', 'CollectOutput', true);
    fclose(fid);
    M = c{1};
end

function V = readVanet(file)
% VANET log as a table, t = packet time (SAST s): TX start (TX/FW) or RxDone (RX) from utc when synced, else the line's time
    if ~isfile(file)
        error('test_analysis:missingLog', 'Log file not found: %s', file);
    end
    fid = fopen(file);
    c = textscan(fid, ['%f%s' repmat('%f', 1, 21)], 'Delimiter', ',');
    fclose(fid);
    names = {'ms', 'event', 'src', 'date', 'time', 'lat', 'lon', 'alt', 'speed', 'course', 'status', 'tasks', ...
             'battery', 'msg', 'seq', 'srcRssi', 'pktRssi', 'snr', 'freqErr', 'utc', 'air', 'sync', 'ttl'};
    n = min(cellfun(@numel, c));            % a partial last line is dropped
    c = cellfun(@(x) x(1:n), c, 'UniformOutput', false);
    V = table(c{:}, 'VariableNames', names);
    V.event = string(V.event);
    V.t = tod(V.time);
    synced = V.sync > 0 & V.utc > 0;
    V.t(synced) = mod(V.utc(synced) / 1e6 + 2 * 3600, 86400);
    V.lat(V.lat == 0 & V.lon == 0) = NaN;
    V.lon(isnan(V.lat)) = NaN;
end

function k = nodeKey(id)
    k = sprintf('n%d', id);
end

function N = loadNodes(dirP, ids)
% Sensor and VANET logs of each node in a folder (node id = log number)
    for id = ids
        N.(nodeKey(id)).S = readSensor(fullfile(dirP, sprintf('sensor_log%d.txt', id)));
        N.(nodeKey(id)).V = readVanet(fullfile(dirP, sprintf('vanet_log%d.txt', id)));
    end
end

% ---- Packets -------------------------------------------------------------------------------------------------------

function L = link(Vs, Vr, sid, span, nearTx)
% Beacons sent by node sid (its TX lines) and whether the receiver logged them (its RX lines, matched on seq)
%   span:   [t0 t1] keeps beacons sent while the receiver was logging ([] = all)
%   nearTx: only matches RX lines within a minute of the beacons, so seq numbers repeated after a reboot on another
%           day of the same log are not matched

    tx = Vs(Vs.event == "TX" & Vs.src == sid, :);
    [~, first] = unique(tx.seq, 'stable');
    tx = tx(first, :);
    if ~isempty(span)
        tx = tx(~(tx.t < span(1) | tx.t > span(2)), :);
    end

    rx = Vr(Vr.event == "RX" & Vr.src == sid, :);
    if nearTx && ~isempty(tx)
        rx = rx(rx.t >= min(tx.t) - 5 & rx.t <= max(tx.t) + 60, :);
    end
    [~, first] = unique(rx.seq, 'stable');
    rx = rx(first, :);

    [delivered, loc] = ismember(tx.seq, rx.seq);
    n = height(tx);
    L = table(tx.seq, tx.t, delivered, nan(n, 1), nan(n, 1), nan(n, 1), tx.utc, tx.air, tx.sync, nan(n, 1), ...
              nan(n, 1), tx.lat, tx.lon, ...
              'VariableNames', {'seq', 't', 'delivered', 'rssi', 'snr', 'rxUtc', 'txUtc', 'air', 'txSync', 'rxSync', ...
                                'rxTtl', 'lat', 'lon'});
    L.rssi(delivered) = rx.pktRssi(loc(delivered));
    L.snr(delivered) = rx.snr(loc(delivered));
    L.rxUtc(delivered) = rx.utc(loc(delivered));
    L.rxSync(delivered) = rx.sync(loc(delivered));
    L.rxTtl(delivered) = rx.ttl(loc(delivered));
end

function span = logSpan(S)
    span = [min(S.t), max(S.t)];
end

function N = concatNodes(dirP, subs, ids)
% Sensor and VANET logs of each node, the subfolders' logs joined in order
    parts = cellfun(@(s) loadNodes(fullfile(dirP, s), ids), subs);
    for id = ids
        k = nodeKey(id);
        S = arrayfun(@(p) p.(k).S, parts, 'UniformOutput', false);
        V = arrayfun(@(p) p.(k).V, parts, 'UniformOutput', false);
        N.(k).S = vertcat(S{:});
        N.(k).V = vertcat(V{:});
    end
end

function [Lab, Lba] = linkFigure(N, name, markRows, a, b, names, cAB, cBA, plotDir)
% RSSI, SNR and received/dropped of both directions against time, with labelled black lines at the marks
% N: logs of nodes a and b (loadNodes or concatNodes) | markRows: {time, label} per line

    [markT, markLabels] = marks(markRows);
    Lab = link(N.(nodeKey(a)).V, N.(nodeKey(b)).V, a, logSpan(N.(nodeKey(b)).S), false);
    Lba = link(N.(nodeKey(b)).V, N.(nodeKey(a)).V, b, logSpan(N.(nodeKey(a)).S), false);
    lab = sprintf('%s -> %s', names(a), names(b));
    lba = sprintf('%s -> %s', names(b), names(a));

    fig = newFigure(name, [100 100 1200 1050]);
    tl = tiledlayout(4, 1, 'TileSpacing', 'compact');
    ax = gobjects(4, 1);

    % Received packets only (a trailing NaN keeps a direction with none in the legend)
    got = @(L, v) deal([seconds(L.t(L.delivered)); seconds(NaN)], [L.(v)(L.delivered); NaN]);

    ax(1) = nexttile; hold on; grid on;
    [x, y] = got(Lab, 'rssi'); plot(x, y, '.-', 'Color', cAB);
    [x, y] = got(Lba, 'rssi'); plot(x, y, '.-', 'Color', cBA);
    ylabel('RSSI (dBm)');
    legend(sprintf('%s (%d received)', lab, sum(Lab.delivered)), sprintf('%s (%d received)', lba, sum(Lba.delivered)), ...
        'Location', 'best', 'Interpreter', 'none', 'AutoUpdate', 'off');

    ax(2) = nexttile; hold on; grid on;
    [x, y] = got(Lab, 'snr'); plot(x, y, '.-', 'Color', cAB);
    [x, y] = got(Lba, 'snr'); plot(x, y, '.-', 'Color', cBA);
    ylabel('SNR (dB)');

    % Received/dropped, one panel per direction so neither hides the other
    Ls = {Lab, Lba};
    cs = {cAB, cBA};
    ls = {lab, lba};
    for k = 1:2
        ax(2 + k) = nexttile; hold on; grid on;
        stem(seconds(Ls{k}.t), 2 * Ls{k}.delivered - 1, 'Color', cs{k}, 'MarkerFaceColor', cs{k}, 'MarkerSize', 3);
        ylim([-1.5 1.5]);
        yticks([-1 1]);
        yticklabels({'Dropped (-1)', 'Received (1)'});
        ylabel(ls{k}, 'Interpreter', 'none');
    end

    % Lines on every panel, labels on the top one only
    for i = 1:4
        for m = 1:numel(markT)
            hl = xline(ax(i), seconds(markT(m)), 'k', 'LineWidth', 1.2);
            if i == 1
                labelLine(hl, markLabels{m});
            end
        end
        xtickformat(ax(i), 'hh:mm:ss');
    end
    linkaxes(ax, 'x');
    xlabel(tl, 'Time sent (SAST)');
    title(tl, name);
    saveFigure(fig, plotDir, name);

    for L = {Lab, lab; Lba, lba}'
        d = L{1}.delivered;
        fprintf('%s: %d of %d received, PDR %.1f %%, average RSSI %.1f dBm, average SNR %.2f dB\n', L{2}, sum(d), ...
            numel(d), 100 * mean(d), mean(L{1}.rssi(d)), mean(L{1}.snr(d)));
    end
    fprintf('Both directions: %d of %d received, PDR %.1f %%\n', sum([Lab.delivered; Lba.delivered]), ...
        height(Lab) + height(Lba), 100 * mean([Lab.delivered; Lba.delivered]));
    fprintf('Packets without a time (not shown on the time axis): %d\n', sum(isnan([Lab.t; Lba.t])));
    fprintf('Black lines: %d\n', numel(markT));
end

% ---- Statistics ----------------------------------------------------------------------------------------------------

function [e, n] = toLocal(lat, lon, lat0, lon0)
% Degrees to metres east/north of (lat0, lon0) (equirectangular, fine over a few km)
    R = 6371000;
    e = deg2rad(lon - lon0) .* cosd(lat0) * R;
    n = deg2rad(lat - lat0) * R;
end

function locationStats(S, truth, who)
% Average position, its variance about the average and against the true location

    ok = ~isnan(S.lat);
    lat = S.lat(ok);
    lon = S.lon(ok);
    fprintf('  %s GPS: %d of %d samples with a position, satellites %d..%d\n', who, sum(ok), height(S), ...
        min(S.sats), max(S.sats));
    if isempty(lat)
        fprintf('  No position logged: no GPS\n');
        return;
    end
    if size(unique([lat lon], 'rows'), 1) == 1
        fprintf('  Position constant at (%.7f, %.7f) in every sample: interpreted as NO GPS (last fix held)\n', lat(1), lon(1));
        return;
    end

    lat0 = mean(lat);
    lon0 = mean(lon);
    [e, n] = toLocal(lat, lon, lat0, lon0);
    fprintf('  Average location: (%.7f, %.7f)\n', lat0, lon0);
    fprintf('  Variance about the average: east %.2f m^2, north %.2f m^2 (std %.2f m, %.2f m), RMS spread %.2f m\n', ...
        var(e), var(n), std(e), std(n), sqrt(mean(e .^ 2 + n .^ 2)));
    if any(isnan(truth))
        fprintf('  True location not supplied: no comparison\n');
        return;
    end
    [e, n] = toLocal(lat, lon, truth(1), truth(2));
    [eb, nb] = toLocal(lat0, lon0, truth(1), truth(2));
    fprintf('  Against the true location (%.7f, %.7f): average is %.2f m away, variance %.2f m^2 (RMS error %.2f m)\n', ...
        truth(1), truth(2), hypot(eb, nb), mean(e .^ 2 + n .^ 2), sqrt(mean(e .^ 2 + n .^ 2)));
end

function latencyStats(P)
% Latency (TX start to RxDone), time on air and their difference of packets timed by both nodes' PPS time base

    P = P(P.txSync > 0 & P.rxSync > 0 & P.txUtc > 0 & P.rxUtc > 0, :);
    if isempty(P)
        fprintf('  No packet timed by PPS on both nodes: no latency\n');
        return;
    end
    lat = mod(P.rxUtc - P.txUtc, 86400e6) / 1e3;     % ms (wraps at UTC midnight)
    air = P.air / 1e3;
    fprintf('  PPS-timed packets: %d (both locked %d, a node in holdover %d)\n', height(P), ...
        sum(P.txSync == 1 & P.rxSync == 1), sum(P.txSync == 2 | P.rxSync == 2));
    fprintf('  %-34s %10s %10s %10s\n', '', 'Min (ms)', 'Avg (ms)', 'Max (ms)');
    fprintf('  %-34s %10.3f %10.3f %10.3f\n', 'Latency (TX start to RxDone)', min(lat), mean(lat), max(lat));
    fprintf('  %-34s %10.3f %10.3f %10.3f\n', 'Time on air (TX start to TxDone)', min(air), mean(air), max(air));
    fprintf('  %-34s %10.3f %10.3f %10.3f\n', 'Latency minus time on air', min(lat - air), mean(lat - air), max(lat - air));
end

function d = distanceTo(L, Sr, maxAge)
% Distance (m) from the sender's position in each beacon to the receiver's nearest GPS fix in time

    d = nan(height(L), 1);
    ok = ~isnan(Sr.lat) & ~isnan(Sr.t) & Sr.sats > 0;
    [t, iu] = unique(Sr.t(ok));
    if numel(t) < 2
        return;
    end
    lat = Sr.lat(ok);
    lon = Sr.lon(ok);
    lat = lat(iu);
    lon = lon(iu);
    i = interp1(t, 1:numel(t), L.t, 'nearest', 'extrap');
    good = ~isnan(i) & ~isnan(L.lat);
    i(~good) = 1;
    age = abs(t(i) - L.t);
    good = good & age <= maxAge;
    [e, n] = toLocal(L.lat(good), L.lon(good), lat(i(good)), lon(i(good)));
    d(good) = hypot(e, n);
end

function [f, label, r2] = bestFit(x, y)
% Best of a straight line, a quadratic and an exponential (a e^(bx)) by adjusted R^2

    n = numel(x);
    sst = sum((y - mean(y)) .^ 2);
    adj = @(sse, k) 1 - (sse / (n - k)) / (sst / (n - 1));
    best = -Inf;

    for deg = 1:2
        if n > deg + 1
            p = polyfit(x, y, deg);
            g = @(xx) polyval(p, xx);
            a = adj(sum((y - g(x)) .^ 2), deg + 1);
            if a > best
                best = a; f = g; r2 = a;
                if deg == 1
                    label = sprintf('y = %.4g x + %.4g', p(1), p(2));
                else
                    label = sprintf('y = %.4g x^2 + %.4g x + %.4g', p(1), p(2), p(3));
                end
            end
        end
    end

    if n > 2 && (all(y > 0) || all(y < 0))
        s = sign(y(1));
        p0 = polyfit(x, log(abs(y)), 1);
        sse = @(q) sum((y - q(1) * exp(q(2) * x)) .^ 2);
        q = fminsearch(sse, [s * exp(p0(2)), p0(1)], optimset('Display', 'off', 'MaxFunEvals', 1e4, 'MaxIter', 1e4));
        a = adj(sse(q), 2);
        if a > best
            best = a; r2 = a;
            f = @(xx) q(1) * exp(q(2) * xx);
            label = sprintf('y = %.4g e^{%.4g x}', q(1), q(2));
        end
    end

    if isinf(best)
        f = []; label = 'no fit'; r2 = NaN;
    end
end

function [f, label, r2] = expFit(x, y)
% Exponential with an offset, y = a e^(bx) + c (levels off at c, e.g. the noise floor), by least squares

    n = numel(x);
    [xs, order] = sort(x);
    ys = y(order);
    m = max(1, round(n / 5));
    c0 = mean(ys(end - m + 1:end));                 % far end
    a0 = mean(ys(1:m)) - c0;                        % near end above it
    if a0 == 0
        a0 = 1;
    end
    b0 = -3 / max(range(xs), eps);                  % decays to ~5 % over the range
    sse = @(q) sum((y - (q(1) * exp(q(2) * x) + q(3))) .^ 2);
    q = fminsearch(sse, [a0 b0 c0], optimset('Display', 'off', 'MaxFunEvals', 2e4, 'MaxIter', 2e4));

    f = @(xx) q(1) * exp(q(2) * xx) + q(3);
    sst = sum((y - mean(y)) .^ 2);
    r2 = 1 - (sse(q) / (n - 3)) / (sst / (n - 1));
    label = sprintf('y = %.4g e^{%.4g x} + %.4g', q(1), q(2), q(3));
end

function scatterFit(x, y, groupA, name, yname, colours, plotDir, model)
% Scatter of y against distance (two colours by direction when groupA is given) with a fit and the correlation
% model: 'best' (line, quadratic or exponential, by adjusted R^2) or 'exp' (a e^(bx) + c)

    ok = ~isnan(x) & ~isnan(y);
    x = x(ok);
    y = y(ok);
    fig = newFigure(name, [100 100 900 600]);
    hold on; grid on;
    h = gobjects(0);
    labels = {};
    if isempty(groupA)
        h(end + 1) = scatter(x, y, 30, colours(1, :), 'filled');
        labels{end + 1} = 'Bins (both directions)';
    else
        groupA = groupA(ok);
        h(end + 1) = scatter(x(groupA), y(groupA), 18, colours(1, :), 'filled', 'MarkerFaceAlpha', 0.6);
        h(end + 1) = scatter(x(~groupA), y(~groupA), 18, colours(2, :), 'filled', 'MarkerFaceAlpha', 0.6);
        labels = [labels, {'Alice -> Bob', 'Bob -> Alice'}];
    end

    r = NaN;
    if numel(x) > 2
        c = corrcoef(x, y);
        r = c(1, 2);
        if strcmp(model, 'exp')
            [f, fl, r2] = expFit(x, y);
            kind = 'Exponential fit';
        else
            [f, fl, r2] = bestFit(x, y);
            kind = 'Best fit';
        end
        if ~isempty(f)
            xx = linspace(min(x), max(x), 200);
            h(end + 1) = plot(xx, f(xx), 'k', 'LineWidth', 2);
            labels{end + 1} = sprintf('%s: %s (adj. R^2 = %.3f)', kind, fl, r2);
        end
        fprintf('%s: n = %d, Pearson r = %.3f, %s %s, adjusted R^2 = %.3f\n', name, numel(x), r, lower(kind), fl, r2);
    end
    xlabel('Distance between Alice and Bob (m)');
    ylabel(yname);
    title(name);
    subtitle(sprintf('All runs merged, Pearson correlation r = %.3f', r));
    legend(h, labels, 'Location', 'best');
    saveFigure(fig, plotDir, name);
end

% ---- shd test ------------------------------------------------------------------------------------------------------

function shdAnalysis(dirP, ids, names, ttlInit)
% Forwards (anticipated from each beacon's destination vs logged), host message (myself log's message field) and
% peer list (serial monitor) vs VANET RX
% A beacon's destination is not logged: its status, tasks and battery identify the host packet behind it, and the
% serial monitor (the simulated hosts) gives that packet's destination

    HOST_MSG_WINDOW_S = 3;      % s after the host sends a code for the myself log to show it
    HOST_BEACON_WINDOW_S = 23;  % s for the next beacon to carry it (beacon interval 15 s + back-off up to 6 s + airtime)
    HOST_SETTLE_S = 1.5;        % s after a host packet change before the myself log is compared with it

    N = loadNodes(dirP, ids);
    for id = ids
        N.(nodeKey(id)).M = readMyself(fullfile(dirP, sprintf('myself_log%d.txt', id)));
    end
    H = readHostMonitor(fullfile(dirP, 'serial_monitor.txt'));
    statusNames = ["Parked", "Charging", "Delivering", "Taxi"];

    % Host (A/B) of each node, from its own bursts ("RX ME <id>")
    me = H(H.dir == "RX" & H.role == "ME", :);
    hostOf = containers.Map('KeyType', 'double', 'ValueType', 'char');
    for id = ids
        h = unique(me.host(me.id == id));
        if isscalar(h)
            hostOf(id) = char(h);
        end
    end
    tx = H(H.dir == "TX", :);
    fprintf('Serial monitor: %d host packets shown, hosts: %s\n', height(tx), ...
        strjoin(cellfun(@(id) sprintf('%s = %s', hostOf(id), names(id)), num2cell(cell2mat(keys(hostOf))), 'UniformOutput', false), ', '));

    % Host packet content -> destination, per host
    destOf = @(h, st, tk, bt) lookupDest(tx(tx.host == h, :), statusNames(st + 1), tk, bt);

    % ---- Forwards
    fprintf('\nForwards (a beacon for a node other than the receiver and not a broadcast is forwarded by it):\n');
    for s = ids
        for r = ids(ids ~= s)
            if ~isKey(hostOf, s)
                continue;
            end
            Vs = N.(nodeKey(s)).V;
            b = Vs(Vs.event == "TX" & Vs.src == s, :);
            span = logSpan(N.(nodeKey(r)).S);
            b = b(b.t >= span(1) & b.t <= span(2), :);
            dest = arrayfun(@(i) destOf(hostOf(s), b.status(i), b.tasks(i), b.battery(i)), (1:height(b))');
            antic = b.seq(~isnan(dest) & dest ~= 0 & dest ~= r);
            Vr = N.(nodeKey(r)).V;
            fw = unique(Vr.seq(Vr.event == "FW" & Vr.src == s));
            fprintf('  %s''s beacons forwarded by %s: anticipated %d, actual %d (%d of the anticipated, %d unexpected)\n', ...
                names(s), names(r), numel(antic), numel(fw), sum(ismember(antic, fw)), sum(~ismember(fw, antic)));
            fprintf('    anticipated seq: %s\n', mat2str(antic'));
            fprintf('    forwarded seq:   %s\n', mat2str(fw'));
            if any(isnan(dest))
                fprintf('    %d beacons with content not sent by the host (destination unknown)\n', sum(isnan(dest)));
            end
            fwTtl = Vr.ttl(Vr.event == "FW" & Vr.src == s);
            fprintf('    forwards sent with ttl %s (direct beacons go out with %d)\n', mat2str(unique(fwTtl)'), ttlInit);
        end
    end

    % The serial monitor's clock is the PC's: aligned to each node's by the offset at which the status, tasks and battery
    % in its myself log best follow its host's packets (searched over +-60 s)
    off = containers.Map('KeyType', 'double', 'ValueType', 'double');
    for id = cell2mat(keys(hostOf))
        M = N.(nodeKey(id)).M;
        txh = tx(tx.host == hostOf(id), :);
        cands = -60:0.1:60;
        score = arrayfun(@(o) mean(hostMatch(M, txh, o, statusNames, HOST_SETTLE_S), 'omitnan'), cands);
        [~, best] = max(score);
        off(id) = cands(best);
    end

    % ---- Host message: the message field in the node's myself log should hold the host's code after the host sends it
    % (the node keeps it until its next beacon, ~15 s; myself is logged every second)
    % A beacon soon after the code can take it before the next myself line, so the code showing in the sensor log's message
    % column (the code in the node's last beacon) before the next beacon is due also counts
    fprintf(['\nHost message (message field of each node''s myself log within %d s of the host sending a code, or the ' ...
             'sensor log''s last-beacon code within %d s):\n'], HOST_MSG_WINDOW_S, HOST_BEACON_WINDOW_S);
    for id = cell2mat(keys(hostOf))
        h = hostOf(id);
        M = N.(nodeKey(id)).M;
        S = N.(nodeKey(id)).S;
        ev = tx(tx.host == h & tx.msg ~= 0, :);
        evNode = ev.t - off(id);
        inMyself = arrayfun(@(i) any(M.msg(M.t > evNode(i) & M.t <= evNode(i) + HOST_MSG_WINDOW_S) == ev.msg(i)), ...
            (1:height(ev))');
        inBeacon = arrayfun(@(i) any(S.msg(S.t > evNode(i) & S.t <= evNode(i) + HOST_BEACON_WINDOW_S) == ev.msg(i)), ...
            (1:height(ev))');
        updated = inMyself | inBeacon;
        fprintf('  %s (host %s, serial clock %+.2f s from the node''s): %d host messages sent (code %s), message field updated after %d (%.0f %%)\n', ...
            names(id), h, off(id), height(ev), mat2str(unique(ev.msg)'), sum(updated), 100 * mean(updated));
        fprintf('    shown in the myself log: %d, carried by a beacon (sensor log): %d\n', sum(inMyself), sum(inBeacon));
        codes = unique(M.msg)';
        fprintf('    message field values in %s''s myself log: %s\n', names(id), ...
            strjoin(arrayfun(@(c) sprintf('%d (x%d)', c, sum(M.msg == c)), codes, 'UniformOutput', false), ', '));

        % Status, tasks and battery of the same host packets, for comparison
        same = hostMatch(M, tx(tx.host == h, :), off(id), statusNames, HOST_SETTLE_S);
        fprintf('    status, tasks and battery matched the host''s packet in %d of %d myself lines (%.0f %%)\n', ...
            sum(same == 1), sum(~isnan(same)), 100 * mean(same, 'omitnan'));
    end

    % ---- Peer list: each peer packet a node reports to its host ("RX Pn") should be an RX line in its VANET log, and
    % each packet it heard directly while the monitor ran should be in its peer list
    fprintf('\nPeer list vs VANET RX (peer packets shown on the serial monitor against each node''s RX lines):\n');
    for id = cell2mat(keys(hostOf))
        h = hostOf(id);
        P = H(H.dir == "RX" & startsWith(H.role, "P") & H.host == h, :);
        [~, first] = unique(P(:, {'id', 'seq'}), 'rows', 'stable');
        P = P(first, :);
        V = N.(nodeKey(id)).V;
        rx = V(V.event == "RX" & V.src ~= id, :);

        [found, loc] = ismember([P.id P.seq], [rx.src rx.seq], 'rows');
        r = rx(loc(found), :);
        p = P(found, :);
        same = r.status == (arrayfun(@(s) find(statusNames == s), p.status) - 1) & r.tasks == p.tasks & ...
               r.battery == p.battery & r.msg == p.msg & r.pktRssi == p.rssi;
        fprintf('  %s (host %s): %d peer packets listed, %d have an RX line, %d match it in status, tasks, battery, message and RSSI\n', ...
            names(id), h, height(P), sum(found), sum(same));
        if any(~found)
            fprintf('    listed without an RX line: %s\n', strjoin(compose('node %d seq %d', [P.id(~found) P.seq(~found)]), ', '));
        end
        bad = find(~same);
        for i = bad(:)'
            fprintf('    node %d seq %d differs: list status %s tasks %d battery %d msg %d rssi %d, RX status %d tasks %d battery %d msg %d rssi %d\n', ...
                p.id(i), p.seq(i), p.status(i), p.tasks(i), p.battery(i), p.msg(i), p.rssi(i), ...
                r.status(i), r.tasks(i), r.battery(i), r.msg(i), r.pktRssi(i));
        end

        % RX lines while the monitor ran, heard directly (a forwarded copy does not refresh the peer list)
        win = [min(H.t), max(H.t)] - off(id);
        direct = rx(rx.t >= win(1) & rx.t <= win(2) & rx.ttl == ttlInit, :);
        missing = ~ismember([direct.src direct.seq], [P.id P.seq], 'rows');
        fprintf('    direct RX lines while the monitor ran: %d, missing from the peer list: %d\n', height(direct), sum(missing));
        if any(missing)
            fprintf('    missing: %s\n', strjoin(compose('node %d seq %d', [direct.src(missing) direct.seq(missing)]), ', '));
        end
    end
end

function same = hostMatch(M, txh, off, statusNames, settle)
% Per myself line: 1 if its status, tasks and battery are those of the host packet being sent at the time (serial
% clock = node clock + off), 0 if not, NaN before the first packet or within settle s of a packet change
    same = nan(height(M), 1);
    ts = M.t + off;
    k = discretize(ts, [txh.t; Inf]);          % index of the last host packet at or before each line
    ok = ~isnan(k);
    ok(ok) = ts(ok) - txh.t(k(ok)) > settle;
    k = k(ok);
    st = arrayfun(@(s) find(statusNames == s), txh.status(k)) - 1;
    same(ok) = M.status(ok) == st & M.tasks(ok) == txh.tasks(k) & M.battery(ok) == round(txh.battery(k));
end

function d = lookupDest(tx, status, tasks, battery)
% Destination of the host packet with this content (battery as the node rounds it), NaN if not found
    k = tx.status == status & tx.tasks == tasks & round(tx.battery) == battery;
    d = NaN;
    if any(k)
        d = mode(tx.dest(k));
    end
end

function H = readHostMonitor(file)
% Serial monitor of the simulated hosts: "HH:MM:SS.mmm > <A|B> TX <status> t= b=% m= to=" and
% "HH:MM:SS.mmm > <A|B> RX <ME|Pn> <id> <status> t= b=% m= seq= <rssi> dBm"
    lines = readlines(file);
    head = '^(\d+):(\d+):([\d.]+) > ([AB]) ';
    body = '(\w+)\s+t=(\d+)\s+b=([\d.]+)%\s+m=(\d+)\s+';
    txTok = regexp(lines, [head 'TX\s+' body 'to=(\d+)'], 'tokens', 'once');
    rxTok = regexp(lines, [head 'RX\s+(ME|P\d)\s+(\d+)\s+' body 'seq=(\d+)\s+(-?\d+) dBm'], 'tokens', 'once');

    txTok = vertcat(txTok{:});      % time(3), host, status, tasks, battery, msg, dest
    rxTok = vertcat(rxTok{:});      % time(3), host, role, id, status, tasks, battery, msg, seq, rssi
    t = @(k) str2double(k(:, 1)) * 3600 + str2double(k(:, 2)) * 60 + str2double(k(:, 3));
    nT = size(txTok, 1);
    nR = size(rxTok, 1);
    H = table([t(txTok); t(rxTok)], [txTok(:, 4); rxTok(:, 4)], [repmat("TX", nT, 1); repmat("RX", nR, 1)], ...
              [repmat("", nT, 1); rxTok(:, 5)], [nan(nT, 1); str2double(rxTok(:, 6))], [txTok(:, 5); rxTok(:, 7)], ...
              str2double([txTok(:, 6); rxTok(:, 8)]), str2double([txTok(:, 7); rxTok(:, 9)]), ...
              str2double([txTok(:, 8); rxTok(:, 10)]), [str2double(txTok(:, 9)); nan(nR, 1)], ...
              [nan(nT, 1); str2double(rxTok(:, 11))], [nan(nT, 1); str2double(rxTok(:, 12))], ...
              'VariableNames', {'t', 'host', 'dir', 'role', 'id', 'status', 'tasks', 'battery', 'msg', 'dest', 'seq', 'rssi'});
    H = sortrows(H, 't');
end

% ---- Figures -------------------------------------------------------------------------------------------------------

function fig = newFigure(name, pos)
    fig = figure('Name', name, 'Color', 'w', 'Position', pos);
end

function [h, labels] = addLines(h, labels, times, colour, style, label, texts)
% Vertical lines at the times, one legend entry for all of them, each labelled with texts{i} if given
    for i = 1:numel(times)
        hl = xline(seconds(times(i)), 'Color', colour, 'LineStyle', style, 'LineWidth', 1.2);
        if nargin > 6
            labelLine(hl, texts{i});
        end
        if i == 1
            h(end + 1) = hl; %#ok<AGROW>
            labels{end + 1} = label; %#ok<AGROW>
        end
    end
end

function labelLine(hl, text)
% Writes the text along a vertical line, at its top
    if strlength(text) > 0
        set(hl, 'Label', text, 'LabelOrientation', 'aligned', 'LabelVerticalAlignment', 'top', ...
            'LabelHorizontalAlignment', 'left', 'FontSize', 9);
    end
end

function barLabelled(labels, y, colours, fmt)
% Grouped bars (one group per label, one bar per column of y) with each value written on its bar
    b = bar(1:size(y, 1), y);
    for k = 1:numel(b)
        b(k).FaceColor = colours(k, :);
        % Labels beyond the bar's end: above if positive, below if negative
        align = {'top', 'bottom'};
        for up = [false true]
            ok = ~isnan(y(:, k)) & (y(:, k) >= 0) == up;
            text(b(k).XEndPoints(ok), b(k).YEndPoints(ok), compose(fmt, y(ok, k)), 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', align{up + 1}, 'FontSize', 8);
        end
    end
    ylim(ylim + [-0.08 0.08] * diff(ylim));          % room for the labels
    xticks(1:size(y, 1));
    xticklabels(labels);
end

function saveFigure(fig, plotDir, name)
% Saves a figure as plots/<name>.png (200 dpi), the name made safe for a file
    set(findall(fig, 'Type', 'axes'), 'Toolbar', []);
    file = regexprep(name, '[<>:"/\\|?*]', '-');
    exportgraphics(fig, fullfile(plotDir, [file '.png']), 'Resolution', 200);
end

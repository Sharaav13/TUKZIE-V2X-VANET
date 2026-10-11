%% TUKZIE V2X VANET (TVV) - RF PROFILING
% Link quality between 2 nodes from their VANET logs (log.txt on each node's SD Card)
%
% Input (in the "logs" folder next to this script):
%   log1.txt  log.txt from node 1
%   log2.txt  log.txt from node 2
%
% Log line (formatVanet in diagnostics.cpp), 19 fields:
%   ms,event,src,date,time,lat,lon,alt,speed,course,status,tasks,battery,message,seq,srcRssi,pktRssi,snr,freqErr
%   TX lines: src = this node, seq = beacon sent | RX lines: src = sender, seq = sender's beacon
%   srcRssi = sender's LoRa.rssi() just before it sent | pktRssi, snr, freqErr = measured by the receiver
%
% Figures (saved as .png and .fig in the "figures" folder):
%   1. PDR moving average per direction
%   2. RSSI: received packet RSSI (pktRssi) with the sender's channel RSSI at Tx (srcRssi)
%   3. SNR and its moving average
%   4. Frequency error and its moving average
%
% The x-axis is the sender's sequence number, since each node's ms clock starts at its own boot

clear; close all; clc;

%% Settings
WINDOW = 10;                % packets in each moving average
TRIM_TO_OVERLAP = true;     % only count beacons sent while the receiver was logging (between its first and last received seq)

scriptDir = fileparts(mfilename('fullpath'));
logDir = fullfile(scriptDir, 'logs');
figDir = fullfile(scriptDir, 'figures');
if ~isfolder(figDir)
    mkdir(figDir);
end

%% Read the logs
log1 = readVanetLog(fullfile(logDir, 'log1.txt'));
log2 = readVanetLog(fullfile(logDir, 'log2.txt'));

id1 = nodeId(log1, 'log1.txt');
id2 = nodeId(log2, 'log2.txt');

%% Packet delivery per direction
d12 = linkDirection(log1, log2, id1, id2, TRIM_TO_OVERLAP);   % node 1 -> node 2 (received by node 2)
d21 = linkDirection(log2, log1, id2, id1, TRIM_TO_OVERLAP);   % node 2 -> node 1 (received by node 1)

pdrOverall = (sum(d12.delivered) + sum(d21.delivered)) / (numel(d12.delivered) + numel(d21.delivered));

fprintf('Node %d -> node %d: %d of %d delivered, PDR = %.2f %%\n', id1, id2, sum(d12.delivered), numel(d12.delivered), 100 * mean(d12.delivered));
fprintf('Node %d -> node %d: %d of %d delivered, PDR = %.2f %%\n', id2, id1, sum(d21.delivered), numel(d21.delivered), 100 * mean(d21.delivered));
fprintf('Overall PDR = %.2f %%\n', 100 * pdrOverall);

label12 = sprintf('Node %d \\rightarrow %d', id1, id2);
label21 = sprintf('Node %d \\rightarrow %d', id2, id1);
c12 = [0.00 0.45 0.74];     % node 1 -> 2
c21 = [0.85 0.33 0.10];     % node 2 -> 1

%% Figure 1: PDR moving average
fig = figure('Name', 'PDR', 'Color', 'w');
hold on; grid on;
plot(d12.sentSeq, 100 * movmean(double(d12.delivered), WINDOW), 'Color', c12, 'LineWidth', 1.5);
plot(d21.sentSeq, 100 * movmean(double(d21.delivered), WINDOW), 'Color', c21, 'LineWidth', 1.5);
yline(100 * pdrOverall, '--k', sprintf('Overall %.1f %%', 100 * pdrOverall), 'LabelHorizontalAlignment', 'left');
ylim([0 105]);
xlabel('Sequence number (sender)');
ylabel('PDR (%)');
title(sprintf('Packet delivery ratio, %d-packet moving average (overall %.2f %%)', WINDOW, 100 * pdrOverall));
legend(sprintf('%s (%.1f %%)', label12, 100 * mean(d12.delivered)), ...
       sprintf('%s (%.1f %%)', label21, 100 * mean(d21.delivered)), 'Location', 'southwest');
saveFigure(fig, figDir, 'rf_pdr');

%% Figure 2: RSSI of received packets with the sender's channel RSSI at Tx
fig = figure('Name', 'RSSI', 'Color', 'w');
hold on; grid on;
plot(d12.rx.seq, d12.rx.pktRssi, '.-', 'Color', c12, 'LineWidth', 1);
plot(d12.rx.seq, d12.rx.srcRssi, ':', 'Color', c12, 'LineWidth', 1.5);
plot(d21.rx.seq, d21.rx.pktRssi, '.-', 'Color', c21, 'LineWidth', 1);
plot(d21.rx.seq, d21.rx.srcRssi, ':', 'Color', c21, 'LineWidth', 1.5);
xlabel('Sequence number (sender)');
ylabel('RSSI (dBm)');
title('RSSI: received packet (pktRssi) and sender''s channel at Tx (srcRssi)');
legend(sprintf('Packet RSSI at node %d (from node %d)', id2, id1), ...
       sprintf('Channel RSSI at node %d when sending', id1), ...
       sprintf('Packet RSSI at node %d (from node %d)', id1, id2), ...
       sprintf('Channel RSSI at node %d when sending', id2), 'Location', 'best');
saveFigure(fig, figDir, 'rf_rssi');

%% Figure 3: SNR and its moving average
fig = figure('Name', 'SNR', 'Color', 'w');
hold on; grid on;
plot(d12.rx.seq, d12.rx.snr, '.', 'Color', c12);
plot(d12.rx.seq, movmean(d12.rx.snr, WINDOW), 'Color', c12, 'LineWidth', 2);
plot(d21.rx.seq, d21.rx.snr, '.', 'Color', c21);
plot(d21.rx.seq, movmean(d21.rx.snr, WINDOW), 'Color', c21, 'LineWidth', 2);
yline(-7.5, '--k', 'SF7 limit (-7.5 dB)', 'LabelHorizontalAlignment', 'left');
xlabel('Sequence number (sender)');
ylabel('SNR (dB)');
title(sprintf('SNR and %d-packet moving average', WINDOW));
legend([label12 ' SNR'], [label12 ' moving average'], [label21 ' SNR'], [label21 ' moving average'], 'Location', 'best');
saveFigure(fig, figDir, 'rf_snr');

%% Figure 4: Frequency error and its moving average
fig = figure('Name', 'Frequency error', 'Color', 'w');
hold on; grid on;
plot(d12.rx.seq, d12.rx.freqErr, '.', 'Color', c12);
plot(d12.rx.seq, movmean(d12.rx.freqErr, WINDOW), 'Color', c12, 'LineWidth', 2);
plot(d21.rx.seq, d21.rx.freqErr, '.', 'Color', c21);
plot(d21.rx.seq, movmean(d21.rx.freqErr, WINDOW), 'Color', c21, 'LineWidth', 2);
xlabel('Sequence number (sender)');
ylabel('Frequency error (Hz)');
title(sprintf('Frequency error and %d-packet moving average', WINDOW));
legend([label12 ' error'], [label12 ' moving average'], [label21 ' error'], [label21 ' moving average'], 'Location', 'best');
saveFigure(fig, figDir, 'rf_freq_error');

fprintf('Figures saved to %s\n', figDir);

%% ------------------------------------------------------------------------------------------------
%% Local functions
%% ------------------------------------------------------------------------------------------------

function T = readVanetLog(file)
% Reads a VANET log into a table (lines without exactly 19 fields are skipped)

    if ~isfile(file)
        error('rf_profiling:missingLog', 'Log file not found: %s', file);
    end

    lines = strtrim(readlines(file));
    lines = lines(strlength(lines) > 0);
    lines = lines(count(lines, ",") == 18);     % 19 fields: skips partial or old-format lines
    if isempty(lines)
        error('rf_profiling:emptyLog', 'No valid log lines in %s (expected 19 fields per line)', file);
    end

    parts = split(lines, ",");
    if isscalar(lines)
        parts = parts.';                         % split returns a column for a single line
    end

    T = table(double(parts(:, 1)), parts(:, 2), double(parts(:, 3)), double(parts(:, 15)), ...
              double(parts(:, 16)), double(parts(:, 17)), double(parts(:, 18)), double(parts(:, 19)), ...
              'VariableNames', {'ms', 'event', 'src', 'seq', 'srcRssi', 'pktRssi', 'snr', 'freqErr'});
end

function id = nodeId(T, name)
% The node's own ID: the src of its TX lines

    tx = T.src(T.event == "TX");
    if isempty(tx)
        error('rf_profiling:noTx', 'No TX lines in %s, so its node ID is unknown', name);
    end
    id = mode(tx);
end

function d = linkDirection(senderLog, receiverLog, senderId, receiverId, trimToOverlap)
% Beacons sent by senderId (from its TX lines) and those received by receiverId (from its RX lines)

    sentSeq = unique(senderLog.seq(senderLog.event == "TX" & senderLog.src == senderId));

    rx = receiverLog(receiverLog.event == "RX" & receiverLog.src == senderId, :);
    [~, first] = unique(rx.seq);                % drop duplicates, keep the first copy
    rx = rx(sort(first), :);
    rx = sortrows(rx, 'seq');

    if isempty(rx)
        warning('rf_profiling:noRx', 'Node %d received nothing from node %d', receiverId, senderId);
    elseif trimToOverlap
        sentSeq = sentSeq(sentSeq >= min(rx.seq) & sentSeq <= max(rx.seq));
    end

    if isempty(sentSeq)
        error('rf_profiling:noSent', 'No beacons from node %d to compare against', senderId);
    end

    d.sentSeq = sentSeq;
    d.delivered = ismember(sentSeq, rx.seq);
    d.rx = rx(ismember(rx.seq, sentSeq), :);
end

function saveFigure(fig, figDir, name)
% Saves a figure as .png (200 dpi) and .fig

    set(findall(fig, 'Type', 'axes'), 'Toolbar', []);     % keep the axes toolbar out of the image
    exportgraphics(fig, fullfile(figDir, [name '.png']), 'Resolution', 200);
    savefig(fig, fullfile(figDir, [name '.fig']));
end

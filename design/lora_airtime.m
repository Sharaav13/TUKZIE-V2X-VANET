%% ========================================================================
%  LoRa Time-on-Air and Duty Cycle Calculator
%  TUKZIE V2X VANET (TVV) - Sharaav Dhebideen
%
%  Implements the time-on-air equations from:
%    Semtech SX1276/77/78/79 Datasheet Rev. 7, Sec. 4.1.1.5 - 4.1.1.7
%  and the duty cycle definition from:
%    ETSI EN 300 220-1 V3.1.1, Sec. 3.1 and 5.4.1 (Tobs = 1 hour)
%
%  Edit the PARAMETERS block, then run.
% =========================================================================

clear; clc;

%% ---------------- PARAMETERS ------------------------------------------

% --- Radio configuration ---
SF          = 12;        % Spreading factor, 6..12
BW          = 125e3;    % Bandwidth [Hz]: 7.8e3 10.4e3 15.6e3 20.8e3 31.2e3
                        %                 41.7e3 62.5e3 125e3 250e3 500e3
CR          = 1;        % Coding rate index: 1=4/5, 2=4/6, 3=4/7, 4=4/8
PL          = 25;       % Payload length [bytes], 1..255
N_PREAMBLE  = 8;        % Programmed preamble symbols (library default = 8)
CRC_ON      = true;     % true if LoRa.enableCrc() is called
IMPLICIT_HDR= false;    % false = explicit header (normal)
LDRO         = [];      % [] = auto (mandated when Ts > 16 ms); or force true/false

% --- Operational / regulatory ---
BEACON_MS   = 15000;     % Intended transmit interval [ms]
DC_LIMIT    = 0.10;     % Regulatory duty cycle limit (0.10 = 10% @ 433 MHz)
T_OBS       = 3600;     % Duty cycle observation period [s] (EN 300 220-1)

% --- Optional ---
DO_SWEEP    = false;    % true to also print an SF/CR sweep table

%% ---------------- VALIDATION ------------------------------------------

validBW = [7.8 10.4 15.6 20.8 31.2 41.7 62.5 125 250 500] * 1e3;
assert(SF >= 6  && SF <= 12,  'SF must be 6..12');
assert(CR >= 1  && CR <= 4,   'CR index must be 1..4 (1=4/5 .. 4=4/8)');
assert(PL >= 1  && PL <= 255, 'PL must be 1..255 bytes');
assert(any(abs(validBW - BW) < 1), 'BW is not an SX1278 bandwidth option');

%% ---------------- CALCULATION -----------------------------------------

[Tpacket, Tpre, Tpay, nPayload, Ts, DE] = ...
    loraToA(PL, SF, BW, CR, N_PREAMBLE, CRC_ON, IMPLICIT_HDR, LDRO);

Tinterval   = BEACON_MS / 1000;              % s
dutyCycle   = Tpacket / Tinterval;           % fraction
compliant   = dutyCycle <= DC_LIMIT;

minInterval = Tpacket / DC_LIMIT;            % s  - shortest legal interval
offTime     = Tpacket * (1/DC_LIMIT - 1);    % s  - silence after each TX
budget      = DC_LIMIT * T_OBS;              % s  - airtime allowed per hour
maxPackets  = floor(budget / Tpacket);       % packets per hour
usedPerHour = Tpacket * (T_OBS / Tinterval); % s  - airtime at chosen interval

%% ---------------- OUTPUT ----------------------------------------------

fprintf('\n=== LoRa Time on Air ====================================\n');
fprintf('  SF %d, BW %.1f kHz, CR 4/%d, PL %d bytes\n', SF, BW/1e3, CR+4, PL);
fprintf('  Preamble %d sym, CRC %s, %s header, LDRO %s\n', ...
        N_PREAMBLE, onoff(CRC_ON), hdr(IMPLICIT_HDR), onoff(DE));
fprintf('  ---------------------------------------------------\n');
fprintf('  Symbol duration   Ts         = %9.3f ms\n', Ts*1e3);
fprintf('  Preamble          Tpreamble  = %9.3f ms\n', Tpre*1e3);
fprintf('  Payload symbols   nPayload   = %9d\n',      nPayload);
fprintf('  Payload           Tpayload   = %9.3f ms\n', Tpay*1e3);
fprintf('  TIME ON AIR       Tpacket    = %9.3f ms\n', Tpacket*1e3);

fprintf('\n=== Duty Cycle ==========================================\n');
fprintf('  Beacon interval              = %9.3f s\n',  Tinterval);
fprintf('  DUTY CYCLE                   = %9.3f %%\n', dutyCycle*100);
fprintf('  Regulatory limit             = %9.3f %%\n', DC_LIMIT*100);
if compliant
    fprintf('  STATUS                       = COMPLIANT (%.1f%% margin)\n', ...
            (1 - dutyCycle/DC_LIMIT)*100);
else
    fprintf('  STATUS                       = *** NON-COMPLIANT ***\n');
    fprintf('  Interval must be at least    = %9.3f s\n', minInterval);
end

fprintf('\n=== Budget (Tobs = %d s) ===============================\n', T_OBS);
fprintf('  Airtime allowed per hour     = %9.1f s\n',  budget);
fprintf('  Airtime used at this interval= %9.1f s\n',  usedPerHour);
fprintf('  Max packets per hour         = %9d\n',      maxPackets);
fprintf('  Min compliant interval       = %9.3f s\n',  minInterval);
fprintf('  Required off-time after TX   = %9.3f s\n',  offTime);
fprintf('=========================================================\n\n');

%% ---------------- OPTIONAL SWEEP --------------------------------------

if DO_SWEEP
    fprintf('Sweep: PL = %d bytes, BW = %.1f kHz, CRC %s\n\n', ...
            PL, BW/1e3, onoff(CRC_ON));
    fprintf('   SF    CR      ToA (ms)   min interval (s)   pkts/hr\n');
    fprintf('  ------------------------------------------------------\n');
    for sf = 7:12
        for cr = [1 4]
            t = loraToA(PL, sf, BW, cr, N_PREAMBLE, CRC_ON, IMPLICIT_HDR, LDRO);
            fprintf('  %3d   4/%d   %10.1f   %16.2f   %7d\n', ...
                    sf, cr+4, t*1e3, t/DC_LIMIT, floor(budget/t));
        end
    end
    fprintf('\n');
end

%% ---------------- LOCAL FUNCTIONS -------------------------------------

function [Tpacket, Tpre, Tpay, nPayload, Ts, DE] = ...
         loraToA(PL, SF, BW, CR, nPre, crcOn, implicitHdr, ldroOverride)
% LORATOA  LoRa time on air.
%   Semtech SX1276 datasheet Rev. 7, Sec. 4.1.1.7.
%
%   Ts        = 2^SF / BW
%   Tpreamble = (nPre + 4.25) * Ts
%   nPayload  = 8 + max( ceil( (8PL - 4SF + 28 + 16CRC - 20IH)
%                              / (4(SF - 2DE)) ) * (CR + 4), 0 )
%   Tpacket   = Tpreamble + nPayload * Ts

    Ts = (2^SF) / BW;                       % symbol duration [s]

    % Low data rate optimisation: mandated when Ts > 16 ms (Sec. 4.1.1.6)
    if isempty(ldroOverride)
        DE = Ts > 16e-3;
    else
        DE = logical(ldroOverride);
    end

    CRC = double(crcOn);
    IH  = double(implicitHdr);

    Tpre = (nPre + 4.25) * Ts;

    % NOTE: denominator is 4*(SF - 2*DE), NOT 4*SF - 2*DE
    numer = 8*PL - 4*SF + 28 + 16*CRC - 20*IH;
    denom = 4 * (SF - 2*double(DE));

    nPayload = 8 + max( ceil(numer/denom) * (CR + 4), 0 );

    Tpay    = nPayload * Ts;
    Tpacket = Tpre + Tpay;
end

function s = onoff(tf)
    if tf, s = 'on'; else, s = 'off'; end
end

function s = hdr(implicitHdr)
    if implicitHdr, s = 'implicit'; else, s = 'explicit'; end
end
function connectionTestOne(boardLabel)
%% ============================================================
%  CONNECTION TEST -- one short charge pulse + one short discharge pulse
%  (about 1-2 minutes, net SOC change ~ 0). Checks whether the wiring /
%  Kelvin-sense / holder contacts of ONE board are healthy.
%
%  USAGE (MATLAB Command Window, after all formation / cycling timers
%  have been stopped):
%       connectionTest('A')
%       connectionTest('B')     % run B too, as a reference
%
%  HOW IT WORKS (no multimeter needed)
%   Both instruments are wired to the same cell, but only ONE carries
%   current at a time. The instrument that is NOT carrying current has
%   (almost) no voltage drop in its wires, so it is a "clean" voltmeter
%   at the cell:
%     * during the CHARGE pulse:    DL3021A reading = clean tap
%     * during the DISCHARGE pulse: DP813A  reading = clean tap
%   From that we get three independent numbers:
%     R_int   : step in the clean-tap voltage when current is switched
%               off, divided by current  = cell + holder-contact
%               resistance (shared by charge and discharge paths).
%     R_force_chg : (DP reading - DL reading) / I during charge
%               = voltage lost between the PSU sense point and the cell.
%     R_force_dis : (DP reading - DL reading) / I during discharge
%               = voltage lost between the load sense point and the cell.
%
%  REFERENCE VALUES / THRESHOLDS
%   R_int: 0.05-0.09 ohm in the Aug 24-27 formation data when the cell
%   was near full (first-sample load step). Cells at low SOC read a bit
%   higher. PASS <= 0.10, WARN <= 0.15, FAIL above.
%   R_force: should be ~0 with correct Kelvin sense (instrument accuracy
%   only). PASS <= 0.04, WARN <= 0.10, FAIL above.
%   These thresholds are practical rules of thumb, not datasheet values.
%
%  SAFETY
%   - connectionTestOne: runs while OTHER boards keep cycling; refuses only
%     if this board's own timer is running. Closes only this board's handles.
%   - Aborts and switches everything off if voltage/current leave a safe
%     range; Ctrl+C is safe too (outputs are switched off on exit).
%   - Order of the two pulses is chosen from the rest voltage so a full
%     cell is discharged first and an empty cell is charged first.
% ============================================================
    if nargin < 1
        error('Usage: connectionTest(''A'') / (''B'') / (''C'')');
    end
    boardLabel = upper(char(boardLabel));

    %% === CONFIG -- addresses come from rigConfig.m (boards A, B, C) ===
    c = rigConfig(boardLabel);

    %% === PARAMETERS ===
    C_Ah    = 1.3;
    I_chg   = 0.5*C_Ah;   % 0.65 A, same as formation charge
    I_dis   = 1.0*C_Ah;   % 1.30 A, same as formation discharge
    T_chg   = 20;         % s  -> ~3.6 mAh in
    T_dis   = 10;         % s  -> ~3.6 mAh out (net SOC change ~ 0)
    V_lim   = 4.0;        % PSU voltage limit during the charge pulse
    V_ovp   = 4.1;
    V_floor = 1.8;        % abort the discharge pulse below this voltage (was 2.0: aborted on low-SOC cells)

    %% === PRE-CHECKS (connectionTestOne: other boards may keep running) ===
    global boards t
    k = [];
    if ~isempty(boards) && isstruct(boards) && isfield(boards,'label')
        k = find(strcmp({boards.label}, boardLabel), 1);
    end
    if ~isempty(k) && iscell(t) && numel(t) >= k && ~isempty(t{k}) && isvalid(t{k}) && strcmp(t{k}.Running,'on')
        error('Board %s timer is STILL RUNNING -- this test only runs on a stopped board.', boardLabel);
    end
    % close ONLY this board's leftover VISA handles (other boards untouched)
    if ~isempty(k)
        try
            if ~isempty(boards(k).DP813A) && isvalid(boards(k).DP813A)
                try, writeline(boards(k).DP813A, ':OUTP:STAT OFF'); catch, end
                delete(boards(k).DP813A);
            end
        catch, end
        try
            if ~isempty(boards(k).DL3021) && isvalid(boards(k).DL3021)
                try, writeline(boards(k).DL3021, ':SOUR:INP:STAT OFF'); catch, end
                delete(boards(k).DL3021);
            end
        catch, end
        if iscell(t) && numel(t) >= k && ~isempty(t{k}), try, delete(t{k}); catch, end, t{k} = []; end
    end

    %% === CONNECT ===
    fprintf('>> [Board %s] Connecting...\n', boardLabel);
    dp = visadev(c.visaDP); try, fopen(dp); catch, end %#ok<TRYNC>
    setupPSURange(dp, c);
    dl = visadev(c.visaDL); try, fopen(dl); catch, end %#ok<TRYNC>
    guard = onCleanup(@() safeOff(dp, dl)); %#ok<NASGU>

    writeline(dp, ':OUTP:STAT OFF');
    writeline(dp, sprintf(':SOUR:VOLT:PROT:LEV %g', V_ovp));
    writeline(dp, ':SOUR:VOLT:PROT:STAT ON');
    writeline(dp, sprintf(':SOUR:CURR:PROT:LEV %g', 1.3*I_chg));
    writeline(dp, ':SOUR:CURR:PROT:STAT ON');

    writeline(dl, ':SOUR:INP:STAT OFF'); pause(0.1);
    writeline(dl, ':SOUR:FUNC:MODE FIX'); pause(0.1);
    writeline(dl, ':SOUR:FUNC CURR'); pause(0.1);
    writeline(dl, sprintf(':SOUR:CURR:LEV:IMM %.4f', I_dis)); pause(0.1);
    writeline(dl, ':SOUR:CURR:SLEW 0.05'); pause(0.1);
    pause(0.3);

    %% === REST VOLTAGE (both instruments, nothing switched on) ===
    nR = 6;
    rDL = zeros(nR, 1); rDP = zeros(nR, 1);
    for k = 1:nR
        rDL(k) = qnum(dl, ':MEAS:VOLT:DC?');
        rDP(k) = qnum(dp, ':MEAS:VOLT:DC?');
        pause(0.3);
    end
    V0_DL = median(rDL); V0_DP = median(rDP);
    fprintf('>> [Board %s] Rest voltage: DL3021A reads %.3f V, DP813A reads %.3f V\n', boardLabel, V0_DL, V0_DP);
    if ~(V0_DL > 1.8 && V0_DL < 4.05)
        error('Rest voltage %.3f V (DL3021A) is outside 1.8-4.05 V: no cell connected, reversed, or sense open. Fix that first.', V0_DL);
    end
    dpUsable = abs(V0_DP - V0_DL) < 0.15;
    if ~dpUsable
        fprintf('>> NOTE: DP813A reads %.3f V while the DL3021A reads %.3f V with everything off.\n', V0_DP, V0_DL);
        fprintf('>>       Either the PSU cannot read a cell while its output is off, or its sense/wiring is wrong.\n');
        fprintf('>>       Discharge-side cross-checks will be skipped.\n');
    end

    %% === TWO PULSES (full cell: discharge first; otherwise charge first) ===
    if V0_DL >= 3.5
        fprintf('>> Order: discharge pulse first, then charge pulse.\n');
        [Pd, Qd] = dischargePulse(dp, dl, I_dis, T_dis, V_floor);
        pause(5);
        [Pc, Qc] = chargePulse(dp, dl, I_chg, V_lim, T_chg);
    else
        fprintf('>> Order: charge pulse first, then discharge pulse.\n');
        [Pc, Qc] = chargePulse(dp, dl, I_chg, V_lim, T_chg);
        pause(5);
        [Pd, Qd] = dischargePulse(dp, dl, I_dis, T_dis, V_floor);
    end
    safeOffOutputs(dp, dl);

    %% === ANALYSIS ===
    ec = Pc.t >= 4;                         % skip first seconds (output ramp)
    Ic = median(Pc.Idp(ec));
    R_force_chg = (median(Pc.Vdp(ec)) - median(Pc.Vdl(ec))) / Ic;
    R_int_chg   = (median(Pc.Vdl(ec)) - V0_DL) / Ic;   % rise of clean-tap voltage after current starts

    ed = Pd.t >= 2;
    Id = median(Pd.Idl(ed));
    dpUsable = dpUsable && median(Pd.Vdp(ed)) > 1.0;   % DP813A reads 0 V when its output is off
    if dpUsable
        R_force_dis = (median(Pd.Vdp(ed)) - median(Pd.Vdl(ed))) / Id;
        R_int_dis   = (Qd.Vdp(1) - mean(Pd.Vdp(end-1:end))) / Id;
    else
        R_force_dis = NaN;
        R_int_dis   = (V0_DL - median(Pd.Vdl(ed))) / Id;   % load-step drop, includes load-side wiring
    end

    g_int_c = grade(R_int_chg,        0.10, 0.15);
    g_int_d = grade(R_int_dis,        0.10, 0.15);
    g_f_c   = grade(abs(R_force_chg), 0.04, 0.10);
    g_f_d   = grade(abs(R_force_dis), 0.04, 0.10);

    fprintf('\n=========== CONNECTION TEST RESULT (Board %s) ===========\n', boardLabel);
    fprintf('Rest voltage                     : %.3f V (DL3021A)\n', V0_DL);
    fprintf('Charge pulse    : I=%.3f A, V_DP=%.3f V, V_DL=%.3f V\n', Ic, median(Pc.Vdp(ec)), median(Pc.Vdl(ec)));
    fprintf('Discharge pulse : I=%.3f A, V_DP=%.3f V, V_DL=%.3f V\n', Id, median(Pd.Vdp(ed)), median(Pd.Vdl(ed)));
    fprintf('R_int  (cell + holder contact), from charge pulse    : %.3f ohm  [%s]\n', R_int_chg, g_int_c);
    fprintf('R_int  (cell + holder contact), from discharge pulse : %.3f ohm  [%s]\n', R_int_dis, g_int_d);
    fprintf('R_force (PSU path, DP-DL at charge)                  : %.3f ohm  [%s]\n', R_force_chg, g_f_c);
    fprintf('R_force (load path, DP-DL at discharge)              : %.3f ohm  [%s]\n', R_force_dis, g_f_d);
    fprintf('Reference: R_int 0.05-0.09 ohm near full SOC; R_force ~0 with correct Kelvin sense.\n');
    if V0_DL < 3.6
        fprintf('NOTE: cell is at low/mid SOC (%.2f V) -- true cell resistance is higher there, so R_int WARN is less alarming.\n', V0_DL);
    end

    fprintf('\nVERDICT:\n');
    anyFail = false;
    if strcmp(g_f_c, 'FAIL') || strcmp(g_f_c, 'WARN')
        anyFail = true;
        fprintf(' - PSU path (DP813A force/sense, protection board, its cables) drops %.2f V at %.2f A. Check the DP813A sense leads reach the cell terminals and all screws on that path.\n', abs(R_force_chg)*Ic, Ic);
    end
    if strcmp(g_f_d, 'FAIL') || strcmp(g_f_d, 'WARN')
        anyFail = true;
        fprintf(' - Load path (DL3021A input/sense cables) drops %.2f V at %.2f A. Check the DL3021A sense leads and its input cables.\n', abs(R_force_dis)*Id, Id);
    end
    if any(strcmp({g_int_c, g_int_d}, 'FAIL')) || any(strcmp({g_int_c, g_int_d}, 'WARN'))
        anyFail = true;
        fprintf(' - Resistance common to both paths is high: holder contacts / cell ends / the cell. Clean + re-seat the cell, tighten terminals, then swap cells between boards A and B to see whether the fault follows the cell or stays with the board.\n');
    end
    if ~anyFail
        fprintf(' - All checks within limits: wiring and contacts look OK.\n');
    end

    %% === SAVE RAW SAMPLES ===
    Tall = [toTable('charge_pulse', Pc); toTable('charge_after_off', Qc); ...
            toTable('discharge_pulse', Pd); toTable('discharge_after_off', Qd)];
    fname = sprintf('%s - %s - ConnectionTest.xlsx', datestr(now, 'yyyy.mm.dd-HH.MM'), boardLabel);
    try
        writetable(Tall, fname);
        fprintf('\n>> Raw samples saved: %s\n', fname);
    catch err
        fprintf('\n>> Could not save raw samples (%s)\n', err.message);
    end
end

%% ----------------------------- helpers -----------------------------
function x = qnum(obj, cmd)
    x = str2double(query(obj, cmd));
end

function P = newP()
    P = struct('t', zeros(0,1), 'Vdp', zeros(0,1), 'Idp', zeros(0,1), 'Vdl', zeros(0,1), 'Idl', zeros(0,1));
end

function P = addSample(P, dp, dl, tt)
    P.t(end+1,1)   = tt;
    P.Vdp(end+1,1) = qnum(dp, ':MEAS:VOLT:DC?');
    P.Idp(end+1,1) = qnum(dp, ':MEAS:CURR:DC?');
    P.Vdl(end+1,1) = qnum(dl, ':MEAS:VOLT:DC?');
    P.Idl(end+1,1) = qnum(dl, ':MEAS:CURR:DC?');
end

function [P, Q] = chargePulse(dp, dl, I, Vlim, T)
    P = newP(); Q = newP();
    fprintf('>> Charge pulse: %.2f A for %d s...\n', I, T);
    writeline(dl, ':SOUR:INP:STAT OFF');
    writeline(dp, sprintf(':SOUR:CURR:LEV:IMM:AMPL %g', I));
    writeline(dp, sprintf(':SOUR:VOLT:LEV:IMM:AMPL %g', Vlim));
    writeline(dp, ':OUTP:STAT ON'); pause(0.1);
    t0 = tic;
    while toc(t0) < T
        P = addSample(P, dp, dl, toc(t0));
        if P.Vdp(end) > 4.05 || P.Idp(end) > 1.3*I
            writeline(dp, ':OUTP:STAT OFF');
            error('Charge pulse aborted: V=%.3f V, I=%.3f A outside the safe range.', P.Vdp(end), P.Idp(end));
        end
    end
    writeline(dp, ':OUTP:STAT OFF');
    t1 = tic; pause(0.6);   % instruments refresh readings slowly: wait before sampling
    for k = 1:3
        Q = addSample(Q, dp, dl, toc(t1));
        pause(0.2);
    end
end

function [P, Q] = dischargePulse(dp, dl, I, T, Vfloor)
    P = newP(); Q = newP();
    fprintf('>> Discharge pulse: %.2f A for %d s...\n', I, T);
    writeline(dp, ':OUTP:STAT OFF');
    writeline(dl, ':SOUR:INP:STAT ON'); pause(0.3);
    t0 = tic;
    while toc(t0) < T
        P = addSample(P, dp, dl, toc(t0));
        if P.Vdl(end) < Vfloor
            writeline(dl, ':SOUR:INP:STAT OFF');
            error('Discharge pulse aborted: V=%.3f V fell below %.2f V.', P.Vdl(end), Vfloor);
        end
    end
    writeline(dl, ':SOUR:INP:STAT OFF');
    t1 = tic; pause(0.6);   % instruments refresh readings slowly: wait before sampling
    for k = 1:3
        Q = addSample(Q, dp, dl, toc(t1));
        pause(0.2);
    end
end

function s = grade(x, passMax, warnMax)
    if isnan(x)
        s = 'n/a';
    elseif x <= passMax
        s = 'PASS';
    elseif x <= warnMax
        s = 'WARN';
    else
        s = 'FAIL';
    end
end

function T = toTable(phase, P)
    n = numel(P.t);
    T = table(repmat({phase}, n, 1), P.t, P.Vdp, P.Idp, P.Vdl, P.Idl, ...
        'VariableNames', {'Phase', 't_s', 'V_DP', 'I_DP', 'V_DL', 'I_DL'});
end

function safeOffOutputs(dp, dl)
    try, writeline(dp, ':OUTP:STAT OFF'); catch, end
    try, writeline(dl, ':SOUR:INP:STAT OFF'); catch, end
end

function safeOff(dp, dl)
    safeOffOutputs(dp, dl);
    try, delete(dp); catch, end
    try, delete(dl); catch, end
end

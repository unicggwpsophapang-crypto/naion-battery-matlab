function preconditionCellMultiBoard
%% ============================================================
%  MULTI-BOARD FORMATION CYCLING -- ONE INDEPENDENT TIMER PER BOARD
%  v2 (2026-10-30)
%
%  Recipe (unchanged): CC 0.5C to 4.0 V, CV until 0.1C, rest 30 min,
%  CC 1C discharge to 1.8 V, rest 30 min, repeated nCycles times.
%  Saves capacityRef_<cellID>.mat (C_measured = last discharge) and the
%  per-cycle xlsx files exactly as before.
%
%  CHANGES IN v2
%   1. Boards A, B and C. Instrument addresses now come from rigConfig.m
%      (edit board C's DP811 / DL3021 strings there once).
%   2. Start-voltage guard: refuses a cell below 1.80 V (over-discharged:
%      use recoveryChargeMulti first) or above 4.05 V.
%   3. Automatic pre-discharge: if the cell starts above 2.10 V it is
%      first discharged at 1C to 1.8 V and rested 30 min, so cycle 1 always
%      starts from empty (fixes the N01 / N12 cycle-1 CE artefacts).
%      Set preDischarge=false for a board to skip it.
%   4. Stale-reading watchdog: if V and I come back IDENTICAL for 20 ticks
%      (~1 min; never more than 2 in 181 real log files) while current should be flowing, or unreadable 5 times in a
%      row, that board stops (DL3021 freeze seen on N02/N03).
%   5. Output-trip check: in the CC part of the charge, current below 50%
%      of the setpoint for 10 ticks stops the board (OVP/OCP trip or open
%      circuit) instead of logging a fake charge.
%   6. Coulomb counting: the time step is reset at the start of every phase
%      and capped at 60 s, so a long pause (start delay, rest) can never be
%      integrated as charge if the first current reading is not zero.
%   7. Charge/discharge setpoints are sent once per phase instead of every
%      3 s -> less USB traffic with 3 boards, and an output that tripped
%      is NOT silently switched back on.
%   8. Per-board PSU voltage offset (rigConfig dpVoffset): Board A's DP813A
%      reads ~51 mV high, so its "4.0 V" was really ~3.95 V. Setpoint and
%      reading are corrected; logged V_DP is the corrected (real) value.
% ============================================================
    global boards t

    %% === RUN CONFIG: one column per board used in this run ===
    % Delete a column to leave that board out. cellID decides the name of
    % capacityRef_<cellID>.mat; the script refuses to overwrite an existing
    % one (rename the old file first if you redo a cell on purpose, e.g.
    % cycle 4 of N12: rename capacityRef_N12.mat -> capacityRef_N12_3cyc.mat).
    cfg = struct( ...
        'label',         {'A',     'B',     'C'}, ...
        'cellID',        {'M02',   'M04',   'M06'}, ...   % lot M formation batch 1 (21 Nov)
        'nCycles',       {3,       3,       3}, ...       % new cells -> 3 cycles
        'startDelayMin', {0,       0,      0}, ...      % stagger as before
        'preDischarge',  {true,    true,    true} ...     % cells rest at 2.36-2.43 V -> start from empty
        );

    nBoards = numel(cfg);

    %% --- refuse to run on top of anything else ---
    tt = timerfindall;
    if ~isempty(tt) && any(strcmp({tt.Running}, 'on'))
        error('A timer is still running (formation / main cycling / discharge). Stop it first: stop(timerfindall); delete(timerfindall)');
    end
    if ~isempty(tt), delete(tt); end
    closeOldHandles();

    %% --- cell-ID safety ---
    for b = 1:nBoards
        if isempty(cfg(b).cellID), error('Board %s: cellID is empty.', cfg(b).label); end
        if isfile(sprintf('capacityRef_%s.mat', cfg(b).cellID))
            error(['capacityRef_%s.mat already exists (board %s).\n' ...
                   'Either this cell was already formed, or you forgot to change cellID.\n' ...
                   'To redo on purpose: rename/move the old file first.'], cfg(b).cellID, cfg(b).label);
        end
        n = cfg(b).nCycles;
        if ~(isnumeric(n) && isscalar(n) && n >= 1 && n == round(n) && n <= 10)
            error('Board %s: nCycles must be a whole number from 1 to 10 (got %s).', cfg(b).label, mat2str(n));
        end
    end
    if numel(unique({cfg.cellID})) < nBoards, error('Two boards use the same cellID.'); end
    if numel(unique({cfg.label}))  < nBoards, error('Two columns use the same board label.'); end

    boards = repmat(emptyBoard(), 1, nBoards);
    t = cell(1, nBoards);

    fprintf('\n============ MULTI-BOARD FORMATION v2 (%d boards) ============\n', nBoards);
    try
        for b = 1:nBoards
            rig = rigConfig(cfg(b).label);
            bd = emptyBoard();
            bd.label = cfg(b).label;
            bd.cellID = cfg(b).cellID;
            bd.dt_lastP = datetime('now');
            if isfield(rig,'dpVoffset'), bd.dpOff = rig.dpVoffset; else, bd.dpOff = 0; end

            p = struct();
            p.N_formation_cycles = cfg(b).nCycles;
            p.capacity_nominal = 1300;
            C = p.capacity_nominal/1000;
            p.I_charge    = 0.5*C;
            p.V_charge    = 4.0;
            p.I_cutoff    = 0.1*C;
            p.I_discharge = 1.0*C;
            p.V_cutoff    = 1.8;
            p.V_OVP       = 4.1;
            p.I_OCP       = 1.0*C*1.3;
            p.V_tol       = 0.01;
            p.rest_time   = 30*60;
            p.EIS_measurement_duration = 5*60;
            p.EIS_buffer_after         = 5*60;
            p.V_start_min = 1.80;
            p.V_predis_above = 2.10;
            bd.params = p;

            fprintf('>> [Board %s] Connecting %s (%s) and DL (%s)...\n', bd.label, rig.dpModel, rig.visaDP, rig.visaDL);
            bd.DP813A = visadev(rig.visaDP); try, fopen(bd.DP813A); catch, end %#ok<TRYNC>
            setupPSURange(bd.DP813A, rig);
            writeline(bd.DP813A, ':OUTP:STAT OFF');
            writeline(bd.DP813A, [':SOUR:VOLT:PROT:LEV ', num2str(p.V_OVP + bd.dpOff)]);
            writeline(bd.DP813A, ':SOUR:VOLT:PROT:STAT ON');
            writeline(bd.DP813A, [':SOUR:CURR:PROT:LEV ', num2str(p.I_OCP)]);
            writeline(bd.DP813A, ':SOUR:CURR:PROT:STAT ON');

            bd.DL3021 = visadev(rig.visaDL); try, fopen(bd.DL3021); catch, end %#ok<TRYNC>
            writeline(bd.DL3021, ':SOUR:INP:STAT OFF'); pause(0.1);
            writeline(bd.DL3021, ':SOUR:FUNC:MODE FIX'); pause(0.1);
            writeline(bd.DL3021, ':SOUR:FUNC CURR'); pause(0.1);
            writeline(bd.DL3021, [':SOUR:CURR:LEV:IMM ', num2str(p.I_discharge, '%.4f')]); pause(0.1);
            writeline(bd.DL3021, ':SOUR:CURR:SLEW 0.05'); pause(0.3);
            boards(b) = bd;   % stored now so a setup error below still closes these handles

            % --- start-voltage guard (median of 3 readings) ---
            v = zeros(3,1);
            for k = 1:3, v(k) = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?')); pause(0.3); end
            V0 = median(v);
            bd.V_start = V0;
            if isnan(V0) || V0 < p.V_start_min
                error(['Board %s (%s): start voltage %.3f V is below %.2f V or unreadable. ' ...
                       'Do not form it directly: run recoveryChargeMulti first (or check the cell/sense wiring).'], ...
                       bd.label, bd.cellID, V0, p.V_start_min);
            end
            if V0 > 4.05
                error('Board %s (%s): start voltage %.3f V is above 4.05 V. Check the cell.', bd.label, bd.cellID, V0);
            end
            if V0 > p.V_predis_above && cfg(b).preDischarge
                bd.stateP = -1;
                fprintf('>> [Board %s / %s] Start %.3f V -> pre-discharge 1C to %.1f V, rest 30 min, then %d formation cycle(s).\n', ...
                    bd.label, bd.cellID, V0, p.V_cutoff, p.N_formation_cycles);
            else
                bd.stateP = 0;
                if V0 > p.V_predis_above
                    fprintf('>> NOTE [Board %s] start %.3f V and preDischarge=false: cycle-1 CE will not be meaningful.\n', bd.label, V0);
                end
                fprintf('>> [Board %s / %s] Start %.3f V -> %d formation cycle(s).\n', bd.label, bd.cellID, V0, p.N_formation_cycles);
            end
            boards(b) = bd;
        end
    catch err
        fprintf('>> Setup failed -- closing every instrument that was opened. Nothing was started.\n');
        closeAll();
        rethrow(err);
    end

    for b = 1:nBoards
        t{b} = timer('StartDelay', cfg(b).startDelayMin*60, 'Period', 3, 'TasksToExecute', inf, ...
            'ExecutionMode', 'fixedRate', 'BusyMode', 'drop', ...
            'TimerFcn', @(~,~)stepBoardTimer(b), ...
            'StopFcn',  @(~,~)cleanupOneBoard(b), ...
            'ErrorFcn', @(~,evt)errorOneBoard(b,evt));
        start(t{b});
        fprintf('>> [Board %s] Timer started (delay %d min).\n', cfg(b).label, cfg(b).startDelayMin);
    end
    fprintf('=====================================================================\n\n');
end

%% ============================================================
function bd = emptyBoard()
% Every board struct is created here, so all boards always have the SAME
% fields in the SAME order (needed for boards(b) = bd assignments).
    bd = struct('label','', 'cellID','', 'doneP',false, 'faultPaused',false, ...
        'stateP',0, 'counterP',0, 'cycleP',1, ...
        'tsP',datetime.empty(0,1), 'V_dpP',[], 'I_dpP',[], 'V_dlP',[], 'I_dlP',[], 'QP',[], ...
        'dt_lastP',NaT, 'C_measured',NaN, 'C_measured_history',[], ...
        'Q_charge_lastCycle',NaN, 'Q_charge_history',[], 'ICE_history',[], ...
        'params',struct(), 'V_start',NaN, 'preQ',0, ...
        'lastV',NaN, 'lastI',NaN, 'sameCount',0, 'nanCount',0, 'lowIcount',0, ...
        'dpOff',0, 'DP813A',[], 'DL3021',[]);
end

function stepBoardTimer(b)
    global boards t
    if boards(b).doneP, return; end
    boards(b) = stepOneBoard(boards(b));
    if boards(b).doneP
        if boards(b).faultPaused
            fprintf('>> [Board %s] Timer stopped by a safety check (see message above) -- NOT a real completion.\n', boards(b).label);
        else
            fprintf('>> [Board %s] FORMATION COMPLETE -- stopping Board %s''s timer only.\n', boards(b).label, boards(b).label);
        end
        stop(t{b});
    end
end

function [dt, bd] = takeDt(bd, nowT)
% Time since the previous sample of THIS phase, capped at 60 s.
    dt = seconds(nowT - bd.dt_lastP);
    if isempty(dt) || isnan(dt) || dt <= 0, dt = 3; end
    if dt > 60
        fprintf('>> WARNING [%s] %.0f s gap between samples -- integrated as 60 s. (MATLAB/USB busy?)\n', bd.label, dt);
        dt = 60;
    end
    bd.dt_lastP = nowT;
end

function [bd, bad] = watchdog(bd, V, I, what)
% Stops the board if the instrument readings are frozen or unreadable.
    bad = false;
    if isnan(V) || isnan(I), bd.nanCount = bd.nanCount + 1; else, bd.nanCount = 0; end
    if ~isnan(V) && ~isnan(I) && V == bd.lastV && I == bd.lastI
        bd.sameCount = bd.sameCount + 1;
    else
        bd.sameCount = 0;
    end
    bd.lastV = V; bd.lastI = I;
    if bd.nanCount >= 5 || bd.sameCount >= 20
        try, writeline(bd.DP813A, ':OUTP:STAT OFF'); catch, end
        try, writeline(bd.DL3021, ':SOUR:INP:STAT OFF'); catch, end
        if bd.nanCount >= 5
            fprintf('\n>> !!!! [%s] %s: readings unreadable 5 times in a row. Board STOPPED. !!!!\n\n', bd.label, what);
        else
            fprintf('\n>> !!!! [%s] %s: V=%.4f V and I=%.4f A identical for 20 samples (~1 min).\n', bd.label, what, V, I);
            fprintf('>>      Instrument reading is probably FROZEN (seen on the DL3021 before). Board STOPPED.\n');
            fprintf('>>      Power-cycle that instrument, run connectionTest, then redo this cell.\n\n');
        end
        bd.doneP = true; bd.faultPaused = true; bad = true;
    end
end

function [bd, bad] = loadCheck(bd, Idl, Iset, what)
% The electronic load must actually sink current once it is switched on.
    bad = false;
    if bd.counterP > 3 && Idl < 0.5*Iset, bd.lowIcount = bd.lowIcount + 1; else, bd.lowIcount = 0; end
    if bd.lowIcount >= 10
        try, writeline(bd.DL3021, ':SOUR:INP:STAT OFF'); catch, end
        fprintf('\n>> !!!! [%s] %s: load current %.3f A instead of %.3f A for 30 s. Board STOPPED. !!!!\n', bd.label, what, Idl, Iset);
        fprintf('>>      Check the DL input (power) leads to the cell, and the DL Von setting (must be below the cell voltage).\n\n');
        bd.doneP = true; bd.faultPaused = true; bad = true;
    end
end

function bd = resetLog(bd)
    bd.tsP = datetime.empty(0,1); bd.V_dpP = []; bd.I_dpP = []; bd.V_dlP = []; bd.I_dlP = []; bd.QP = [];
    bd.sameCount = 0; bd.nanCount = 0; bd.lowIcount = 0; bd.lastV = NaN; bd.lastI = NaN;
end

function bd = stepOneBoard(bd)
    nowT = datetime('now');
    bd.counterP = bd.counterP + 1;
    p = bd.params;

    switch bd.stateP
        case -1 % ----- PRE-DISCHARGE to 1.8 V (not recorded as a formation cycle) -----
            if bd.counterP == 1
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                writeline(bd.DL3021, ':SOUR:INP:STAT ON'); pause(0.3);
                bd.dt_lastP = nowT; bd.preQ = 0; bd = resetLog(bd);
            end
            Vdl = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?')); pause(0.08);
            Idl = str2double(query(bd.DL3021, ':MEAS:CURR:DC?'));
            [bd, bad] = watchdog(bd, Vdl, Idl, 'pre-discharge'); if bad, return; end
            if isnan(Vdl) || isnan(Idl), return; end
            [dtP, bd] = takeDt(bd, nowT);
            bd.preQ = bd.preQ + Idl*dtP/3.6;
            [bd, bad] = loadCheck(bd, Idl, p.I_discharge, 'pre-discharge'); if bad, return; end
            if mod(bd.counterP, 20) == 0
                fprintf('>> [%s | pre-discharge] V=%.3fV I=%.3fA Q=%.1f mAh\n', bd.label, Vdl, Idl, bd.preQ);
            end
            if Vdl <= p.V_cutoff
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                if bd.counterP <= 2
                    % A cell that was already discharged to 1.8 V rebounds to ~2.1-2.2 V
                    % at rest and falls back below 1.8 V as soon as 1C is applied:
                    % that is an EMPTY cell, not a fault.
                    fprintf('>> [%s] Cell is already empty (%.3f V at rest, below %.1f V under 1C at once). Skipping pre-discharge; resting 30 min.\n', ...
                        bd.label, bd.V_start, p.V_cutoff);
                else
                    fprintf('>> [%s] Pre-discharge done: %.1f mAh removed. Resting 30 min before formation cycle 1.\n', bd.label, bd.preQ);
                end
                bd.stateP = -2; bd.counterP = 0;
            end

        case -2 % ----- rest after pre-discharge -----
            if (bd.counterP*3) >= p.rest_time
                bd.stateP = 0; bd.counterP = 0; bd = resetLog(bd); bd.dt_lastP = datetime('now');
                fprintf('>> [%s] -> Starting formation cycle 1/%d\n', bd.label, p.N_formation_cycles);
            end

        case 0 % ----- FORMATION CHARGE (CC-CV) -----
            if bd.counterP == 1
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                writeline(bd.DP813A, [':SOUR:CURR:LEV:IMM:AMPL ', num2str(p.I_charge)]);
                writeline(bd.DP813A, [':SOUR:VOLT:LEV:IMM:AMPL ', num2str(p.V_charge + bd.dpOff)]);
                writeline(bd.DP813A, ':OUTP:STAT ON'); pause(0.1);
                bd.dt_lastP = nowT;
            end
            Vdp = str2double(query(bd.DP813A, ':MEAS:VOLT:DC?')) - bd.dpOff; pause(0.08);   % corrected to real cell voltage
            Idp = str2double(query(bd.DP813A, ':MEAS:CURR:DC?'));
            [bd, bad] = watchdog(bd, Vdp, Idp, 'charge'); if bad, return; end
            if isnan(Vdp) || isnan(Idp), return; end
            [dtP, bd] = takeDt(bd, nowT);

            bd.tsP(end+1,1) = nowT; bd.V_dpP(end+1,1) = Vdp; bd.I_dpP(end+1,1) = Idp;
            Q_prev = 0; if ~isempty(bd.QP), Q_prev = bd.QP(end); end
            Q_now = Q_prev + Idp*dtP/3.6;
            bd.QP(end+1,1) = Q_now;

            if mod(bd.counterP, 20) == 0
                fprintf('>> [%s | Cyc %d/%d] Charge: V=%.3fV I=%.3fA Q=%.1f mAh\n', ...
                    bd.label, bd.cycleP, p.N_formation_cycles, Vdp, Idp, Q_now);
            end

            if Vdp > p.V_charge + 0.05
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                fprintf('\n>> !!!! [%s] V=%.3f V is above %.2f V during charge. Board STOPPED. !!!!\n\n', bd.label, Vdp, p.V_charge + 0.05);
                bd.doneP = true; bd.faultPaused = true; return
            end
            % output tripped / open circuit: still in CC region but (almost) no current
            if bd.counterP > 3 && Vdp < p.V_charge - 0.05 && Idp < 0.5*p.I_charge
                bd.lowIcount = bd.lowIcount + 1;
            else
                bd.lowIcount = 0;
            end
            if bd.lowIcount >= 10
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                fprintf('\n>> !!!! [%s] Charge current %.3f A is far below %.3f A at V=%.3f V for 30 s.\n', bd.label, Idp, p.I_charge, Vdp);
                fprintf('>>      PSU output tripped (OVP/OCP) or the charge path is open. Board STOPPED. !!!!\n\n');
                bd.doneP = true; bd.faultPaused = true; return
            end

            if Vdp >= p.V_charge - p.V_tol && Idp <= p.I_cutoff
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                if bd.counterP <= 2
                    fprintf('\n>> !!!! [%s] SANITY CHECK FAILED: charge "completed" in only %.0f s -- probably NO CELL connected. Board STOPPED. !!!!\n\n', ...
                        bd.label, bd.counterP*3);
                    bd.doneP = true; bd.faultPaused = true; return
                end
                bd.Q_charge_lastCycle = bd.QP(end);
                bd.Q_charge_history(end+1,1) = bd.Q_charge_lastCycle;
                T = table(bd.tsP, bd.V_dpP, bd.I_dpP, bd.QP, 'VariableNames', {'DateTime','V_DP','I_DP','Q_mAh'});
                fname = sprintf('%s - %s - Formation Charge Cycle%d.xlsx', datestr(nowT,'yyyy.mm.dd-HH.MM'), bd.label, bd.cycleP);
                writetable(T, fname);
                bd.stateP = 1; bd.counterP = 0;
                fprintf('>> [%s | Cyc %d/%d] Charge complete. Q_charge=%.1f mAh (%s). Resting.\n', ...
                    bd.label, bd.cycleP, p.N_formation_cycles, bd.Q_charge_lastCycle, fname);
            end

        case 1 % ----- rest after charge (EIS window alert at the 20-min mark) -----
            EIS_alert_time = p.rest_time - p.EIS_measurement_duration - p.EIS_buffer_after;
            if (bd.counterP*3) >= EIS_alert_time && (bd.counterP*3) < EIS_alert_time + 3
                fprintf('>> [%s | Cyc %d/%d] >>> EIS WINDOW NOW <<<\n', bd.label, bd.cycleP, p.N_formation_cycles);
                try, sound(sin(2*pi*880*(0:1/8000:0.15)), 8000); catch, beep; end
            end
            if (bd.counterP*3) >= p.rest_time
                bd.stateP = 2; bd.counterP = 0; bd = resetLog(bd); bd.dt_lastP = datetime('now');
                fprintf('>> [%s | Cyc %d/%d] -> Discharge\n', bd.label, bd.cycleP, p.N_formation_cycles);
            end

        case 2 % ----- FORMATION DISCHARGE -----
            if bd.counterP == 1
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                writeline(bd.DL3021, ':SOUR:INP:STAT ON'); pause(0.3);
                bd.dt_lastP = nowT;
            end
            Vdl = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?')); pause(0.08);
            Idl = str2double(query(bd.DL3021, ':MEAS:CURR:DC?'));
            [bd, bad] = watchdog(bd, Vdl, Idl, 'discharge'); if bad, return; end
            if isnan(Vdl) || isnan(Idl), return; end
            [dtP, bd] = takeDt(bd, nowT);
            [bd, bad] = loadCheck(bd, Idl, p.I_discharge, 'discharge'); if bad, return; end

            bd.tsP(end+1,1) = nowT; bd.V_dlP(end+1,1) = Vdl; bd.I_dlP(end+1,1) = Idl;
            Q_prev = 0; if ~isempty(bd.QP), Q_prev = bd.QP(end); end
            Q_now = Q_prev + Idl*dtP/3.6;
            bd.QP(end+1,1) = Q_now;

            if mod(bd.counterP, 20) == 0
                fprintf('>> [%s | Cyc %d/%d] Discharge: V=%.3fV I=%.3fA Q=%.1f mAh\n', ...
                    bd.label, bd.cycleP, p.N_formation_cycles, Vdl, Idl, Q_now);
            end

            if Vdl <= p.V_cutoff
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                if bd.counterP <= 2
                    fprintf('\n>> !!!! [%s] SANITY CHECK FAILED: discharge "completed" in only %.0f s -- probably NO CELL connected. Board STOPPED. !!!!\n\n', ...
                        bd.label, bd.counterP*3);
                    bd.doneP = true; bd.faultPaused = true; return
                end
                bd.stateP = 3; bd.counterP = 0;
                bd.C_measured = bd.QP(end);
                bd.C_measured_history(end+1,1) = bd.C_measured;
                ICE_this = NaN;
                if ~isnan(bd.Q_charge_lastCycle) && bd.Q_charge_lastCycle > 0
                    ICE_this = bd.C_measured/bd.Q_charge_lastCycle*100;
                end
                bd.ICE_history(end+1,1) = ICE_this;
                T = table(bd.tsP, bd.V_dlP, bd.I_dlP, bd.QP, 'VariableNames', {'DateTime','V_DL','I_DL','Q_mAh'});
                fname = sprintf('%s - %s - Formation Discharge Cycle%d.xlsx', datestr(nowT,'yyyy.mm.dd-HH.MM'), bd.label, bd.cycleP);
                writetable(T, fname);
                fprintf('>> [%s | Cyc %d/%d] Discharge complete. C_measured=%.1f mAh  CE=%.1f%% (%s)\n', ...
                    bd.label, bd.cycleP, p.N_formation_cycles, bd.C_measured, ICE_this, fname);
                bd = resetLog(bd);
            end

        case 3 % ----- rest after discharge: next cycle or finish -----
            if (bd.counterP*3) >= p.rest_time
                if bd.cycleP < p.N_formation_cycles
                    bd.cycleP = bd.cycleP + 1;
                    bd.stateP = 0; bd.counterP = 0; bd = resetLog(bd); bd.dt_lastP = datetime('now');
                    fprintf('>> [%s] -> Starting formation cycle %d/%d\n', bd.label, bd.cycleP, p.N_formation_cycles);
                else
                    bd.doneP = true;
                    C_measured = bd.C_measured; C_measured_final = bd.C_measured; %#ok<NASGU>
                    C_measured_history = bd.C_measured_history; %#ok<NASGU>
                    Q_charge_history = bd.Q_charge_history; ICE_history = bd.ICE_history; %#ok<NASGU>
                    cellID = bd.cellID; boardLabel = bd.label; savedAt = datestr(now,'yyyy-mm-dd HH:MM'); %#ok<NASGU>
                    V_start = bd.V_start; preDischarge_mAh = bd.preQ; scriptVersion = 'precondition v2 2026-10-30'; %#ok<NASGU>
                    save(sprintf('capacityRef_%s.mat', bd.cellID), 'C_measured','C_measured_final', ...
                        'C_measured_history','Q_charge_history','ICE_history','cellID','boardLabel','savedAt', ...
                        'V_start','preDischarge_mAh','scriptVersion');
                    writetable(table((1:numel(bd.C_measured_history))', bd.Q_charge_history, bd.C_measured_history, bd.ICE_history, ...
                        'VariableNames', {'Cycle','Q_charge_mAh','Q_discharge_mAh','ICE_percent'}), ...
                        sprintf('%s - %s (%s) - Formation Cycles Summary.xlsx', datestr(datetime('now'),'yyyy.mm.dd-HH.MM'), bd.label, bd.cellID));
                    fprintf('>> [%s / %s] C_measured_final=%.1f mAh. Saved capacityRef_%s.mat\n', bd.label, bd.cellID, bd.C_measured, bd.cellID);
                end
            end
    end
end

function cleanupOneBoard(b)
% StopFcn: switch this board off and export any half-finished phase.
    global boards
    bd = boards(b);
    nowT = datetime('now');
    try
        if bd.stateP == 0 && ~isempty(bd.tsP)
            T = table(bd.tsP, bd.V_dpP, bd.I_dpP, bd.QP, 'VariableNames', {'DateTime','V_DP','I_DP','Q_mAh'});
            fname = sprintf('%s - %s - Interrupted Formation Charge Cycle%d.xlsx', datestr(nowT,'yyyy.mm.dd-HH.MM'), bd.label, bd.cycleP);
            writetable(T, fname);
            fprintf('>> [%s] Interrupted charge data exported: %s\n', bd.label, fname);
        elseif bd.stateP == 2 && ~isempty(bd.tsP)
            T = table(bd.tsP, bd.V_dlP, bd.I_dlP, bd.QP, 'VariableNames', {'DateTime','V_DL','I_DL','Q_mAh'});
            fname = sprintf('%s - %s - Interrupted Formation Discharge Cycle%d.xlsx', datestr(nowT,'yyyy.mm.dd-HH.MM'), bd.label, bd.cycleP);
            writetable(T, fname);
            fprintf('>> [%s] Interrupted discharge data exported: %s\n', bd.label, fname);
        end
    catch err
        fprintf('>> [%s] Warning: could not export interrupted data (%s)\n', bd.label, err.message);
    end
    try
        writeline(bd.DP813A, ':OUTP:STAT OFF');
        writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
    catch
        fprintf('>> Warning: could not cleanly stop instruments for board %s\n', bd.label);
    end
end

function errorOneBoard(b, event)
    global boards
    fprintf('>> [Board %s] Timer error: %s\n', boards(b).label, event.Data.message);
    cleanupOneBoard(b);
end

function closeAll()
    global boards
    for k = 1:numel(boards)
        try, writeline(boards(k).DP813A, ':OUTP:STAT OFF'); catch, end
        try, writeline(boards(k).DL3021, ':SOUR:INP:STAT OFF'); catch, end
        try, delete(boards(k).DP813A); catch, end
        try, delete(boards(k).DL3021); catch, end
    end
end

function closeOldHandles()
% Close VISA handles left in the global "boards" by an earlier (stopped) run.
    global boards
    try
        for k = 1:numel(boards)
            if isfield(boards(k),'DP813A') && ~isempty(boards(k).DP813A) && isvalid(boards(k).DP813A)
                try, writeline(boards(k).DP813A, ':OUTP:STAT OFF'); catch, end
                delete(boards(k).DP813A);
            end
            if isfield(boards(k),'DL3021') && ~isempty(boards(k).DL3021) && isvalid(boards(k).DL3021)
                try, writeline(boards(k).DL3021, ':SOUR:INP:STAT OFF'); catch, end
                delete(boards(k).DL3021);
            end
        end
    catch
    end
end

function mainCyclingMultiBoard_v2h(onlyBoards)
%% ============================================================
%  MAIN CYCLING -- MULTI-BOARD (A/B/C), ONE CONDITION PER BOARD, RESUMABLE
%  v2 (2026-10-30)
%
%  USAGE
%    mainCyclingMultiBoard          % start/resume every board with RUN_THIS_BOARD = true
%    mainCyclingMultiBoard('A')     % start/resume ONLY board A -- boards that are still
%                                   % running are NOT touched (no more waiting for the
%                                   % slowest board before resuming after an EIS stop)
%    mainCyclingMultiBoard('BC')
%
%  ONE CYCLE
%    CC charge at chargeCrate until Q reaches SOC_upper (or V reaches the
%    charge-stop voltage) -> rest 30 min -> CC 1C discharge until Q reaches
%    SOC 20% (or V reaches 2.00 V) -> rest 30 min.
%    SOC is Coulomb counted from C_measured in capacityRef_<cellID>.mat.
%
%  EIS CHECKPOINTS (cycles listed in p.EIS_cycles, default 1,5,10,15,20)
%    After the rest that follows the charge, the discharge goes down to
%    50% SOC, the load switches off, the cell rests 60 min and then the
%    board STOPS with a checkpoint. Do EIS / weight / dimensions, then
%    run  mainCyclingMultiBoard('<that board>')  to finish the discharge
%    to 20% and continue. (Plan: EIS at 50% SOC approached by discharge,
%    1 h rest, 25 C.) For chamber runs at 45 C the script tells you when
%    to switch the chamber to 25 C and back.
%
%  CHANGES IN v2
%   1. Board C (DP811 + DL3021); VISA strings come from rigConfig.m.
%   2. Resume/start a subset of boards while the others keep running.
%   3. Charge-stop voltage per datasheet: 3.95 V when chargeCrate > 1
%      (3C/2C), 4.00 V at 1C (was 4.03 V for all). The PSU voltage limit
%      is set to the same value; if the cell gets there before the SOC
%      target, the PSU holds it (CV) until the target is reached, so every
%      C-rate cycles the SAME SOC window. Charge > 1C refused unless
%      ambientC is 25-35 C (datasheet: fast charge only 25-35 C).
%   4. Each cycle records how charge and discharge ended in TripPhase:
%      "Q" = SOC target in CC, "Q+CV" = target reached after a CV hold,
%      "V" = CV current fell below 0.05C (charge) / hit 2.00 V (discharge)
%      before the SOC target. SOC_atCVstart_pct = SOC where the CV hold began
%      (NaN = no CV needed) -> report it so readers see how much was CV.
%   5. SOC-drift monitor: the rest voltage at 20% SOC after every
%      discharge is logged; a change of more than 60 mV from cycle 1
%      (~3% SOC on this steep part of the curve) prints a warning.
%      Rest voltage after charge and before EIS are logged too.
%   6. Fresh start begins with an anchor: 1C discharge to 1.8 V, rest
%      30 min, 0.5C charge of exactly 20% of C_measured, rest 30 min.
%      (set prepToSOC20 = false if you already did that yourself).
%   7. Stale-reading watchdog (20 identical V+I samples or 5 unreadable),
%      output-trip check, Coulomb time step reset per phase and capped.
%   8. Setpoints are sent once per phase, not every 3 s.
%  11. (v2c 2026-11-01) LOWER SOC BOUND IS VOLTAGE-ANCHORED: discharge ends when
%      the 1C-loaded voltage reaches V_anchor_low (cfg, = SOC 20%) and the count is
%      reset there; AnchorDrift_mAh in the Summary = how far the count had drifted.
%      Upper bound = 20% + 70/60/50% of C_measured by count from that anchor.
%  12. Every cycle's V/I/Q samples are saved to MainLog_<board>_<cell>_cycNN.csv.
%  13. (v2d 2026-11-01) ANCHOR SELF-CORRECTION FROM REST VOLTAGE: the loaded-voltage
%      anchor taken from formation put the bottom at ~24-26 % SOC in Run 1 cycles
%      6-10 (rest V after discharge +53..+91 mV above cycle 1, EIS V +50..+73 mV).
%      After every anchored discharge the 30-min rest voltage is compared with the
%      cycle-1 rest voltage at 20 % (Vref_low). If it differs by > 10 mV:
%        V_anchor_low -= 0.8*err (max 100 mV per step) and the count is corrected by
%        0.8*err / k_OCV (k_OCV from cycle 1: (V_EIS - Vref_low) / 30 % SOC).
%      On resume the correction is applied once from the last anchored cycle if it
%      has not been applied yet. Notes in the Summary say what was changed.
%  17. (v2h 2026-11-19) AUTO PRE-CHARGE before the anchor (fresh start only):
%      if the cell rests below cfg.preChargeV - 20 mV (cells stored at ~1.8-2.2 V),
%      the board first charges 0.5C CC-CV to preChargeV (2.60 V, CV until I < 0.05C,
%      max 90 min), rests 10 min, then runs the usual anchor (1C to 1.8 V ...).
%      Replaces the manual chargeOnly(...,0.5,2.8) step: a cell already at ~1.8 V only
%      touched 1.8 V for 3 s in Run 6, so 0 % was not defined. Step-R guard is NOT
%      applied here (charge-start step R at ~0 % SOC is ~0.2 ohm, normal).
%      preChargeV = NaN switches it off. Resumed boards are not affected.
%  16. (v2g 2026-11-09) SAFETY ONLY (charge/discharge/SOC logic unchanged):
%      a) Rest phases (-2,-4,1,3,4) now read DL voltage AND DL/PSU current every tick.
%         Current > 0.05 A while resting -> outputs re-sent OFF; still flowing after
%         5 ticks -> board STOPPED. DL voltage identical for 40 ticks (~2 min) ->
%         reading FROZEN -> STOPPED. Rest V below the discharge floor -> STOPPED.
%         (Run 3 N07, 9 Nov: DL3021A froze with the input ON during the pre-EIS rest
%         and drained the cell ~1.2 Ah unseen.)
%      b) Step resistance at charge/discharge start (|V(2nd sample) - V(rest)| / I)
%         > p.stepR_max (0.15 ohm) -> STOPPED: holder/contact problem, re-seat + connectionTest.
%      c) Every safety stop now saves the checkpoint with the CURRENT state and Q,
%         so a plain resume continues from the right SOC (no checkpoint edit).
%      d) Repeating alarm (3 high beeps every 30 s) after a safety stop until
%         stopAlarm is run (or the board is resumed).
%  15. (v2f 2026-11-06) VISA auto-reconnect: if a USB/VISA write/read fails, the board
%      reconnects its DP+DL (up to 3 tries, 5 s apart) and re-enters the current phase.
%      If all tries fail it saves a checkpoint with the CURRENT Q/state, then stops,
%      so resume continues from the right SOC (no manual checkpoint edit needed).
%  14. (v2e 2026-11-02) LOG ONLY: rest phases are now written to MainLog too
%      (state 1 = rest after charge, 3 = rest after discharge, 4 = rest before EIS;
%      DL voltage every tick, I = 0). Control logic is identical to v2d.
%   9. Checkpoints store the cellID; resuming with a different cell on
%      that board is refused. Result files carry the cellID in the name.
%  10. Per-board PSU voltage offset from rigConfig (dpVoffset): the PSU
%      setpoint and reading are corrected so 3.95 V / 4.00 V are the REAL
%      cell voltage on every board (Board A's DP813A reads ~51 mV high).
%   NOTE: start each board's checkpoint fresh with v2. Old v1 checkpoint
%   files (mainCyclingCheckpoint_A.mat ...) resume, but their first cycle
%   summary row may miss the new columns (filled with NaN).
% ============================================================
    global boards t

    %% === SESSION CONFIG ===
    N_CYCLES_THIS_SESSION = 20;   % EIS checkpoints stop the board anyway
    N_CYCLES_TOTAL        = 20;

    %% === BOARD CONFIG: one column per board, condition is per board ===
    cfg = struct( ...
        'label',         {'A',      'B',      'C'}, ...
        'cellID',        {'M01',    'M05',    'M06'}, ...     % <-- Run 7 (3C / 20-70% / 25 C, lot N). Physical cell on this board (must have capacityRef_<cellID>.mat)
        'RUN_THIS_BOARD',{true,     true,     true}, ...
        'chargeCrate',   {2,        2,        2}, ...         % <-- 1, 2 or 3
        'SOC_upper_pct', {90,       90,       90}, ...        % <-- 70, 80 or 90
        'ambientC',      {45,       45,       45}, ...        % <-- 25 or 45 (chamber setpoint)
        'prepToSOC20',   {true,     true,     true}, ...      % fresh start: anchor at 1.8 V then charge 20%
        'preChargeV',    {2.60,     2.60,     2.60}, ...      % v2h: fresh start pre-charge 0.5C CC-CV to this V if the cell rests lower (NaN = off)
        'startDelayMin', {0,        0,       0}, ...        % fresh start only; resume starts at once
        'V_anchor_low',  {NaN,      NaN,      NaN}, ...        % 1C-loaded voltage at SOC 20% (V). NaN = learn it at cycle 1
        'allowOutOfSpec',{true,     true,     true});
    % V_anchor_low: discharge now ends when the 1C-LOADED voltage reaches this
    % value (= SOC 20% of this cell), and the Coulomb count is reset to 20% there.
    % Coulomb counting alone drifted 1-2 % SOC per cycle in Run 1 (charge and
    % discharge are counted by different instruments + true CE < 100 %).
    % Values for Cell01/03/04: 1C formation discharge of 2026-10-31 at Q = 0.8*C.
    % NaN: cycle 1 ends on Q (just anchored, so exact) and its end voltage is learnt.

    nBoards = numel(cfg);
    labels = {cfg.label};
    if numel(unique(labels)) < nBoards, error('Two columns use the same board label.'); end

    if nargin < 1 || isempty(onlyBoards)
        sel = find([cfg.RUN_THIS_BOARD]);
    else
        want = cellstr(upper(char(onlyBoards))')';
        sel = find(ismember(labels, want));
        missing = setdiff(want, labels);
        if ~isempty(missing), error('Board(s) %s not in cfg.', strjoin(missing, ',')); end
        notEnabled = labels(sel(~[cfg(sel).RUN_THIS_BOARD]));
        if ~isempty(notEnabled), error('Board(s) %s have RUN_THIS_BOARD = false in cfg.', strjoin(notEnabled, ',')); end
    end
    if isempty(sel), error('No board selected to run.'); end

    %% === keep running boards, rebuild the global layout only if it is safe ===
    layoutOK = ~isempty(boards) && isstruct(boards) && numel(boards) == nBoards && ...
               isequal(sort(fieldnames(boards)), sort(fieldnames(emptyBoard()))) && ...
               isequal({boards.label}, labels) && iscell(t) && numel(t) == nBoards;
    if ~layoutOK
        tt = timerfindall;
        if ~isempty(tt), tt = tt(~strcmp({tt.Tag}, 'NaIonAlarm')); end   % v2g: the alarm timer does not count
        if ~isempty(tt) && any(strcmp({tt.Running}, 'on'))
            error(['A timer is still running but the board layout changed (cfg edited, or an old script version). ' ...
                   'Wait for it to stop, or stop it: stop(timerfindall); delete(timerfindall)']);
        end
        if ~isempty(tt), delete(tt); end
        closeOldHandles();
        boards = repmat(emptyBoard(), 1, nBoards);
        for b = 1:nBoards, boards(b).label = labels{b}; boards(b).doneM = true; end
        t = cell(1, nBoards);
    end
    for b = sel
        if ~isempty(t{b}) && isvalid(t{b}) && strcmp(t{b}.Running, 'on')
            error('Board %s is STILL RUNNING. Only boards that have stopped can be (re)started.', labels{b});
        end
    end

    stopAlarmTimers();   % v2g: (re)starting a board silences the fault alarm
    fprintf('\n============ MAIN CYCLING v2h: starting/resuming board(s) %s ============\n', strjoin(labels(sel), ', '));

    for b = sel
        c = cfg(b);
        rig = rigConfig(c.label);
        if c.chargeCrate > 1 && (c.ambientC < 25 || c.ambientC > 35)
            if isfield(c,'allowOutOfSpec') && c.allowOutOfSpec
                warning('Board %s: %gC charge at %g C is OUTSIDE the datasheet window - allowed by cfg.allowOutOfSpec.', c.label, c.chargeCrate, c.ambientC);
            else
                error('Board %s: %gC charge at %g C is outside the datasheet window (fast charge only at 25-35 C).', c.label, c.chargeCrate, c.ambientC);
            end
        end
        if ~ismember(c.SOC_upper_pct, [70 80 90]), warning('Board %s: SOC_upper_pct = %g is not one of the planned windows.', c.label, c.SOC_upper_pct); end

        % close this board's old instrument handles (its timer is stopped)
        try, writeline(boards(b).DP813A, ':OUTP:STAT OFF'); catch, end   % v2g.1: delete even if the write fails
        try, delete(boards(b).DP813A); catch, end
        try, writeline(boards(b).DL3021, ':SOUR:INP:STAT OFF'); catch, end
        try, delete(boards(b).DL3021); catch, end
        if ~isempty(t{b}), try, delete(t{b}); catch, end, t{b} = []; end

        bd = emptyBoard();
        bd.label = c.label; bd.cellID = c.cellID;
        checkpointFile = sprintf('mainCyclingCheckpoint_%s.mat', c.label);
        isResume = isfile(checkpointFile);

        if isResume
            S = load(checkpointFile);
            if isfield(S, 'cellID') && ~strcmp(S.cellID, c.cellID)
                error(['Board %s: checkpoint belongs to cell %s but cfg says %s. Put the right cell back, ' ...
                       'or move %s away if you really start a new cell.'], c.label, S.cellID, c.cellID, checkpointFile);
            end
            bd.params = S.params;
            bd.params = upgradeParams(bd.params, c);   % adds any field newer than the checkpoint (keeps existing values)
            if bd.params.chargeCrate ~= c.chargeCrate || round(bd.params.SOC_upper*100) ~= c.SOC_upper_pct
                error(['[%s] Checkpoint condition (%gC, SOC20-%.0f%%) differs from cfg (%gC, SOC20-%.0f%%). ' ...
                       'Delete %s if you meant to start a NEW condition.'], c.label, bd.params.chargeCrate, ...
                       bd.params.SOC_upper*100, c.chargeCrate, c.SOC_upper_pct, checkpointFile);
            end
            bd.stateM = S.stateM; bd.cycleM = S.cycleM; bd.QcurrentM = S.QcurrentM;
            bd.phaseLogM = S.phaseLogM; bd.cycleSummaryM = ensureSummaryCols(S.cycleSummaryM);
            bd.Q_chg_this = getf(S, 'Q_chg_this', 0); bd.Q_dis_this = getf(S, 'Q_dis_this', 0);
            bd.eisDone = getf(S, 'eisDone', false); bd.Vref_low = getf(S, 'Vref_low', NaN);
            bd.Vrest_chg = getf(S, 'Vrest_chg', NaN); bd.V_EIS = getf(S, 'V_EIS', NaN);
            bd.chgTrip = getf(S, 'chgTrip', ""); bd.socCV = getf(S, 'socCV', NaN);
            if isnan(bd.params.V_anchor_low) && ~isnan(c.V_anchor_low)
                bd.params.V_anchor_low = c.V_anchor_low;
            end
            % v2d: apply the rest-voltage correction once from the last anchored cycle
            if ~isnan(bd.params.V_anchor_low) && ~isnan(bd.Vref_low) && height(bd.cycleSummaryM) > 0 && ismember(bd.stateM, [0 2])
                lastRow = bd.cycleSummaryM(end, :);
                if lastRow.Cycle > bd.params.anchorFB_lastCyc && contains(string(lastRow.TripPhase), "Vanchor") && ~isnan(lastRow.Vrest_afterDischarge_V)
                    % state 2 = about to discharge to the anchor (count gets reset there), so only move the anchor
                    [bd, fbNote] = anchorFeedback(bd, lastRow.Vrest_afterDischarge_V, bd.stateM == 0);
                    bd.params.anchorFB_lastCyc = lastRow.Cycle;
                    if strlength(fbNote) > 0, fprintf('>>   [%s] On resume (from cycle %d rest V): %s\n', c.label, lastRow.Cycle, fbNote); end
                end
            end
            fprintf('>>   Lower SOC anchor: %.3f V (1C loaded).\n', bd.params.V_anchor_low);
            fprintf('>> [Board %s / %s] RESUMING: cycle %d/%d, state %d, Q=%.1f mAh (SOC %.1f%%).\n', c.label, c.cellID, ...
                bd.cycleM, bd.params.N_cycles, bd.stateM, bd.QcurrentM, 100*bd.QcurrentM/bd.params.C_measured);
            if bd.params.ambientC ~= 25 && bd.stateM == 2 && bd.eisDone
                fprintf('>>   Chamber run: chamber must be back at %g C and the cell soaked (>= 1-2 h) before this resume.\n', bd.params.ambientC);
            end
        else
            matname = sprintf('capacityRef_%s.mat', c.cellID);
            if ~isfile(matname)
                error('%s not found -- form cell %s first with preconditionCellMultiBoard (or check cellID).', matname, c.cellID);
            end
            S2 = load(matname, 'C_measured');
            bd.params = makeParams(c, S2.C_measured, N_CYCLES_TOTAL);
            p = bd.params;
            fprintf('>> [Board %s / %s] Fresh start: %gC charge, SOC 20-%g%%, %g C. C_measured = %.1f mAh (%s)\n', ...
                c.label, c.cellID, p.chargeCrate, c.SOC_upper_pct, p.ambientC, p.C_measured, matname);
            fprintf('>>   Q window %.1f-%.1f mAh, I_charge %.3f A, charge stop %.2f V, EIS at cycles %s.\n', ...
                p.Q_lower, p.Q_upper, p.I_charge, p.V_chg_stop, mat2str(p.EIS_cycles));
            bd.cycleM = 1;
            if c.prepToSOC20
                bd.stateM = -1; bd.QcurrentM = NaN;
            else
                bd.stateM = 0; bd.QcurrentM = p.Q_lower;
                fprintf('>>   prepToSOC20=false: trusting that the cell is at SOC 20%% NOW.\n');
            end
        end
        bd.N_cycles_this_session = N_CYCLES_THIS_SESSION;
        bd.dt_lastM = datetime('now'); bd.phaseStartM = datetime('now');
        if isfield(rig,'dpVoffset'), bd.dpOff = rig.dpVoffset; else, bd.dpOff = 0; end
        bd.rigCfg = rig; bd.nReconnect = 0;   % v2f

        % --- connect ---
        p = bd.params;
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

        V0 = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?'));
        if isnan(V0) || V0 < 1.8 || V0 > 4.05
            try, delete(bd.DP813A); delete(bd.DL3021); catch, end
            error('Board %s: cell voltage %.3f V (DL) is outside 1.80-4.05 V or unreadable. Check the cell and sense wiring.', c.label, V0);
        end
        fprintf('>> [Board %s] Ready. Cell at %.3f V.\n', c.label, V0);
        % v2h: fresh start with a deeply discharged cell -> pre-charge to V_precharge first
        if ~isResume && bd.stateM == -1 && ~isnan(p.V_precharge) && V0 < p.V_precharge - 0.02
            bd.stateM = -6;
            fprintf('>>   Cell below %.2f V -> PRE-CHARGE 0.5C CC-CV to %.2f V, rest %.0f min, then anchor 1C to %.2f V.\n', ...
                p.V_precharge, p.V_precharge, p.precharge_rest/60, p.V_prep_cutoff);
        end
        boards(b) = bd;

        delayS = 0; if ~isResume, delayS = c.startDelayMin*60; end
        t{b} = timer('StartDelay', delayS, 'Period', 3, 'TasksToExecute', inf, ...
            'ExecutionMode', 'fixedRate', 'BusyMode', 'drop', ...
            'TimerFcn', @(~,~)stepBoardMain(b), ...
            'StopFcn',  @(~,~)cleanupOneBoardMain(b), ...
            'ErrorFcn', @(~,evt)errorOneBoardMain(b,evt));
        start(t{b});
        fprintf('>> [Board %s] Timer started (delay %.0f min).\n', c.label, delayS/60);
    end
    fprintf('=====================================================================\n\n');
end

%% ============================================================
function p = makeParams(c, C_measured, N_total)
    p = struct();
    p.cellID         = c.cellID;
    p.chargeCrate    = c.chargeCrate;
    p.dischargeCrate = 1;
    p.ambientC       = c.ambientC;
    p.C_measured     = C_measured;
    p.SOC_lower      = 0.20;
    p.SOC_upper      = c.SOC_upper_pct/100;
    p.Q_lower        = p.SOC_lower * C_measured;
    p.Q_upper        = p.SOC_upper * C_measured;
    p.I_charge       = p.chargeCrate    * C_measured/1000;
    p.I_discharge    = p.dischargeCrate * C_measured/1000;
    p.I_prep_charge  = 0.5 * C_measured/1000;
    if p.chargeCrate > 1, p.V_chg_stop = 3.95; else, p.V_chg_stop = 4.00; end
    p.V_tol             = 0.01;
    p.I_cv_cutoff       = 0.05 * C_measured/1000;   % end CV early only if the current is this low
    p.V_OVP             = p.V_chg_stop + 0.10;
    p.V_discharge_floor = 2.00;
    p.V_prep_cutoff     = 1.80;
    p.I_OCP          = max(p.chargeCrate, p.dischargeCrate) * C_measured/1000 * 1.3;
    p.rest_time      = 30*60;
    p.N_cycles       = N_total;
    p.EIS_cycles     = [1 5 10 15 20];
    p.EIS_SOC        = 0.50;
    p.Q_EIS          = p.EIS_SOC * C_measured;
    p.EIS_rest_time  = 60*60;
    p.drift_warn_V   = 0.060;
    p.V_anchor_low   = c.V_anchor_low;      % NaN -> learnt at the end of cycle 1
    p.Q_safety_below = 0.15 * C_measured;   % stop if the count runs this far below 20% without reaching the anchor voltage
    p.anchorFB_gain     = 0.8;     % fraction of the rest-voltage error corrected per step
    p.anchorFB_deadband = 0.010;   % V: ignore smaller rest-voltage errors
    p.anchorFB_maxStep  = 0.100;   % V: max anchor change per step
    p.anchorFB_lastCyc  = 0;       % last cycle whose rest voltage was used
    p.V_anchor_min      = 2.30;    % V: anchor never set below this
    p.V_anchor_max      = 2.80;
    p.scriptVersion  = 'mainCycling v2h 2026-11-19';
    p.stepR_max      = 0.15;   % v2g: ohm, step resistance limit at charge/discharge start
    p.I_rest_max     = 0.05;   % v2g: A, current allowed while resting
    p.restFrozenN    = 40;     % v2g: identical rest readings in a row (~2 min) = frozen
    if isfield(c, 'preChargeV'), p.V_precharge = c.preChargeV; else, p.V_precharge = NaN; end   % v2h
    p.precharge_rest = 10*60;  % v2h: s, rest after the pre-charge
    p.precharge_maxS = 90*60;  % v2h: s, pre-charge time limit
end

function p = upgradeParams(p, c)
% v1 checkpoint -> add the v2 fields with the v2 values.
    q = makeParams(c, p.C_measured, p.N_cycles);
    f = fieldnames(q);
    for k = 1:numel(f), if ~isfield(p, f{k}), p.(f{k}) = q.(f{k}); end, end
end

function [bd, note] = anchorFeedback(bd, Vlow, fixCount)
% v2d: compare the rest voltage after an anchored discharge with the cycle-1
% rest voltage at SOC 20% (bd.Vref_low) and move the anchor (and the count).
    p = bd.params; note = "";
    err = Vlow - bd.Vref_low;                 % >0: cell rests higher than at 20% -> bottom SOC too high
    if abs(err) <= p.anchorFB_deadband, return; end
    dV = max(-p.anchorFB_maxStep, min(p.anchorFB_maxStep, p.anchorFB_gain * err));
    Vold = p.V_anchor_low;
    Vnew = max(p.V_anchor_min, min(p.V_anchor_max, Vold - dV));
    bd.params.V_anchor_low = Vnew;
    note = sprintf("anchor FB: rest %+.0f mV vs cyc1 -> anchor %.3f->%.3f V", 1000*err, Vold, Vnew);
    if fixCount
        % OCV slope from cycle 1 (V per mAh between 20% and 50% SOC)
        k = NaN;
        S = bd.cycleSummaryM;
        if height(S) > 0 && S.Cycle(1) == 1 && ~isnan(S.V_beforeEIS_V(1))
            k = (S.V_beforeEIS_V(1) - bd.Vref_low) / (0.30 * p.C_measured);
        end
        if isnan(k) || k <= 0, k = 0.0135 / (p.C_measured/100); end   % fallback 13.5 mV per % SOC
        dQ = p.anchorFB_gain * err / k;
        dQ = max(-0.08*p.C_measured, min(0.08*p.C_measured, dQ));
        bd.QcurrentM = bd.QcurrentM + dQ;
        note = note + sprintf(", count %+.1f mAh (SOC now %.1f%%)", dQ, 100*bd.QcurrentM/p.C_measured);
    end
end

function v = getf(S, name, default)
    if isfield(S, name), v = S.(name); else, v = default; end
end

function T = newSummary()
    T = table('Size', [0 11], 'VariableTypes', {'double','double','double','double','string','double','double','double','double','double','string'}, ...
        'VariableNames', {'Cycle','Q_charge_mAh','Q_discharge_mAh','CE_pct','TripPhase','SOC_atCVstart_pct', ...
                          'Vrest_afterCharge_V','Vrest_afterDischarge_V','V_beforeEIS_V','AnchorDrift_mAh','Note'});
end

function T = ensureSummaryCols(T)
    ref = newSummary();
    for v = ref.Properties.VariableNames
        if ~ismember(v{1}, T.Properties.VariableNames)
            if strcmp(class(ref.(v{1})), 'string'), T.(v{1}) = strings(height(T), 1); else, T.(v{1}) = nan(height(T), 1); end
        end
    end
    T = T(:, ref.Properties.VariableNames);
end

function bd = emptyBoard()
% All board structs are built here -> identical fields, identical order.
    bd = struct('label','', 'cellID','', 'faultPaused',false, 'doneM',false, ...
        'stateM',0, 'counterM',0, 'cycleM',1, 'dt_lastM',NaT, 'phaseStartM',NaT, ...
        'QcurrentM',NaN, 'phaseLogM',table('Size',[0 4], 'VariableTypes', {'double','string','datetime','datetime'}, ...
                'VariableNames', {'Cycle','Phase','StartTime','EndTime'}), ...
        'cycleSummaryM',newSummary(), 'params',struct(), ...
        'Q_chg_this',0, 'Q_dis_this',0, 'Q_prep',0, 'cyclesDoneThisSession',0, 'N_cycles_this_session',20, ...
        'chgTrip',"", 'disTrip',"", 'cvEntered',false, 'socCV',NaN, 'Vrest_chg',NaN, 'V_EIS',NaN, 'Vref_low',NaN, 'eisDone',false, ...
        'lastV',NaN, 'lastI',NaN, 'sameCount',0, 'nanCount',0, 'lowIcount',0, ...
        'lowVcount',0, 'anchorDQ',NaN, 'tlog',zeros(0,6), ...
        'dpOff',0, 'DP813A',[], 'DL3021',[], 'rigCfg',struct(), 'nReconnect',0, ...   % v2f: rigCfg/nReconnect
        'Vpre',NaN, 'restSame',0, 'restLastV',NaN, 'restIcount',0);            % v2g
end

function stepBoardMain(b)
    global boards t
    if boards(b).doneM, return; end
    try
        boards(b) = stepOneBoardMain(boards(b));
    catch err
        if ~isVisaError(err), rethrow(err); end
        % ---- v2f: VISA/USB dropped -> reconnect and re-enter the current phase ----
        bd = boards(b);
        fprintf('\n>> !!!! [%s] VISA error in state %d, cycle %d, Q=%.1f mAh: %s\n', bd.label, bd.stateM, bd.cycleM, bd.QcurrentM, err.message);
        ok = false;
        for k = 1:3
            pause(5);
            fprintf('>> [%s] Reconnect try %d/3 ...\n', bd.label, k);
            [bd, ok] = reconnectBoard(bd);
            if ok, break; end
        end
        if ok
            bd.nReconnect = bd.nReconnect + 1;
            bd.counterM = 0;                       % next tick re-sends the setpoints of this phase
            fprintf('>> [%s] Reconnected (#%d this session). Continuing state %d, Q=%.1f mAh (charge/discharge time during the gap is not counted, <=~5 mAh).\n\n', ...
                bd.label, bd.nReconnect, bd.stateM, bd.QcurrentM);
            boards(b) = bd;
            return
        end
        fprintf('>> !!!! [%s] Reconnect FAILED. Saving checkpoint at the CURRENT state/Q, then stopping.\n', bd.label);
        fprintf('>>      Switch OFF the DP output and DL input BY HAND now (the PC can no longer reach them).\n');
        fprintf('>>      Then replug USB, check visadevlist, and resume this board -- no checkpoint edit needed.\n\n');
        try, saveCheckpoint(bd); catch e2, fprintf('>> [%s] could not save checkpoint: %s\n', bd.label, e2.message); end
        boards(b) = bd;
        rethrow(err);
    end
    if boards(b).doneM
        bd = boards(b);
        if bd.faultPaused
            try, writeline(bd.DP813A, ':OUTP:STAT OFF'); catch, end
            try, writeline(bd.DL3021, ':SOUR:INP:STAT OFF'); catch, end
            bd = saveFaultCheckpoint(bd);
            boards(b) = bd;
            fprintf('>> [Board %s] Timer stopped by a safety check -- NOT a real completion.\n', bd.label);
            fprintf('>>      CHECK BY HAND that the DP output and the DL INPUT lights are OFF.\n');
            fprintf('>>      Fix the cause, run stopAlarm, connectionTest(''%s''), then mainCyclingMultiBoard_v2h(''%s'') -- no checkpoint edit needed.\n', bd.label, bd.label);
            startAlarm(bd.label);
        elseif bd.cycleM > bd.params.N_cycles
            fprintf('>> [Board %s / %s] ALL %d CYCLES COMPLETE for this condition.\n', bd.label, bd.cellID, bd.params.N_cycles);
        elseif bd.stateM == 2 && bd.eisDone
            fprintf('>> [Board %s] Stopped for EIS checkpoint (cycle %d, 50%% SOC). When done: mainCyclingMultiBoard_v2h(''%s'')\n', bd.label, bd.cycleM, bd.label);
        else
            fprintf('>> [Board %s] Session limit reached. Checkpoint saved -- mainCyclingMultiBoard_v2h(''%s'') resumes at cycle %d.\n', bd.label, bd.label, bd.cycleM);
        end
        stop(t{b});
    end
end

function bd = saveFaultCheckpoint(bd)
% v2g: save the CURRENT state/Q so resume continues from the real SOC.
% Anchor charge (-3) restarts from the 1.8 V anchor because its count starts at 0.
    try
        if bd.stateM == -3, bd.stateM = -1; bd.QcurrentM = NaN; end
        if ismember(bd.stateM, [1 3 4 -2 -4 -5]), bd.phaseStartM = datetime('now'); end   % rest restarts in full
        bd.counterM = 0;
        saveCheckpoint(bd);
        fprintf('>> [%s] Fault checkpoint saved: cycle %d, state %d, Q=%.1f mAh (SOC %.1f%%).\n', bd.label, bd.cycleM, ...
            bd.stateM, bd.QcurrentM, 100*bd.QcurrentM/bd.params.C_measured);
    catch e
        fprintf('>> [%s] could not save fault checkpoint: %s\n', bd.label, e.message);
    end
end

function startAlarm(label)
% v2g: 3 high beeps every 30 s until stopAlarm (or a board is restarted).
    try
        tmr = timer('Tag','NaIonAlarm', 'Period',30, 'ExecutionMode','fixedSpacing', 'TasksToExecute',inf, ...
            'TimerFcn',@(~,~)alarmSound(label));
        start(tmr);
    catch
        alarmSound(label);
    end
end

function alarmSound(label)
    fs = 8000; tone = 0.9*sin(2*pi*2000*(0:1/fs:0.25)); gap = zeros(1, round(0.15*fs));
    try, sound([tone gap tone gap tone], fs); catch, beep; end
    fprintf('>> !!!! ALARM: board %s stopped on a safety check (%s). Run stopAlarm after you have checked it.\n', label, datestr(now,'HH:MM:SS'));
end

function stopAlarmTimers()
    try
        tt = timerfindall('Tag','NaIonAlarm');
        if ~isempty(tt), stop(tt); delete(tt); end
    catch
    end
end

function [bd, bad] = restGuard(bd, nowT, stateCode)
% v2g: replaces logRest. Logs the DL voltage AND checks that nothing flows while resting.
    bad = false;
    V = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?')); pause(0.05);
    Idl = str2double(query(bd.DL3021, ':MEAS:CURR:DC?')); pause(0.05);
    Idp = str2double(query(bd.DP813A, ':MEAS:CURR:DC?'));
    if ~isnan(V)
        Ia = [abs(Idl) abs(Idp)]; Ia = max([Ia(~isnan(Ia)) 0]);
        bd.tlog(end+1,:) = [posixtime(nowT), stateCode, V, Ia, bd.QcurrentM, bd.cycleM];
    end
    p = bd.params;
    % 1) current while resting
    if (~isnan(Idl) && abs(Idl) > p.I_rest_max) || (~isnan(Idp) && abs(Idp) > p.I_rest_max)
        bd.restIcount = bd.restIcount + 1;
        try, writeline(bd.DL3021, ':SOUR:INP:STAT OFF'); catch, end
        try, writeline(bd.DP813A, ':OUTP:STAT OFF'); catch, end
        if bd.restIcount == 1
            fprintf('>> WARNING [%s] %.3f A (DL) / %.3f A (PSU) flowing during a REST -- re-sent OFF.\n', bd.label, Idl, Idp);
        end
    else
        bd.restIcount = 0;
    end
    if bd.restIcount >= 5
        fprintf('\n>> !!!! [%s] Current still flowing during REST after 5 OFF commands (DL %.3f A, PSU %.3f A). Board STOPPED. !!!!\n', bd.label, Idl, Idp);
        fprintf('>>      SWITCH THE DL INPUT / PSU OUTPUT OFF BY HAND, power-cycle that instrument.\n\n');
        bd.doneM = true; bd.faultPaused = true; bad = true; return
    end
    % 2) frozen reading
    if ~isnan(V) && V == bd.restLastV, bd.restSame = bd.restSame + 1; else, bd.restSame = 0; end
    bd.restLastV = V;
    if bd.restSame >= p.restFrozenN
        try, writeline(bd.DL3021, ':SOUR:INP:STAT OFF'); catch, end
        try, writeline(bd.DP813A, ':OUTP:STAT OFF'); catch, end
        fprintf('\n>> !!!! [%s] DL voltage %.6f V identical for %d readings (~2 min) during REST -- DL FROZEN. Board STOPPED. !!!!\n', bd.label, V, bd.restSame);
        fprintf('>>      The load may still be sinking current: switch the DL INPUT OFF BY HAND and power-cycle the DL.\n\n');
        bd.doneM = true; bd.faultPaused = true; bad = true; return
    end
    % 3) cell drained while resting
    if ~isnan(V) && V < p.V_discharge_floor - 0.1 && stateCode ~= -2
        try, writeline(bd.DL3021, ':SOUR:INP:STAT OFF'); catch, end
        fprintf('\n>> !!!! [%s] Rest voltage %.3f V below %.2f V -- cell being drained? Board STOPPED. !!!!\n\n', bd.label, V, p.V_discharge_floor - 0.1);
        bd.doneM = true; bd.faultPaused = true; bad = true; return
    end
end

function [bd, bad] = stepRcheck(bd, Vnow, Inow, what)
% v2g: step resistance at the 2nd sample after switching on.
    bad = false;
    if bd.counterM ~= 2 || isnan(bd.Vpre) || isnan(Vnow) || isnan(Inow) || abs(Inow) < 0.3, return; end
    R = abs(Vnow - bd.Vpre) / abs(Inow);
    fprintf('>> [%s | Cyc %d] %s step R = %.0f mOhm\n', bd.label, bd.cycleM, what, 1000*R);
    if R > bd.params.stepR_max
        try, writeline(bd.DP813A, ':OUTP:STAT OFF'); catch, end
        try, writeline(bd.DL3021, ':SOUR:INP:STAT OFF'); catch, end
        fprintf('\n>> !!!! [%s] %s step resistance %.0f mOhm > %.0f mOhm -- holder/contact problem. Board STOPPED. !!!!\n', ...
            bd.label, what, 1000*R, 1000*bd.params.stepR_max);
        fprintf('>>      Wipe the cell ends + holder, re-seat, run connectionTest, then resume.\n\n');
        bd.doneM = true; bd.faultPaused = true; bad = true;
    end
end

function [dt, bd] = takeDt(bd, nowT)
    dt = seconds(nowT - bd.dt_lastM);
    if isempty(dt) || isnan(dt) || dt <= 0, dt = 3; end
    if dt > 60
        fprintf('>> WARNING [%s] %.0f s gap between samples -- integrated as 60 s. (MATLAB/USB busy?)\n', bd.label, dt);
        dt = 60;
    end
    bd.dt_lastM = nowT;
end

function [bd, bad] = watchdog(bd, V, I, what)
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
            fprintf('\n>> !!!! [%s] %s: V=%.4f V and I=%.4f A identical for 20 samples (~1 min) -- reading FROZEN? Board STOPPED. !!!!\n', bd.label, what, V, I);
            fprintf('>>      Power-cycle that instrument, check with connectionTest, then resume this board.\n\n');
        end
        bd.doneM = true; bd.faultPaused = true; bad = true;
    end
end

function [bd, bad] = loadCheck(bd, Idl, Iset, what)
% The electronic load must actually sink current once it is switched on.
    bad = false;
    if bd.counterM > 3 && Idl < 0.5*Iset, bd.lowIcount = bd.lowIcount + 1; else, bd.lowIcount = 0; end
    if bd.lowIcount >= 10
        try, writeline(bd.DL3021, ':SOUR:INP:STAT OFF'); catch, end
        fprintf('\n>> !!!! [%s] %s: load current %.3f A instead of %.3f A for 30 s. Board STOPPED. !!!!\n', bd.label, what, Idl, Iset);
        fprintf('>>      Check the DL input (power) leads to the cell, and the DL Von setting (must be below the cell voltage).\n\n');
        bd.doneM = true; bd.faultPaused = true; bad = true;
    end
end

function bd = flushLog(bd, tag)
% Write this board's V/I/Q samples to CSV and clear them.
%   state: 0 charge, 1 rest after charge, 2 discharge, 3 rest after discharge, 4 rest before EIS (1/3/4 = v2e, I = 0)
%   columns: t_posix_s (local clock), state, V (V; charge = PSU reading offset-corrected, discharge = DL), I (A), Q_count (mAh), cycle
    if isempty(bd.tlog), return; end
    fn = sprintf('MainLog_%s_%s_%s.csv', bd.label, bd.cellID, tag);
    try
        T = array2table(bd.tlog, 'VariableNames', {'t_posix_s','state','V','I_A','Q_count_mAh','cycle'});
        T.time = datetime(T.t_posix_s, 'ConvertFrom', 'posixtime');
        writetable(T, fn);
    catch err
        fprintf('>> [%s] Warning: could not write %s (%s)\n', bd.label, fn, err.message);
    end
    bd.tlog = zeros(0,6);
end

function bd = enterPhase(bd, state, nowT)
    bd.stateM = state; bd.counterM = 0; bd.phaseStartM = nowT; bd.dt_lastM = nowT;
    bd.sameCount = 0; bd.nanCount = 0; bd.lowIcount = 0; bd.lastV = NaN; bd.lastI = NaN; bd.lowVcount = 0;
    bd.restSame = 0; bd.restLastV = NaN; bd.restIcount = 0;   % v2g
end

function bd = logPhase(bd, name, nowT)
    bd.phaseLogM = [bd.phaseLogM; {bd.cycleM, string(name), bd.phaseStartM, nowT}];
end

function s = elapsedS(bd, nowT)
    s = seconds(nowT - bd.phaseStartM);
end

function bd = logRest(bd, nowT, stateCode)
% v2e: LOG ONLY. One DL voltage reading per timer tick during rests (outputs are off).
% Never stops or changes the cycle: any read error is ignored.
    try
        V = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?'));
        if ~isnan(V)
            bd.tlog(end+1,:) = [posixtime(nowT), stateCode, V, 0, bd.QcurrentM, bd.cycleM];
        end
    catch
    end
end

function V = restVoltage(bd)
% Cell voltage with everything off (DL3021 sense), median of 3.
    v = nan(3,1);
    for k = 1:3, v(k) = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?')); pause(0.15); end
    V = median(v, 'omitnan');
end

function chirp()
    try, sound(sin(2*pi*880*(0:1/8000:0.15)), 8000); catch, beep; end
end

function beepN(n)
% v2f: n clear beeps (0.6 s tone, 0.4 s gap) in one non-blocking sound call.
    fs = 8000;
    tone = 0.8*sin(2*pi*1000*(0:1/fs:0.6));
    gap  = zeros(1, round(0.4*fs));
    w = [];
    for k = 1:n, w = [w, tone, gap]; end %#ok<AGROW>
    try, sound(w, fs); catch, for k = 1:n, beep; pause(0.5); end, end
end

function bd = stepOneBoardMain(bd)
    nowT = datetime('now');
    bd.counterM = bd.counterM + 1;
    p = bd.params;

    switch bd.stateM
        case -6 % ----- v2h PRE-CHARGE: 0.5C CC-CV to V_precharge (fresh start, cell stored low) -----
            if bd.counterM == 1
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                writeline(bd.DP813A, [':SOUR:CURR:LEV:IMM:AMPL ', num2str(p.I_prep_charge)]);
                writeline(bd.DP813A, [':SOUR:VOLT:LEV:IMM:AMPL ', num2str(p.V_precharge + bd.dpOff)]);
                writeline(bd.DP813A, ':OUTP:STAT ON'); pause(0.1);
                bd.dt_lastM = nowT; bd.Q_prep = 0; bd.phaseStartM = nowT; bd.lowIcount = 0;   % real start (after startDelay)
                fprintf('>> [%s] Pre-charge: 0.5C CC-CV to %.2f V (CV until I < %.0f mA).\n', bd.label, p.V_precharge, 1000*p.I_cv_cutoff);
            end
            Vdp = str2double(query(bd.DP813A, ':MEAS:VOLT:DC?')) - bd.dpOff; pause(0.08);
            Idp = str2double(query(bd.DP813A, ':MEAS:CURR:DC?'));
            if isnan(Vdp) || isnan(Idp), bd.nanCount = bd.nanCount + 1; else, bd.nanCount = 0; end
            if bd.nanCount >= 5
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                fprintf('\n>> !!!! [%s] pre-charge: readings unreadable 5 times in a row. Board STOPPED. !!!!\n\n', bd.label);
                bd.doneM = true; bd.faultPaused = true; return
            end
            if isnan(Vdp) || isnan(Idp), return; end
            [dtP, bd] = takeDt(bd, nowT);
            bd.Q_prep = bd.Q_prep + Idp*dtP/3.6;
            bd.tlog(end+1,:) = [posixtime(nowT), -6, Vdp, Idp, NaN, bd.cycleM];
            if mod(bd.counterM, 20) == 0
                fprintf('>> [%s | pre-charge] V=%.3fV I=%.3fA Q=%.1f mAh\n', bd.label, Vdp, Idp, bd.Q_prep);
            end
            if Vdp > p.V_precharge + 0.10
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                fprintf('\n>> !!!! [%s] pre-charge: V=%.3f V above %.2f V. Board STOPPED. !!!!\n\n', bd.label, Vdp, p.V_precharge + 0.10);
                bd.doneM = true; bd.faultPaused = true; return
            end
            % CV finished: at the voltage and current below 0.05C for 5 ticks (~15 s)
            if Vdp >= p.V_precharge - p.V_tol && Idp < p.I_cv_cutoff, bd.lowIcount = bd.lowIcount + 1; else, bd.lowIcount = 0; end
            tooLong = elapsedS(bd, nowT) > p.precharge_maxS;
            if bd.lowIcount >= 5 || tooLong
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                if tooLong
                    fprintf('>> WARNING [%s] pre-charge hit the %.0f-min limit (V=%.3f V, I=%.3f A) -- continuing anyway.\n', bd.label, p.precharge_maxS/60, Vdp, Idp);
                end
                fprintf('>> [%s] Pre-charge done: %.1f mAh in %.1f min. Rest %.0f min, then anchor.\n', bd.label, bd.Q_prep, elapsedS(bd, nowT)/60, p.precharge_rest/60);
                bd = logPhase(bd, "precharge_to_" + sprintf('%.2fV', p.V_precharge), nowT);
                bd = enterPhase(bd, -5, nowT);
            end

        case -5 % ----- v2h rest after pre-charge -----
            [bd, bad] = restGuard(bd, nowT, -5); if bad, return; end
            if elapsedS(bd, nowT) >= p.precharge_rest
                bd = logPhase(bd, "precharge_rest", nowT);
                bd = enterPhase(bd, -1, nowT);
            end

        case -1 % ----- ANCHOR: 1C discharge to 1.8 V (fresh start only) -----
            if bd.counterM == 1
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                writeline(bd.DL3021, ':SOUR:INP:STAT ON'); pause(0.3);
                bd.dt_lastM = nowT; bd.Q_prep = 0; bd.phaseStartM = nowT;   % real start (after startDelay)
                fprintf('>> [%s] Anchor: discharging 1C to %.2f V before cycle 1.\n', bd.label, p.V_prep_cutoff);
            end
            Vdl = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?')); pause(0.08);
            Idl = str2double(query(bd.DL3021, ':MEAS:CURR:DC?'));
            [bd, bad] = watchdog(bd, Vdl, Idl, 'anchor discharge'); if bad, return; end
            if isnan(Vdl) || isnan(Idl), return; end
            [dtP, bd] = takeDt(bd, nowT);
            bd.Q_prep = bd.Q_prep + Idl*dtP/3.6;
            if mod(bd.counterM, 20) == 0
                fprintf('>> [%s | anchor] DISCHARGE to %.2f V: V=%.3fV I=%.3fA Q=%.1f mAh\n', bd.label, p.V_prep_cutoff, Vdl, Idl, bd.Q_prep);
            end
            [bd, bad] = loadCheck(bd, Idl, p.I_discharge, 'anchor discharge'); if bad, return; end
            if Vdl <= p.V_prep_cutoff
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                fprintf('>> [%s] Anchor discharge done (%.1f mAh removed). Rest 30 min.\n', bd.label, bd.Q_prep);
                bd = logPhase(bd, "anchor_discharge", nowT);
                bd = enterPhase(bd, -2, nowT);
            end

        case -2 % ----- rest at 0% -----
            [bd, bad] = restGuard(bd, nowT, -2); if bad, return; end
            if elapsedS(bd, nowT) >= p.rest_time
                bd = logPhase(bd, "anchor_rest_0pct", nowT);
                bd = enterPhase(bd, -3, nowT);
            end

        case -3 % ----- 0.5C charge of exactly SOC_lower * C_measured -----
            if bd.counterM == 1
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                writeline(bd.DP813A, [':SOUR:CURR:LEV:IMM:AMPL ', num2str(p.I_prep_charge)]);
                writeline(bd.DP813A, [':SOUR:VOLT:LEV:IMM:AMPL ', num2str(p.V_chg_stop + bd.dpOff)]);
                writeline(bd.DP813A, ':OUTP:STAT ON'); pause(0.1);
                bd.dt_lastM = nowT; bd.QcurrentM = 0;
                fprintf('>> [%s] Anchor: charging 0.5C to SOC 20%% (%.1f mAh).\n', bd.label, p.Q_lower);
            end
            Vdp = str2double(query(bd.DP813A, ':MEAS:VOLT:DC?')) - bd.dpOff; pause(0.08);   % corrected to real cell voltage
            Idp = str2double(query(bd.DP813A, ':MEAS:CURR:DC?'));
            [bd, bad] = watchdog(bd, Vdp, Idp, 'anchor charge'); if bad, return; end
            if isnan(Vdp) || isnan(Idp), return; end
            [dtP, bd] = takeDt(bd, nowT);
            bd.QcurrentM = bd.QcurrentM + Idp*dtP/3.6;
            if mod(bd.counterM, 20) == 0
                fprintf('>> [%s | anchor] CHARGE to 20%%: V=%.3fV I=%.3fA Q=%.1f/%.1f mAh\n', bd.label, Vdp, Idp, bd.QcurrentM, p.Q_lower);
            end
            if bd.QcurrentM >= p.Q_lower
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                fprintf('>> [%s] Anchor done: cell at SOC 20%% (%.1f mAh). Rest 30 min, then cycle 1.\n', bd.label, bd.QcurrentM);
                bd = logPhase(bd, "anchor_charge_to_20pct", nowT);
                bd = enterPhase(bd, -4, nowT);
            elseif Vdp >= p.V_chg_stop - p.V_tol
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                fprintf('\n>> !!!! [%s] Reached %.2f V while charging only to 20%% SOC -- something is wrong. Board STOPPED. !!!!\n\n', bd.label, Vdp);
                bd.doneM = true; bd.faultPaused = true; return
            end

        case -4 % ----- rest at 20% -----
            [bd, bad] = restGuard(bd, nowT, -4); if bad, return; end
            if elapsedS(bd, nowT) >= p.rest_time
                bd = logPhase(bd, "anchor_rest_20pct", nowT);
                bd.Q_chg_this = 0; bd.Q_dis_this = 0;
                bd = enterPhase(bd, 0, nowT);
            end

        case 0 % ----- CHARGE toward Q_upper -----
            if bd.counterM == 1
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                bd.Vpre = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?'));   % v2g: rest V for step-R check
                writeline(bd.DP813A, [':SOUR:CURR:LEV:IMM:AMPL ', num2str(p.I_charge)]);
                writeline(bd.DP813A, [':SOUR:VOLT:LEV:IMM:AMPL ', num2str(p.V_chg_stop + bd.dpOff)]);
                writeline(bd.DP813A, ':OUTP:STAT ON'); pause(0.1);
                bd.dt_lastM = nowT; bd.cvEntered = false; bd.socCV = NaN;
                fprintf('>> [%s | Cyc %d/%d] CHARGE START %gC -> SOC %.0f%% (stop/CV at %.2f V)\n', bd.label, bd.cycleM, p.N_cycles, ...
                    p.chargeCrate, 100*p.SOC_upper, p.V_chg_stop);
            end
            Vdp = str2double(query(bd.DP813A, ':MEAS:VOLT:DC?')) - bd.dpOff; pause(0.08);   % corrected to real cell voltage
            Idp = str2double(query(bd.DP813A, ':MEAS:CURR:DC?'));
            [bd, bad] = watchdog(bd, Vdp, Idp, 'charge'); if bad, return; end
            [bd, bad] = stepRcheck(bd, Vdp, Idp, 'charge'); if bad, return; end
            if isnan(Vdp) || isnan(Idp), return; end
            [dtP, bd] = takeDt(bd, nowT);
            bd.QcurrentM  = bd.QcurrentM  + Idp*dtP/3.6;
            bd.Q_chg_this = bd.Q_chg_this + Idp*dtP/3.6;
            bd.tlog(end+1,:) = [posixtime(nowT), 0, Vdp, Idp, bd.QcurrentM, bd.cycleM];

            if mod(bd.counterM, 20) == 0
                fprintf('>> [%s | Cyc %d/%d] CHARGE %gC: V=%.3fV I=%.3fA Q=%.1f/%.1f mAh\n', ...
                    bd.label, bd.cycleM, p.N_cycles, p.chargeCrate, Vdp, Idp, bd.QcurrentM, p.Q_upper);
            end
            if bd.counterM <= 2 && Vdp >= p.V_chg_stop - p.V_tol
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                fprintf('\n>> !!!! [%s] SANITY CHECK FAILED at charge start (V=%.3f V). Likely NO CELL connected. !!!!\n\n', bd.label, Vdp);
                bd.doneM = true; bd.faultPaused = true; return
            end
            if Vdp > p.V_chg_stop + 0.05
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                fprintf('\n>> !!!! [%s] V=%.3f V above %.2f V during charge (overcharge?). Board STOPPED. !!!!\n\n', bd.label, Vdp, p.V_chg_stop + 0.05);
                bd.doneM = true; bd.faultPaused = true; return
            end
            if bd.counterM > 3 && Vdp < p.V_chg_stop - 0.05 && Idp < 0.5*p.I_charge
                bd.lowIcount = bd.lowIcount + 1;
            else
                bd.lowIcount = 0;
            end
            if bd.lowIcount >= 10
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                fprintf('\n>> !!!! [%s] Charge current %.3f A far below %.3f A for 30 s -- PSU tripped (OVP/OCP) or path open. Board STOPPED. !!!!\n\n', ...
                    bd.label, Idp, p.I_charge);
                bd.doneM = true; bd.faultPaused = true; return
            end

            % The PSU limit is V_chg_stop, so once the cell gets there the PSU
            % holds that voltage (CV) and the current tapers. Charging goes on
            % until Q_upper is reached -> every C-rate gets the SAME SOC window.
            % Only if the current falls below 0.05C first does it stop short ("V").
            atV = Vdp >= p.V_chg_stop - p.V_tol;
            if atV && ~bd.cvEntered
                bd.cvEntered = true; bd.socCV = 100*bd.QcurrentM/p.C_measured;
                fprintf('>> [%s | Cyc %d/%d] Reached %.2f V at SOC %.1f%% -> CV hold until SOC %.0f%%.\n', bd.label, bd.cycleM, ...
                    p.N_cycles, p.V_chg_stop, 100*bd.QcurrentM/p.C_measured, 100*p.SOC_upper);
            end
            reachedQ  = bd.QcurrentM >= p.Q_upper;
            cvTooLow  = atV && Idp < p.I_cv_cutoff && bd.counterM > 3;
            if reachedQ || cvTooLow
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                if reachedQ && bd.cvEntered, bd.chgTrip = "Q+CV";
                elseif reachedQ,             bd.chgTrip = "Q";
                else,                        bd.chgTrip = "V";
                end
                bd = logPhase(bd, "charge", nowT);
                fprintf('>> [%s | Cyc %d/%d] Charge complete (%s): Q=%.1f mAh = SOC %.1f%%, V=%.3f V. Resting.\n', bd.label, bd.cycleM, ...
                    p.N_cycles, bd.chgTrip, bd.QcurrentM, 100*bd.QcurrentM/p.C_measured, Vdp);
                if bd.chgTrip == "V"
                    fprintf('>>   NOTE [%s] CV current fell below %.0f mA BEFORE SOC %.0f%% -- logged in TripPhase.\n', bd.label, 1000*p.I_cv_cutoff, 100*p.SOC_upper);
                end
                bd = enterPhase(bd, 1, nowT);
            end

        case 1 % ----- rest after charge -----
            [bd, bad] = restGuard(bd, nowT, 1); if bad, return; end
            if elapsedS(bd, nowT) >= p.rest_time
                bd.Vrest_chg = restVoltage(bd);
                bd = logPhase(bd, "rest_after_charge", nowT);
                if ismember(bd.cycleM, p.EIS_cycles) && ~bd.eisDone
                    fprintf('>> [%s | Cyc %d/%d] EIS cycle: discharging 1C to 50%% SOC, then 60 min rest, then STOP.\n', bd.label, bd.cycleM, p.N_cycles);
                end
                bd = enterPhase(bd, 2, nowT);
            end

        case 2 % ----- DISCHARGE toward Q_lower (pauses at Q_EIS on EIS cycles) -----
            if bd.counterM == 1
                writeline(bd.DP813A, ':OUTP:STAT OFF');
                bd.Vpre = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?'));   % v2g: rest V for step-R check
                writeline(bd.DL3021, ':SOUR:INP:STAT ON'); pause(0.3);
                bd.dt_lastM = nowT;
            end
            Vdl = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?')); pause(0.08);
            Idl = str2double(query(bd.DL3021, ':MEAS:CURR:DC?'));
            [bd, bad] = watchdog(bd, Vdl, Idl, 'discharge'); if bad, return; end
            [bd, bad] = stepRcheck(bd, Vdl, Idl, 'discharge'); if bad, return; end
            if isnan(Vdl) || isnan(Idl), return; end
            [dtP, bd] = takeDt(bd, nowT);
            bd.QcurrentM  = bd.QcurrentM  - Idl*dtP/3.6;
            bd.tlog(end+1,:) = [posixtime(nowT), 2, Vdl, Idl, bd.QcurrentM, bd.cycleM];
            [bd, bad] = loadCheck(bd, Idl, p.I_discharge, 'discharge'); if bad, return; end
            bd.Q_dis_this = bd.Q_dis_this + Idl*dtP/3.6;

            if mod(bd.counterM, 20) == 0
                fprintf('>> [%s | Cyc %d/%d] DISCHARGE %gC: V=%.3fV I=%.3fA Q=%.1f/%.1f mAh\n', ...
                    bd.label, bd.cycleM, p.N_cycles, p.dischargeCrate, Vdl, Idl, bd.QcurrentM, p.Q_lower);
            end
            if Vdl <= p.V_discharge_floor && bd.counterM <= 2
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                fprintf('\n>> !!!! [%s] SANITY CHECK FAILED at discharge start (V=%.3f V). Likely NO CELL connected. !!!!\n\n', bd.label, Vdl);
                bd.doneM = true; bd.faultPaused = true; return
            end
            if Vdl < p.V_discharge_floor - 0.1
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                fprintf('\n>> !!!! [%s] V=%.3f V fell well below the %.2f V floor. Board STOPPED. !!!!\n\n', bd.label, Vdl, p.V_discharge_floor);
                bd.doneM = true; bd.faultPaused = true; return
            end

            stopNow = false;
            if ismember(bd.cycleM, p.EIS_cycles) && ~bd.eisDone && bd.QcurrentM <= p.Q_EIS
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                bd = logPhase(bd, "discharge_to_EIS_SOC", nowT);
                fprintf('>> [%s | Cyc %d/%d] At 50%% SOC (Q=%.1f mAh). Resting 60 min before EIS.\n', bd.label, bd.cycleM, p.N_cycles, bd.QcurrentM);
                if p.ambientC ~= 25
                    fprintf('>>   CHAMBER RUN: set the chamber to 25 C NOW so the cell is at 25 C for EIS.\n');
                end
                fprintf('>>   (1 beep = rest before EIS started; 2 beeps in 60 min = time to measure EIS)\n');
                beepN(1);
                bd = enterPhase(bd, 4, nowT);
            else
                useAnchor = ~isnan(p.V_anchor_low);
                if useAnchor && Vdl <= p.V_anchor_low, bd.lowVcount = bd.lowVcount + 1; else, bd.lowVcount = 0; end
                if useAnchor && bd.lowVcount >= 3
                    bd.anchorDQ = bd.QcurrentM - p.Q_lower;      % >0: cell was below the count (count drifted high)
                    bd.QcurrentM = p.Q_lower;                     % re-anchor the Coulomb count at SOC 20%
                    bd.disTrip = "Vanchor"; stopNow = true;
                    fprintf('>> [%s | Cyc %d/%d] Lower anchor %.3f V reached; count was %+.1f mAh off -> reset to SOC 20%%.\n', ...
                        bd.label, bd.cycleM, p.N_cycles, p.V_anchor_low, bd.anchorDQ);
                elseif ~useAnchor && bd.QcurrentM <= p.Q_lower
                    bd.disTrip = "Q"; stopNow = true;
                    p.V_anchor_low = Vdl; bd.params.V_anchor_low = Vdl;    % learn the anchor (count is exact right after the start anchor)
                    fprintf('>> [%s] Learnt lower anchor: %.3f V (1C loaded) at SOC 20%%.\n', bd.label, Vdl);
                elseif useAnchor && bd.QcurrentM <= p.Q_lower - p.Q_safety_below
                    bd.disTrip = "Qlimit"; stopNow = true; bd.anchorDQ = bd.QcurrentM - p.Q_lower;
                    fprintf('>> WARNING [%s] count is %.0f mAh below SOC 20%% but V=%.3f V is still above the anchor %.3f V. Stopped discharge.\n', ...
                        bd.label, p.Q_lower - bd.QcurrentM, Vdl, p.V_anchor_low);
                elseif Vdl <= p.V_discharge_floor
                    bd.disTrip = "V"; stopNow = true;
                end
            end
            if stopNow
                writeline(bd.DL3021, ':SOUR:INP:STAT OFF');
                bd = logPhase(bd, "discharge", nowT);
                fprintf('>> [%s | Cyc %d/%d] Discharge complete on %s (Q=%.1f mAh, V=%.3f V). Resting.\n', bd.label, bd.cycleM, ...
                    p.N_cycles, bd.disTrip, bd.QcurrentM, Vdl);
                if bd.disTrip == "V"
                    fprintf('>>   NOTE [%s] hit %.2f V before SOC 20%% -- the SOC count has drifted LOW; check the drift log.\n', bd.label, p.V_discharge_floor);
                end
                bd = enterPhase(bd, 3, nowT);
            end

        case 4 % ----- rest at 50% SOC, then hard stop for EIS -----
            [bd, bad] = restGuard(bd, nowT, 4); if bad, return; end
            if elapsedS(bd, nowT) >= p.EIS_rest_time
                bd.V_EIS = restVoltage(bd);
                bd = logPhase(bd, "rest_before_EIS", nowT);
                fprintf('>> [%s | Cyc %d/%d] >>> EIS + WEIGH + DIMENSION CHECKPOINT NOW (cell %s at 50%% SOC, %.3f V) <<<\n', ...
                    bd.label, bd.cycleM, p.N_cycles, bd.cellID, bd.V_EIS);
                fprintf('>>   1) EIS with Hioki BT4560   2) weigh   3) diameter (3 points) + length\n');
                if p.ambientC ~= 25
                    fprintf('>>   After EIS: set the chamber back to %g C, soak >= 1-2 h, THEN resume.\n', p.ambientC);
                end
                fprintf('>>   Resume with: mainCyclingMultiBoard_v2h(''%s'')\n', bd.label);
                beepN(2);
                bd.eisDone = true;
                bd = flushLog(bd, sprintf('cyc%02d_toEIS', bd.cycleM));
                bd = enterPhase(bd, 2, nowT);   % resume walks straight back into the discharge
                bd.doneM = true;
                saveCheckpoint(bd);
                return
            end

        case 3 % ----- rest after discharge: bookkeeping, drift check -----
            [bd, bad] = restGuard(bd, nowT, 3); if bad, return; end
            if elapsedS(bd, nowT) >= p.rest_time
                Vlow = restVoltage(bd);
                note = "";
                if isnan(bd.Vref_low)
                    bd.Vref_low = Vlow;
                elseif abs(Vlow - bd.Vref_low) > p.drift_warn_V
                    note = sprintf("SOC drift? rest V at 20%% moved %+.0f mV from cycle-1", 1000*(Vlow - bd.Vref_low));
                    fprintf('>> WARNING [%s] %s (%.3f -> %.3f V). Consider re-anchoring at the next EIS stop.\n', bd.label, note, bd.Vref_low, Vlow);
                end
                if bd.disTrip == "Vanchor" && ~isnan(bd.Vref_low) && bd.cycleM > 1
                    [bd, fbNote] = anchorFeedback(bd, Vlow, true);   % move anchor AND correct the count now (next phase is charge)
                    bd.params.anchorFB_lastCyc = bd.cycleM;
                    if strlength(fbNote) > 0
                        fprintf('>> [%s | Cyc %d] %s\n', bd.label, bd.cycleM, fbNote);
                        if strlength(note) > 0, note = note + " | " + fbNote; else, note = fbNote; end
                    end
                end
                CE = NaN; if bd.Q_chg_this > 0, CE = 100*bd.Q_dis_this/bd.Q_chg_this; end
                bd = logPhase(bd, "rest_after_discharge", nowT);
                bd.cycleSummaryM = [bd.cycleSummaryM; {bd.cycleM, bd.Q_chg_this, bd.Q_dis_this, CE, ...
                    "chg:" + bd.chgTrip + " dis:" + bd.disTrip, bd.socCV, bd.Vrest_chg, Vlow, bd.V_EIS, bd.anchorDQ, note}];
                fprintf('>> [%s | Cyc %d/%d] Cycle complete. Q_chg=%.1f  Q_dis=%.1f mAh  CE=%.2f%%  Vrest(20%%)=%.3f V\n', ...
                    bd.label, bd.cycleM, p.N_cycles, bd.Q_chg_this, bd.Q_dis_this, CE, Vlow);

                bd.cyclesDoneThisSession = bd.cyclesDoneThisSession + 1;
                bd = flushLog(bd, sprintf('cyc%02d', bd.cycleM));
                bd.Q_chg_this = 0; bd.Q_dis_this = 0; bd.eisDone = false;
                bd.chgTrip = ""; bd.disTrip = ""; bd.socCV = NaN; bd.Vrest_chg = NaN; bd.V_EIS = NaN; bd.anchorDQ = NaN;

                if bd.cycleM >= p.N_cycles
                    bd.cycleM = bd.cycleM + 1;
                    bd.doneM = true;
                    saveResultsMain(bd);
                    deleteCheckpointIfExists(bd.label);
                else
                    bd.cycleM = bd.cycleM + 1;
                    bd = enterPhase(bd, 0, nowT);
                    if bd.cyclesDoneThisSession >= bd.N_cycles_this_session, bd.doneM = true; end
                    saveCheckpoint(bd);
                end
            end
    end
end

function saveCheckpoint(bd)
    stateM = bd.stateM; cycleM = bd.cycleM; QcurrentM = bd.QcurrentM; %#ok<NASGU>
    phaseLogM = bd.phaseLogM; cycleSummaryM = bd.cycleSummaryM; params = bd.params; %#ok<NASGU>
    Q_chg_this = bd.Q_chg_this; Q_dis_this = bd.Q_dis_this; eisDone = bd.eisDone; %#ok<NASGU>
    Vref_low = bd.Vref_low; Vrest_chg = bd.Vrest_chg; V_EIS = bd.V_EIS; chgTrip = bd.chgTrip; socCV = bd.socCV; cellID = bd.cellID; %#ok<NASGU>
    save(sprintf('mainCyclingCheckpoint_%s.mat', bd.label), 'stateM','cycleM','QcurrentM','phaseLogM','cycleSummaryM', ...
        'params','Q_chg_this','Q_dis_this','eisDone','Vref_low','Vrest_chg','V_EIS','chgTrip','socCV','cellID');
    if bd.stateM == 2
        fprintf('>> [%s] Checkpoint saved: cycle %d/%d paused at 50%% SOC for EIS.\n', bd.label, bd.cycleM, bd.params.N_cycles);
    else
        fprintf('>> [%s] Checkpoint saved: completed cycle %d/%d.\n', bd.label, bd.cycleM-1, bd.params.N_cycles);
    end
end

function deleteCheckpointIfExists(label)
    f = sprintf('mainCyclingCheckpoint_%s.mat', label);
    if isfile(f), delete(f); end
end

function cleanupOneBoardMain(b)
    global boards
    bd = boards(b);
    nowT = datetime('now');
    try, flushLog(bd, sprintf('cyc%02d_interrupted_%s', bd.cycleM, datestr(nowT,'HHMM'))); catch, end
    try
        tag = sprintf('%s (%s)', bd.label, bd.cellID);
        if height(bd.phaseLogM) > 0
            writetable(bd.phaseLogM, sprintf('%s - %s - Interrupted MainCycling PhaseLog.xlsx', datestr(nowT,'yyyy.mm.dd-HH.MM'), tag));
        end
        if height(bd.cycleSummaryM) > 0
            writetable(bd.cycleSummaryM, sprintf('%s - %s - Interrupted MainCycling Summary.xlsx', datestr(nowT,'yyyy.mm.dd-HH.MM'), tag));
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

function errorOneBoardMain(b, event)
    global boards
    fprintf('>> [Board %s] Timer error: %s\n', boards(b).label, event.Data.message);
    cleanupOneBoardMain(b);
end

function saveResultsMain(bd)
    nowT = datetime('now');
    p = bd.params;
    tag = sprintf('%s (%s) - MainCycling %gC SOC%.0f-%.0f %gdegC', bd.label, bd.cellID, p.chargeCrate, p.SOC_lower*100, p.SOC_upper*100, p.ambientC);
    fname1 = sprintf('%s - %s Summary.xlsx', datestr(nowT,'yyyy.mm.dd-HH.MM'), tag);
    fname2 = sprintf('%s - %s PhaseLog.xlsx', datestr(nowT,'yyyy.mm.dd-HH.MM'), tag);
    writetable(bd.cycleSummaryM, fname1);
    writetable(bd.phaseLogM, fname2);
    fprintf('>> [%s] Saved: %s and %s\n', bd.label, fname1, fname2);
end

function closeOldHandles()
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

function tf = isVisaError(err)
% v2f: true for USB/VISA transport errors only (logic bugs are NOT swallowed).
    m = lower(err.message);
    tf = contains(m, 'visa') || contains(m, 'transport') || contains(m, 'error writing') || ...
         contains(m, 'error reading') || contains(m, 'timeout') || contains(m, 'invalid or deleted object') || ...
         contains(m, 'not connected');
end

function [bd, ok] = reconnectBoard(bd)
% v2f: drop the old VISA objects and reopen DP + DL with the same settings as at start.
    ok = false;
    try, delete(bd.DP813A); catch, end
    try, delete(bd.DL3021); catch, end
    try
        rig = bd.rigCfg; p = bd.params;
        bd.DP813A = visadev(rig.visaDP);
        setupPSURange(bd.DP813A, rig);
        writeline(bd.DP813A, ':OUTP:STAT OFF');
        writeline(bd.DP813A, [':SOUR:VOLT:PROT:LEV ', num2str(p.V_OVP + bd.dpOff)]);
        writeline(bd.DP813A, ':SOUR:VOLT:PROT:STAT ON');
        writeline(bd.DP813A, [':SOUR:CURR:PROT:LEV ', num2str(p.I_OCP)]);
        writeline(bd.DP813A, ':SOUR:CURR:PROT:STAT ON');
        bd.DL3021 = visadev(rig.visaDL);
        writeline(bd.DL3021, ':SOUR:INP:STAT OFF'); pause(0.1);
        writeline(bd.DL3021, ':SOUR:FUNC:MODE FIX'); pause(0.1);
        writeline(bd.DL3021, ':SOUR:FUNC CURR'); pause(0.1);
        writeline(bd.DL3021, [':SOUR:CURR:LEV:IMM ', num2str(p.I_discharge, '%.4f')]); pause(0.1);
        writeline(bd.DL3021, ':SOUR:CURR:SLEW 0.05'); pause(0.3);
        V0 = str2double(query(bd.DL3021, ':MEAS:VOLT:DC?'));
        if isnan(V0) || V0 < 1.8 || V0 > 4.05
            fprintf('>> [%s] Reconnected but cell reads %.3f V -- not continuing.\n', bd.label, V0);
            return
        end
        fprintf('>> [%s] Instruments back. Cell at %.3f V.\n', bd.label, V0);
        ok = true;
    catch e
        fprintf('>> [%s] reconnect failed: %s\n', bd.label, e.message);
    end
end

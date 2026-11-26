function dischargeOnly(boardLabel, Crate, Vcutoff)
%% DISCHARGE ONLY (constant current) for ONE board, no timers, no cycling.
%  dischargeOnly('A')            % 1C to 1.8 V
%  dischargeOnly('B', 1, 1.8)
%  Stop any time with Ctrl+C: the load input is switched off automatically.
    if nargin < 1, error('Usage: dischargeOnly(''A'')'); end
    if nargin < 2, Crate = 1; end
    if nargin < 3, Vcutoff = 1.8; end
    boardLabel = upper(char(boardLabel));
    C_Ah = 1.3; I = Crate*C_Ah; maxHours = 4; V_ceiling = 4.05;

    c = rigConfig(boardLabel);   % addresses for A / B / C live in rigConfig.m

    if ~isempty(timerfindall)
        error('Timers still exist. Run: stop(timerfindall); delete(timerfindall)');
    end
    closeOldHandles();

    dp = visadev(c.visaDP); dl = visadev(c.visaDL);
    global testLog
    testLog = zeros(0,6);   % t, V_DP(=0 when output off), I_DL, V_DL, Q_mAh, phase(unused)
    guard = onCleanup(@() finish(dp, dl, boardLabel, 'Discharge')); %#ok<NASGU>
    writeline(dp, ':OUTP:STAT OFF');
    writeline(dl, ':SOUR:INP:STAT OFF'); pause(0.1);
    writeline(dl, ':SOUR:FUNC:MODE FIX'); pause(0.1);
    writeline(dl, ':SOUR:FUNC CURR'); pause(0.1);
    writeline(dl, sprintf(':SOUR:CURR:LEV:IMM %.4f', I)); pause(0.1);
    writeline(dl, ':SOUR:CURR:SLEW 0.05'); pause(0.3);
    V0 = str2double(query(dl, ':MEAS:VOLT:DC?'));
    if ~(V0 > Vcutoff + 0.1 && V0 < V_ceiling)
        error('Rest voltage %.3f V not usable (cutoff %.2f V): no cell / bad sense / already empty.', V0, Vcutoff);
    end
    fprintf('>> [%s] Rest %.3f V. Discharge %.2f A (%.2fC) to %.2f V.\n', boardLabel, V0, I, Crate, Vcutoff);

    writeline(dl, ':SOUR:INP:STAT ON');
    t0 = tic; tPrev = 0; Q = 0; lastPrint = -30; low = 0;
    while true
        pause(0.5);
        t = toc(t0);
        Vdl = str2double(query(dl, ':MEAS:VOLT:DC?'));
        Idl = str2double(query(dl, ':MEAS:CURR:DC?'));
        Q = Q + Idl*(t - tPrev)/3.6;  tPrev = t;
        testLog(end+1,:) = [t 0 Idl Vdl Q 0]; %#ok<AGROW>
        if t - lastPrint >= 30
            fprintf('  t=%5.1f min  V=%.3f  I=%.3f A  Q=%.0f mAh\n', t/60, Vdl, Idl, Q);
            lastPrint = t;
        end
        if Vdl <= Vcutoff, low = low + 1; else, low = 0; end
        if low >= 3, fprintf('>> Cutoff %.2f V reached. Done.\n', Vcutoff); break; end
        if t > maxHours*3600, fprintf('>> Max time reached.\n'); break; end
    end
    writeline(dl, ':SOUR:INP:STAT OFF');
    k = find(testLog(:,3) > 0.95*I, 1);
    step = NaN; v60 = NaN;
    if ~isempty(k), step = V0 - testLog(k,4); end
    j = find(testLog(:,1) >= 60, 1); if ~isempty(j), v60 = testLog(j,4); end
    fprintf('\nRESULT: voltage step at load start = %.3f V (normal <= 0.10 V at 1.3 A, R_app = %.3f ohm)\n', step, step/I);
    fprintf('        V at 60 s = %.3f V (normal 3.79-3.84 V when starting from full)\n', v60);
    fprintf('        Q discharged = %.0f mAh, time = %.1f min\n', testLog(end,5), testLog(end,1)/60);
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
    catch, end
end

function finish(dp, dl, label, kind)
    global testLog
    try, writeline(dp, ':OUTP:STAT OFF'); catch, end
    try, writeline(dl, ':SOUR:INP:STAT OFF'); catch, end
    L = testLog;
    if ~isempty(L)
        T = array2table(L, 'VariableNames', {'t_s','V_DP','I_DL','V_DL','Q_mAh','Phase'});
        fn = sprintf('%s - %s - %sOnly.xlsx', datestr(now,'yyyy.mm.dd-HH.MM'), label, kind);
        try, writetable(T, fn); fprintf('>> Saved %s\n', fn); catch, end
    end
    try, delete(dp); catch, end
    try, delete(dl); catch, end
end

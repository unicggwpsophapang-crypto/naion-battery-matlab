function chargeOnly(boardLabel, Crate, Vtarget, cutoffCrate)
%% CHARGE ONLY (CC-CV) for ONE board, no timers, no cycling.
%  chargeOnly('A')                 % 0.5C to 4.0 V, CV cutoff 0.05C
%  chargeOnly('B', 1)              % 1C
%  chargeOnly('A', 0.5, 4.0, 0.05) % all arguments
%  chargeOnly('C')                 % board C (DP811 + DL3021)
%  Stop any time with Ctrl+C: the output is switched off automatically.
%  Logs every ~1 s, prints a line every 30 s, saves xlsx at the end (or when interrupted).
    if nargin < 1, error('Usage: chargeOnly(''A'')'); end
    if nargin < 2, Crate = 0.5; end
    if nargin < 3, Vtarget = 4.0; end
    if nargin < 4, cutoffCrate = 0.05; end
    boardLabel = upper(char(boardLabel));
    C_Ah = 1.3; I = Crate*C_Ah; Icut = cutoffCrate*C_Ah;
    maxHours = 6;  V_abort = 4.05;  V_ovp = 4.1;

    c = rigConfig(boardLabel);   % addresses for A / B / C live in rigConfig.m
    if Crate > 1 && Vtarget > 3.95
        error('Datasheet: charge above 1C only to 3.95 V (and only at 25-35 C). Use Vtarget <= 3.95.');
    end

    if ~isempty(timerfindall)
        error('Timers still exist. Run: stop(timerfindall); delete(timerfindall)');
    end
    closeOldHandles();

    dp = visadev(c.visaDP); dl = visadev(c.visaDL);
    setupPSURange(dp, c);
    off = 0; if isfield(c,'dpVoffset'), off = c.dpVoffset; end   % PSU reads this much above the real cell voltage (rigConfig)
    global testLog
    testLog = zeros(0,6);   % t, V_DP, I_DP, V_DL, Q_mAh, phase(1=CC,2=CV)
    guard = onCleanup(@() finish(dp, dl, boardLabel, 'Charge')); %#ok<NASGU>
    writeline(dl, ':SOUR:INP:STAT OFF');
    writeline(dp, ':OUTP:STAT OFF');
    writeline(dp, sprintf(':SOUR:VOLT:PROT:LEV %g', V_ovp + off));  writeline(dp, ':SOUR:VOLT:PROT:STAT ON');
    writeline(dp, sprintf(':SOUR:CURR:PROT:LEV %g', 1.3*I));  writeline(dp, ':SOUR:CURR:PROT:STAT ON');
    pause(0.3);
    V0 = str2double(query(dl, ':MEAS:VOLT:DC?'));
    if ~(V0 > 1.8 && V0 < 4.0)
        error('Rest voltage %.3f V not in 1.8-4.0 V: no cell / bad sense / already full.', V0);
    end
    fprintf('>> [%s] Rest %.3f V. Charge %.2f A (%.2fC) to %.2f V, CV cutoff %.0f mA.\n', boardLabel, V0, I, Crate, Vtarget, Icut*1000);

    writeline(dp, sprintf(':SOUR:CURR:LEV:IMM:AMPL %g', I));
    writeline(dp, sprintf(':SOUR:VOLT:LEV:IMM:AMPL %g', Vtarget + off));
    writeline(dp, ':OUTP:STAT ON');
    t0 = tic; tPrev = 0; Q = 0; lastPrint = -30; belowCut = 0;
    while true
        pause(0.5);
        t = toc(t0);
        Vdp = str2double(query(dp, ':MEAS:VOLT:DC?')) - off;
        Idp = str2double(query(dp, ':MEAS:CURR:DC?'));
        Vdl = str2double(query(dl, ':MEAS:VOLT:DC?'));
        Q = Q + Idp*(t - tPrev)/3.6;  tPrev = t;
        phase = 1 + (Vdp >= Vtarget - 0.01 && Idp < 0.98*I);
        testLog(end+1,:) = [t Vdp Idp Vdl Q phase]; %#ok<AGROW>
        if t - lastPrint >= 30
            fprintf('  t=%5.1f min  V_DP=%.3f  V_DL=%.3f  I=%.3f A  Q=%.0f mAh  %s\n', t/60, Vdp, Vdl, Idp, Q, ternary(phase==1,'CC','CV'));
            lastPrint = t;
        end
        if Vdp > V_abort || Vdl > V_abort, error('Aborted: voltage %.3f/%.3f V above %.2f V.', Vdp, Vdl, V_abort); end
        if phase == 2 && Idp < Icut, belowCut = belowCut + 1; else, belowCut = 0; end
        if belowCut >= 5, fprintf('>> CV cutoff reached (I<%.0f mA). Done.\n', Icut*1000); break; end
        if t > maxHours*3600, fprintf('>> Max time reached.\n'); break; end
    end
    jump = NaN;
    k = find(testLog(:,3) > 0.9*I, 1);
    if ~isempty(k) && k>1, jump = testLog(k,2) - V0; end
    ccEnd = find(testLog(:,6)==2, 1);
    fprintf('\nRESULT: charge jump at start = %.3f V (normal 0.10-0.30 V)\n', jump);
    if ~isempty(ccEnd), fprintf('        Q at CC->CV = %.0f mAh (normal ~1140-1310 mAh at 0.5C)\n', testLog(ccEnd,5)); end
    fprintf('        Q total = %.0f mAh, time = %.1f min, CV time = %.1f min\n', testLog(end,5), testLog(end,1)/60, (testLog(end,1)-ifelse(isempty(ccEnd),testLog(end,1),testLog(ccEnd,1)))/60);
end

function r = ternary(c,a,b), if c, r=a; else, r=b; end, end
function r = ifelse(c,a,b), if c, r=a; else, r=b; end, end

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
        T = array2table(L, 'VariableNames', {'t_s','V_DP','I_DP','V_DL','Q_mAh','Phase'});
        fn = sprintf('%s - %s - %sOnly.xlsx', datestr(now,'yyyy.mm.dd-HH.MM'), label, kind);
        try, writetable(T, fn); fprintf('>> Saved %s\n', fn); catch, end
    end
    try, delete(dp); catch, end
    try, delete(dl); catch, end
end

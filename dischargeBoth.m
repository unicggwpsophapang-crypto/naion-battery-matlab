function dischargeBoth(boards, Crate, Vcutoff)
%% DISCHARGE ONLY on one, two or all three boards (A/B/C) at the same time, each fully independent.
%  dischargeBoth()                 % A and B, 1C to 1.8 V
%  dischargeBoth('A')              % only A
%  dischargeBoth('B', 0.5, 2.0)    % only B, 0.5C, cutoff 2.0 V
%  dischargeBoth('AB', 1, 1.8)
%  dischargeBoth('ABC')            % all three boards
%
%  Each board has its OWN timer, OWN instruments, OWN log. When one board reaches
%  its cutoff (or errors) only that board stops; the other keeps going.
%  Stop manually (Ctrl+C does NOT stop timers):
%       stopDischarge        % both boards
%       stopDischarge('A')   % only A
%  Each board saves "<date> - <A|B|C> - DischargeOnly.xlsx" when it stops.
    if nargin < 1 || isempty(boards), boards = 'AB'; end
    if nargin < 2, Crate = 1; end
    if nargin < 3, Vcutoff = 1.8; end
    boards = upper(char(boards));
    labels = {};
    for k = 1:numel(boards)
        if ~ismember(boards(k), 'ABC'), error('Board must be A, B and/or C.'); end
        labels{end+1} = boards(k); %#ok<AGROW>
    end
    labels = unique(labels, 'stable');

    C_Ah = 1.3; I = Crate*C_Ah;
    if I > 2.0, error('Requested %.2f A is above the 2 A guard in this script.', I); end

    for k = 1:numel(labels), cfg.(labels{k}) = rigConfig(labels{k}); end   % addresses live in rigConfig.m

    % ---- timer pre-check: own stopped timers are cleaned, anything else running blocks ----
    tt = timerfindall;
    for k = 1:numel(tt)
        isOwn = ~isempty(regexp(tt(k).Name, '^disch[ABC]$', 'once'));
        if strcmp(tt(k).Running, 'on')
            error('Timer "%s" is still running. Run stopDischarge (own) or stop(timerfindall) first.', tt(k).Name);
        elseif isOwn
            delete(tt(k));
        end
    end
    tt = timerfindall;
    if ~isempty(tt)
        error('%d other timer(s) exist (old formation/cycling?). Run stop(timerfindall); delete(timerfindall) first.', numel(tt));
    end
    closeOldHandles();

    global DISCH
    DISCH = struct();

    % ---- set up each board (all set up before any timer starts) ----
    for k = 1:numel(labels)
        L = labels{k};
        S = struct('label', L, 'I', I, 'Vcutoff', Vcutoff, 'active', false, 'done', false);
        S.dl = visadev(cfg.(L).visaDL);
        S.dp = visadev(cfg.(L).visaDP);
        try, writeline(S.dp, ':OUTP:STAT OFF'); catch, end
        writeline(S.dl, ':SOUR:INP:STAT OFF'); pause(0.1);
        writeline(S.dl, ':SOUR:FUNC:MODE FIX'); pause(0.1);
        writeline(S.dl, ':SOUR:FUNC CURR'); pause(0.1);
        writeline(S.dl, sprintf(':SOUR:CURR:LEV:IMM %.4f', I)); pause(0.1);
        writeline(S.dl, ':SOUR:CURR:SLEW 0.05'); pause(0.3);
        S.V0 = str2double(query(S.dl, ':MEAS:VOLT:DC?'));
        if ~(S.V0 > Vcutoff + 0.1 && S.V0 < 4.05)
            cleanupHandles(S);
            abortAll(DISCH);
            error('Board %s: rest voltage %.3f V not usable (cutoff %.2f V). Nothing was started.', L, S.V0, Vcutoff);
        end
        S.log = zeros(0,4);   % t, V_DL, I_DL, Q_mAh
        S.Q = 0; S.tPrev = 0; S.lastPrint = -30; S.low = 0;
        S.tmr = timer('Name', ['disch' L], 'ExecutionMode', 'fixedSpacing', 'Period', 1, ...
                      'BusyMode', 'drop', ...
                      'TimerFcn', @(~,~) stepOne(L), ...
                      'StopFcn',  @(~,~) cleanupOne(L, 'stopped'), ...
                      'ErrorFcn', @(~,e) cleanupOne(L, ['timer error: ' e.Data.message]));
        DISCH.(L) = S;
        fprintf('>> [%s] rest %.3f V. Discharge %.2f A (%.2fC) to %.2f V.\n', L, S.V0, I, Crate, Vcutoff);
    end

    % ---- load ON and start timers ----
    for k = 1:numel(labels)
        L = labels{k};
        writeline(DISCH.(L).dl, ':SOUR:INP:STAT ON');
        DISCH.(L).t0 = tic;
        DISCH.(L).active = true;
        start(DISCH.(L).tmr);
    end
    fprintf('>> Running. Stop with stopDischarge (or stopDischarge(''A'') / (''B'')).\n');
end

function stepOne(L)
    global DISCH
    S = DISCH.(L);
    if ~S.active, return; end
    try
        t = toc(S.t0);
        V = str2double(query(S.dl, ':MEAS:VOLT:DC?'));
        Ia = str2double(query(S.dl, ':MEAS:CURR:DC?'));
        S.Q = S.Q + Ia*(t - S.tPrev)/3.6;  S.tPrev = t;
        S.log(end+1,:) = [t V Ia S.Q];
        if t - S.lastPrint >= 30
            fprintf('[%s] t=%5.1f min  V=%.3f  I=%.3f A  Q=%.0f mAh\n', L, t/60, V, Ia, S.Q);
            S.lastPrint = t;
        end
        if V <= S.Vcutoff, S.low = S.low + 1; else, S.low = 0; end
        stopNow = '';
        if S.low >= 3, stopNow = 'cutoff reached'; end
        if t > 4*3600, stopNow = 'max time'; end
        if V > 4.05,   stopNow = 'voltage above 4.05 V'; end
        DISCH.(L) = S;
        if ~isempty(stopNow)
            DISCH.(L).reason = stopNow;
            stop(S.tmr);
        end
    catch err
        DISCH.(L) = S;
        DISCH.(L).reason = ['error: ' err.message];
        stop(S.tmr);
    end
end

function cleanupOne(L, defaultReason)
    global DISCH
    if ~isfield(DISCH, L), return; end
    S = DISCH.(L);
    if S.done, return; end
    try, writeline(S.dl, ':SOUR:INP:STAT OFF'); catch, end
    try, writeline(S.dp, ':OUTP:STAT OFF'); catch, end
    S.done = true; S.active = false;
    reason = defaultReason;
    if isfield(S, 'reason') && ~isempty(S.reason), reason = S.reason; end
    if ~isempty(S.log)
        Lg = S.log;
        k = find(Lg(:,3) > 0.95*S.I, 1);
        step = NaN; v60 = NaN;
        if ~isempty(k), step = S.V0 - Lg(k,2); end
        j = find(Lg(:,1) >= 60, 1); if ~isempty(j), v60 = Lg(j,2); end
        fprintf('\n[%s] STOPPED (%s). Q=%.0f mAh, %.1f min, last V=%.3f V\n', L, reason, Lg(end,4), Lg(end,1)/60, Lg(end,2));
        fprintf('[%s] voltage step at load start = %.3f V (R_app = %.3f ohm); V at 60 s = %.3f V\n\n', L, step, step/S.I, v60);
        T = array2table(Lg, 'VariableNames', {'t_s','V_DL','I_DL','Q_mAh'});
        fn = sprintf('%s - %s - DischargeOnly.xlsx', datestr(now,'yyyy.mm.dd-HH.MM'), L);
        try, writetable(T, fn); fprintf('[%s] saved %s\n', L, fn); catch, end
    else
        fprintf('\n[%s] STOPPED (%s) with no samples.\n', L, reason);
    end
    cleanupHandles(S);
    DISCH.(L) = S;
end

function cleanupHandles(S)
    try, writeline(S.dl, ':SOUR:INP:STAT OFF'); catch, end
    try, writeline(S.dp, ':OUTP:STAT OFF'); catch, end
    try, delete(S.dl); catch, end
    try, delete(S.dp); catch, end
end

function abortAll(D)
    f = fieldnames(D);
    for k = 1:numel(f)
        S = D.(f{k});
        cleanupHandles(S);
        try, delete(S.tmr); catch, end
    end
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

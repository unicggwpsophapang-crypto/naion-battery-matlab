function fixCheckpointAfterEIS(label, V_EIS)
% Run 5 cycle 1: board was stopped during the 60-min rest at 50% SOC (state 4),
% EIS was then measured by hand. Mark the EIS as done so resume goes straight
% into the discharge 50% -> 20% of the SAME cycle (no second 60-min rest).
%   fixCheckpointAfterEIS('B', 3.078)
    f = sprintf('mainCyclingCheckpoint_%s.mat', label);
    if ~isfile(f), error('%s not found in %s', f, pwd); end
    S = load(f);
    fprintf('OLD: cell %s, cycle %d, state %d, Q=%.1f mAh (SOC %.1f%%), eisDone=%d\n', ...
        S.cellID, S.cycleM, S.stateM, S.QcurrentM, 100*S.QcurrentM/S.params.C_measured, S.eisDone);
    if S.cycleM ~= 1 || S.stateM ~= 4
        error('Expected cycle 1 / state 4 (rest before EIS). Nothing changed -- send this line to Claude.');
    end
    soc = 100*S.QcurrentM/S.params.C_measured;
    if abs(soc - 50) > 2, error('SOC %.1f%% is not ~50%%. Nothing changed.', soc); end
    copyfile(f, sprintf('mainCyclingCheckpoint_%s_beforeFix.mat', label));
    S.stateM = 2; S.eisDone = true; S.V_EIS = V_EIS;
    save(f, '-struct', 'S');
    fprintf('NEW: cycle %d, state 2 (discharge to 20%%), eisDone=1, V_EIS=%.3f V. Backup saved.\n', S.cycleM, V_EIS);
    fprintf('Now: connectionTestOne(''%s'') then mainCyclingMultiBoard_v2g(''%s'')\n', label, label);
end

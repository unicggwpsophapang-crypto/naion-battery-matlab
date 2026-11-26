function setupPSURange(dp, rig)
%% SETUPPSURANGE  Select the output range on PSUs that have more than one (DP811).
%  Call right after visadev(), while the output is OFF. Does nothing for
%  boards whose rigConfig dpRange is empty (DP813A).
%  A failure here is reported but not fatal: both DP811 ranges (20 V/10 A,
%  40 V/5 A) can deliver the 3.9 A needed for 3C, P20V just leaves more headroom.
    if ~isfield(rig, 'dpRange') || isempty(rig.dpRange), return; end
    try
        writeline(dp, ':OUTP:STAT OFF'); pause(0.1);
        writeline(dp, sprintf(':OUTP:RANG %s', rig.dpRange)); pause(0.3);
        got = strtrim(char(query(dp, ':OUTP:RANG?')));
        if contains(upper(got), upper(rig.dpRange)) || (strcmpi(rig.dpRange,'P20V') && contains(upper(got),'LOW'))
            fprintf('>> [Board %s] %s range set to %s.\n', rig.label, rig.dpModel, got);
        else
            fprintf('>> WARNING [Board %s] asked %s for range %s but it reports "%s". Check the front panel.\n', ...
                rig.label, rig.dpModel, rig.dpRange, got);
        end
    catch err
        fprintf('>> WARNING [Board %s] could not set %s range (%s). Set it on the front panel to %s.\n', ...
            rig.label, rig.dpModel, err.message, rig.dpRange);
    end
end

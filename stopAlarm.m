function stopAlarm()
%% Silence the v2g fault alarm (does not touch the cycling timers).
    tt = timerfindall('Tag','NaIonAlarm');
    if isempty(tt), fprintf('>> No alarm running.\n'); return; end
    stop(tt); delete(tt);
    fprintf('>> Alarm stopped.\n');
end

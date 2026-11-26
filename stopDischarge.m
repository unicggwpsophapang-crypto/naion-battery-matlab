function stopDischarge(label)
%% Stop dischargeBoth. Each board switches its load OFF and saves its own xlsx.
%  stopDischarge        % every board
%  stopDischarge('A')   % only A (B keeps running)
    if nargin < 1 || isempty(label)
        tt = timerfind('-regexp', 'Name', '^disch[ABC]$');
    else
        tt = timerfind('Name', ['disch' upper(char(label))]);
    end
    if isempty(tt), fprintf('No discharge timer found.\n'); return; end
    for k = 1:numel(tt)
        if strcmp(tt(k).Running, 'on'), stop(tt(k)); end
    end
end

function delayedResume()
% Called by a timer: resume boards A/B/C of the current run and log the result.
    fid = fopen('delayedResume_log.txt','a');
    try
        fprintf(fid, '%s start\n', datestr(now));
        mainCyclingMultiBoard_v2g('ABC');
        fprintf(fid, '%s started OK\n', datestr(now));
    catch e
        fprintf(fid, '%s FAILED: %s\n', datestr(now), e.message);
        for k = 1:10, beep; pause(0.5); end
    end
    fclose(fid);
end

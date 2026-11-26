function recoveryChargeMulti(boards)
% recoveryChargeMulti  Low-current recovery charge on several boards at once.
%   Each board charges at 65 mA until its cell reaches 2.00 V (read by its
%   DL3021A); boards stop independently. Any fault stops only that board.
%
%   boards = struct array with fields: name, psuAddr, dlAddr
%   Example:
%     b(1) = struct("name","N05","psuAddr","<DP813A addr board A>","dlAddr","<DL3021A addr board A>");
%     b(2) = struct("name","N06","psuAddr","<DP813A addr board B>","dlAddr","<DL3021A addr board B>");
%     recoveryChargeMulti(b)

I_chg = 0.065; V_target = 2.00; V_psuMax = 2.60; V_minStart = 1.20;
T_max_min = 90; T_check = 10; dt = 5;   % 90 min: N05/N06 needed >43 mAh (40 min was too short)

n = numel(boards);
for k = 1:n
    boards(k).psu = visadev(boards(k).psuAddr);
    boards(k).dl  = visadev(boards(k).dlAddr);
end
cleanupObj = onCleanup(@() allOff(boards));

active = true(1,n);
for k = 1:n
    b = boards(k);
    writeline(b.dl, ":SOUR:INP:STAT OFF");
    V0 = str2double(writeread(b.dl, ":MEAS:VOLT?"));
    boards(k).V0 = V0; boards(k).lastV = NaN; boards(k).same = 0; boards(k).log = [];
    fprintf("[%s] Start voltage: %.4f V\n", b.name, V0);
    if isnan(V0) || V0 < V_minStart
        fprintf("[%s] Start voltage too low or unreadable. Skipped.\n", b.name); active(k) = false; continue
    end
    if V0 >= V_target
        fprintf("[%s] Already >= %.2f V. Skipped.\n", b.name, V_target); active(k) = false; continue
    end
    writeline(b.psu, sprintf(":SOUR1:VOLT %.3f", V_psuMax));
    writeline(b.psu, sprintf(":SOUR1:CURR %.3f", I_chg));
    writeline(b.psu, ":OUTP CH1,ON");
end

t0 = tic;
while any(active)
    pause(dt);
    tmin = toc(t0)/60;
    for k = find(active)
        b = boards(k);
        V = str2double(writeread(b.dl, ":MEAS:VOLT?"));
        I = str2double(writeread(b.psu, ":MEAS:CURR? CH1"));
        boards(k).log(end+1,:) = [tmin, V, I];
        fprintf("[%s] t=%5.1f min  V_cell=%.4f V  I=%.4f A\n", b.name, tmin, V, I);

        if V == b.lastV, boards(k).same = b.same + 1; else, boards(k).same = 0; end
        boards(k).lastV = V;

        reason = "";
        if V >= V_target
            reason = sprintf("reached %.3f V (OK)", V);
        elseif boards(k).same >= 12
            reason = "FAULT: reading unchanged for 1 min (DL3021A frozen?)";
        elseif tmin >= T_check && V - b.V0 < 0.020
            reason = sprintf("FAULT: rose only %.0f mV in %d min (possible short)", (V-b.V0)*1000, T_check);
        elseif I < 0.5*I_chg
            reason = sprintf("FAULT: current %.3f A below setpoint (check charge path)", I);
        elseif tmin >= T_max_min
            reason = sprintf("FAULT: timeout after %d min", T_max_min);
        end
        if reason ~= ""
            try, writeline(b.psu, ":OUTP CH1,OFF"); catch, end
            active(k) = false;
            Q = trapz(boards(k).log(:,1)*60, boards(k).log(:,3))/3.6;
            fprintf("[%s] STOP: %s. Charge added %.1f mAh.\n", b.name, reason, Q);
            L = boards(k).log;
            writematrix([["t_min","V_cell","I_A"]; string(L)], b.name + "_recoveryCharge.csv");
        end
    end
end
fprintf("All boards done. Rest cells 1 h disconnected, then re-measure.\n");
end

function allOff(boards)
for k = 1:numel(boards)
    try, writeline(boards(k).psu, ":OUTP CH1,OFF"); catch, end
    try, writeline(boards(k).dl,  ":SOUR:INP:STAT OFF"); catch, end
end
end

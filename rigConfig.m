function rig = rigConfig(label)
%% RIGCONFIG  One place for every board's instrument addresses.
%  rig = rigConfig('A')   -> struct for board A (errors if not filled in)
%  R   = rigConfig()      -> all boards
%
%  Every script (formation, main cycling, connectionTest, chargeOnly,
%  dischargeOnly, dischargeBoth) reads its VISA strings from here, so a
%  new board is added in ONE place only.
%
%  Board C: DP811 + DL3021. Run `visadevlist` and paste the two resource
%  strings below in place of the XXX placeholders.
%
%  dpRange: '' for DP813A (single range in our use), 'P20V' for DP811
%  (20 V / 10 A range, enough headroom for 3C = 3.9 A).
%
%  dpVoffset: how much HIGHER the PSU's own voltage reading is than the
%  real cell voltage (V_DP - V_DL from connectionTest, same cell on every
%  board; the three DLs agree within 3 mV). The scripts add it to the PSU
%  voltage setpoint and subtract it from the PSU reading, so 4.00 V / 3.95 V
%  mean the same real cell voltage on every board.
%  Measured 2026-10-30 with cell C01_A at ~2.8 V: A +51 mV, B -9 mV, C +4 mV.
%  Re-measure near 3.9 V (connectionTest on a charged cell) and update.

    R.A = struct('visaDP',  'USB0::0x1AB1::0x0E11::DP8J275200187::0::INSTR', ...
                 'visaDL',  'USB0::0x1AB1::0x0E11::DL3A275100868::0::INSTR', ...
                 'dpModel', 'DP813A', 'dpRange', '', 'dpVoffset', 0);
    R.B = struct('visaDP',  'USB0::0x1AB1::0x0E11::DP8J275200184::0::INSTR', ...
                 'visaDL',  'USB0::0x1AB1::0x0E11::DL3A275100876::0::INSTR', ...
                 'dpModel', 'DP813A', 'dpRange', '', 'dpVoffset', -0.030);
    R.C = struct('visaDP',  'USB0::0x1AB1::0x0E11::DP8H220800012::0::INSTR', ...   % <-- EDIT: DP811 from visadevlist
                 'visaDL',  'USB0::0x1AB1::0x0E11::DL3C243000152::0::INSTR', ...   % <-- EDIT: DL3021 from visadevlist
                 'dpModel', 'DP811', 'dpRange', 'P20V', 'dpVoffset', 0);

    if nargin == 0
        rig = R;
        return
    end
    label = upper(char(label));
    if ~isfield(R, label)
        error('rigConfig: unknown board "%s". Known boards: %s', label, strjoin(fieldnames(R)', ', '));
    end
    rig = R.(label);
    rig.label = label;
    if contains(rig.visaDP, 'XXX') || contains(rig.visaDL, 'XXX')
        error(['rigConfig: board %s still has XXX in its VISA strings. Run visadevlist and ' ...
               'paste the DP/DL resource strings into rigConfig.m.'], label);
    end
end

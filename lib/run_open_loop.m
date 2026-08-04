function [SOC_trace, Vs_trace] = run_open_loop(I_arr, dt_arr, rc)
%RUN_OPEN_LOOP  Open-loop 2-RC Thevenin cell model (per-cell SOC + voltage).
%   Coulomb-counts SOC and integrates two RC branches. rc must carry the
%   HPPC lookup tables, capacity Q, and initial charge SOC0.
%
%   TABLE INDEXING: the HPPC tables are indexed by SOC (index 1 = empty,
%   index 11 = full) -- see the battery block in params_cfr26. This used to look
%   them up at 1-SOC, i.e. backwards, while reading OCV at SOC from the SAME grid.
%   Fixed. Checked against the July 11 pack-voltage trace: on the points where the
%   R/C tables actually do work (top decile of |I_cell|) reading them at SOC beats
%   reading them at 1-SOC by 6.3% RMSE, and by 9.5% on the top 1%. Coulomb-counted
%   SOC does not touch these tables, so the SOC numbers themselves did not move.

    % Trailing-zero guard: every R/C table ends in a hard 0 at SOC=1 (an HPPC fit
    % artifact -- see params_cfr26). Drop that last grid point and clamp, so we can
    % never interpolate toward a zero resistance / zero capacitance. Data untouched.
    nValid   = numel(rc.SOC_lookupR) - 1;
    socGrid  = rc.SOC_lookupR(1:nValid);
    lookupRC = @(tbl, soc) interp1(socGrid, tbl(1:nValid), ...
                                   min(max(soc, socGrid(1)), socGrid(end)), 'linear');

    Nk = length(I_arr);
    SOC_trace = zeros(Nk,1); Vs_trace = zeros(Nk,1);
    SOC = rc.SOC0; Vrc1 = 0; Vrc2 = 0;
    for k = 1:Nk
        % Pick the charging or discharging table set -- cells aren't symmetric.
        if I_arr(k) < 0   % charging branch
            Ri = lookupRC(rc.Ri_c, SOC);  R1 = lookupRC(rc.R1_c, SOC);
            R2 = lookupRC(rc.R2_c, SOC);  C1 = lookupRC(rc.C1_c, SOC);
            C2 = lookupRC(rc.C2_c, SOC);
        else              % discharging branch
            Ri = lookupRC(rc.Ri_d, SOC);  R1 = lookupRC(rc.R1_d, SOC);
            R2 = lookupRC(rc.R2_d, SOC);  C1 = lookupRC(rc.C1_d, SOC);
            C2 = lookupRC(rc.C2_d, SOC);
        end
        % Coulomb counting: charge in/out over capacity. This is what sets SOC.
        SOC = SOC - (I_arr(k)*dt_arr(k)) / rc.Q;
        SOC = min(max(SOC, 0), 1.2);
        % Resting voltage at this SOC (OCV table has no trailing-zero problem).
        VOC = interp1(rc.SOC_lookupR, rc.OCV_lookup, SOC, 'linear', 'extrap');
        % Both RC branches relax exponentially toward I*R over this timestep.
        tao1 = R1*C1; tao2 = R2*C2;
        Vrc1 = exp(-dt_arr(k)/tao1)*Vrc1 + R1*(1-exp(-dt_arr(k)/tao1))*I_arr(k);
        Vrc2 = exp(-dt_arr(k)/tao2)*Vrc2 + R2*(1-exp(-dt_arr(k)/tao2))*I_arr(k);
        % Terminal voltage = resting voltage minus instant IR drop minus both sags.
        SOC_trace(k) = SOC;
        Vs_trace(k)  = VOC - I_arr(k)*Ri - Vrc1 - Vrc2;
    end
end

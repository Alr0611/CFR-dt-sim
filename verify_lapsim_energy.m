%% VERIFY_LAPSIM_ENERGY  --  independent MATLAB rebuild of the CFR27 energy column
% Rebuilds the 49-cell (gear ratio x halfshaft angle) drivetrain-efficiency matrix
% used by LapSim_Research/config_matrix.csv, using ONLY this repo's own functions
% (emrax208_efficiency.m, params_cfr26.m). Nothing is imported from the Python.
% Purpose: catch any drift between the Python study and the MATLAB source of truth.
clear; clc;
cd(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'lib'));
p = params_cfr26();

RATIOS = [4.00 4.20 4.40 4.61 4.80 5.00 5.20];
ANGLES = [0 1 2 3 4 5 12];
CORNER_FRAC = 0.276;                 % = 1 - p.hs_frac_straight (0.724)

etaHs = @(b) 1 - 2*p.hs_kloss*sind(b);
MECH  = 0.98*0.95*0.97*0.92;         % spur x bearings x chain x diff
etaDt = @(a,cf) MECH * ((1-cf)*etaHs(a) + cf*etaHs(a + p.hs_corner_deg));

fprintf('=== CONSTANTS ===\n');
fprintf('  1 - hs_frac_straight            = %.4f  (CORNER_FRAC used: %.4f)\n', ...
        1-p.hs_frac_straight, CORNER_FRAC);
fprintf('  MECH stack                      = %.7f\n', MECH);
fprintf('  eta_dt(12 deg)                  = %.6f   (params %.4f)\n', etaDt(12,CORNER_FRAC), p.eta_drivetrain);
fprintf('  eta_dt( 5 deg)                  = %.6f\n', etaDt(5,CORNER_FRAC));
fprintf('  eta_dt( 0 deg)                  = %.6f\n', etaDt(0,CORNER_FRAC));

%% ---- duty cycle: the measured July 11 endurance run --------------------------
D    = readtable('data/endurance_july11_with_odo_wide.csv');
t    = D.t_s;
rpmM = D.PM100DX_motorSpeed;
tqM  = D.PM100DX_torqueFeedback;
if sum(tqM.*rpmM,'omitnan') < 0, tqM = -tqM; end     % repo sign normalisation
dt   = [diff(t); median(diff(t))];
Psh  = tqM .* rpmM * (2*pi/60);
keep = isfinite(Psh) & Psh > 1000;                   % repo 1 kW motoring gate
rpmM = rpmM(keep); Psh = Psh(keep); dt = dt(keep);
fprintf('\n  duty cycle: %d motoring points, %.3f kWh at the shaft\n', ...
        numel(rpmM), sum(Psh.*dt)/3.6e6);

%% ---- the sweep --------------------------------------------------------------
Ebatt = zeros(numel(RATIOS), numel(ANGLES));
Emot  = zeros(numel(RATIOS), numel(ANGLES));
Pwheel = Psh * 0.794;                                 % held FIXED (ratio-invariant)
for i = 1:numel(RATIOS)
    for j = 1:numel(ANGLES)
        Pshaft = Pwheel / etaDt(ANGLES(j), CORNER_FRAC);
        rpm    = abs(rpmM) * (RATIOS(i)/p.gear_current);
        w      = max(rpm*(2*pi/60), 1e-3);
        tq     = Pshaft ./ w;
        eff    = emrax208_efficiency(rpm, abs(tq), p);   % REPO's own function
        Ebatt(i,j) = sum(Pshaft./eff .* dt)/3600;
        Emot(i,j)  = sum(Pshaft.*dt) / sum(Pshaft./eff .* dt);
    end
end
base = Ebatt(RATIOS==4.61, ANGLES==12);
fprintf('  baseline 4.61 @ 12 deg = %.1f Wh\n\n', base);

fprintf('=== ENERGY DELTA %% vs 4.61 @ 12 deg (MATLAB, repo functions) ===\n');
fprintf('%7s', 'ratio'); fprintf('%11d deg', ANGLES); fprintf('\n');
for i = 1:numel(RATIOS)
    fprintf('%7.2f', RATIOS(i));
    fprintf('%14.3f%%', 100*(Ebatt(i,:)-base)/base);
    fprintf('\n');
end

fprintf('\n=== eta_motor (E_mech/E_elec) ===\n');
fprintf('%7s', 'ratio'); fprintf('%11d deg', ANGLES); fprintf('\n');
for i = 1:numel(RATIOS)
    fprintf('%7.2f', RATIOS(i)); fprintf('%14.4f', Emot(i,:)); fprintf('\n');
end

%% ---- write for the Python-side comparison -----------------------------------
fid = fopen('output/lapsim_energy_matlab.csv','w');
fprintf(fid,'gear_ratio,halfshaft_deg,eta_drivetrain,eta_motor,energy_pct\n');
for i = 1:numel(RATIOS)
    for j = 1:numel(ANGLES)
        fprintf(fid,'%.2f,%d,%.4f,%.4f,%.3f\n', RATIOS(i), ANGLES(j), ...
            etaDt(ANGLES(j),CORNER_FRAC), Emot(i,j), 100*(Ebatt(i,j)-base)/base);
    end
end
fclose(fid);
fprintf('\nWrote output/lapsim_energy_matlab.csv\n');

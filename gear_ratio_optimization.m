%% CFR27 GEAR RATIO OPTIMIZATION  --
%
% Long story short
%   We drove the car. We recorded everything. Now the question is: what if we'd
%   been geared differently? This tries every ratio from 4.00 to 5.20 on the
%   SAME laps we actually drove, and reports what each one would've cost us.
%
% The trick
%   Wheel speed doesn't care about gearing. The car went as fast as it went.
%   So we keep the real wheel speed, redo the motor math at each ratio, and
%   watch how much energy we'd have burned.
%
% Why does this code use 2 different runs?
%   Efficiency stuff  -> comp June 20. real comp track, better numbers.
%   Pack charge stuff -> July 11 test. Why not comp? Because comp DNF'd
%       (cell too hot), so it never reached the finish. You can't measure
%       "charge left at the end" on a run that didn't have an end.
%
% Rules
%   Numbers live in params_cfr26.m. Physics lives in lib/. Please don't put
%   either one in here.
%
% The accel column below is a quick-and-dirty estimate, good enough for
% comparing ratios. The real accel study (the one that counts spinning parts)
% is accel_model.m -- go there if accel is what you care about.

clear; clc; close all;
cd(fileparts(mfilename('fullpath')));   % run from the repo root so data/ and output/ paths resolve
if ~exist('output','dir'), mkdir('output'); end   % fresh copy may have no output/ folder
addpath(fullfile(fileparts(mfilename('fullpath')), 'lib'));  % works wherever MATLAB is pointed
p = params_cfr26();

%% ---- LOAD DATA ----
% Pull both telemetry runs in and put every channel into the units the rest of the
% file expects: per-CELL voltage and current, motor rpm/torque, mean wheel speed.
% Two runs on purpose -- see the header for why efficiency and SOC come from different
% sessions. The sign checks below fix logs that record motoring torque negative.
enduranceData        = readtable('data/endurance_july11_with_odo_wide.csv');   % July 11 (SOC + validation)
time                 = enduranceData.t_s;
cellVoltage          = enduranceData.BMSB_packVoltage / p.N_series;            % per-cell V
cellCurrent          = -(enduranceData.BMSB_packCurrent / p.N_parallel);       % per-cell A, +ve = discharge/motoring
motorSpeed_endurance = enduranceData.PM100DX_motorSpeed;
motorTorque_endurance= enduranceData.PM100DX_torqueFeedback;
wheelSpeed_endurance = mean([enduranceData.VCFRONT_wheelSpeedFL, enduranceData.VCFRONT_wheelSpeedFR, ...
                             enduranceData.VCREAR_wheelSpeedRL, enduranceData.VCREAR_wheelSpeedRR], 2);
numSamples           = length(time);
timeStep             = [diff(time); mean(diff(time))];
signCheck            = corrcoef(motorSpeed_endurance, motorTorque_endurance);
if signCheck(1,2) < 0, motorTorque_endurance = -motorTorque_endurance; end   % positive torque = motoring
fprintf('July 11 (SOC source): %d samples, %.1f min\n', numSamples, (time(end)-time(1))/60);

compData                 = readtable('data/comp_june20_data.csv');   % Comp June 20 (efficiency source)
motorSpeed_compMeasured  = compData.PM100DX_motorSpeed;
motorTorque_compMeasured = compData.PM100DX_torqueFeedback;
wheelSpeed_compMeasured  = mean([compData.VCFRONT_wheelSpeedFL, compData.VCFRONT_wheelSpeedFR, ...
                                 compData.VCREAR_wheelSpeedRL, compData.VCREAR_wheelSpeedRR], 2);
timeStep_comp            = [diff(compData.t_s); median(diff(compData.t_s))];
% NaN-proof sign fix: comp logs motoring torque negative -> net torque*rpm < 0.
if sum(motorTorque_compMeasured .* motorSpeed_compMeasured, 'omitnan') < 0
    motorTorque_compMeasured = -motorTorque_compMeasured;
end
wheelVsMotorRmse = sqrt(mean((wheelSpeed_compMeasured*p.gear_current - motorSpeed_compMeasured).^2, 'omitnan'));
fprintf('Comp (efficiency source): %d samples, %.1f min | RMSE(wheel*%.2f vs motor)=%.1f rpm\n', ...
    height(compData), compData.t_s(end)/60, p.gear_current, wheelVsMotorRmse);

% MOTORING FILTER -- TODO (design review, STUBBED not applied):
%   The driver-intent definition (>15% accelerator / torque request = accelerating)
%   requires re-exporting with VCFRONT_acceleratorPosition or VCFRONT_torqueRequest.
%   Neither is in comp_june20 / endurance_july11 today (needs DAQ access), so we keep
%   the existing shaft-power > 1 kW motoring gate (motoringMask_* below). Do NOT
%   approximate driver intent from torqueFeedback -- that is achieved torque, not demand.

%% ---- BATTERY MODEL VALIDATION (July 11) ----
% Before trusting the battery model to answer 'what if we regeared', check it can
% reproduce the pack voltage we actually recorded. Two ways: open-loop (coulomb count
% + 2-RC, no feedback) and a Kalman filter that corrects SOC off measured voltage.
% Starting SOC comes from the resting voltage of the first sample, not the BMS.
rc = p.rc;
rc.SOC0 = interp1(p.rc.OCV_lookup, p.rc.SOC_lookupR, cellVoltage(1), 'linear', 'extrap');
bmsSOC  = enduranceData.BMSB_packSOC;
fprintf('Initial SOC: %.1f%% from rest voltage (%.3f V/cell) | BMS %.1f%%\n', ...
    rc.SOC0*100, cellVoltage(1), bmsSOC(1));

[socOpenLoop, voltageOpenLoop] = run_open_loop(cellCurrent, timeStep, rc);
print_err('Open-loop RC', voltageOpenLoop - cellVoltage);

% ---- Extended Kalman Filter (voltage-corrected SOC) ----------------------------
% Standard EKF notation kept on purpose (long names would hurt readability for
% anyone who knows the math). State X = [SOC; V_R1; V_R2]:
%   A      state-transition matrix        B    input (current) vector
%   X      state estimate                 Pcov state covariance
%   Q_noise process noise    R_noise measurement noise
%   Ri     instantaneous cell resistance  R1,R2 / C1,C2  the two RC-branch R's and C's
%   tao1,2 RC time constants (R*C)        V_R1,V_R2 the two RC-branch voltages
%   VOCV   open-circuit voltage(SOC)      dOCV  dOCV/dSOC (the H Jacobian term)
%   H      measurement Jacobian           K1   Kalman gain      Ut  predicted terminal V
%
% TABLE INDEXING: the HPPC R/C tables are indexed by SOC (index 1 = empty), same as
% OCV -- see params_cfr26. This block used to read them at 1-SOC while reading OCV at
% SOC off the same grid. Fixed; see run_open_loop for the voltage-trace check that
% settled it. Coulomb-counted SOC never touches these tables, so no SOC number moved.
Q_noise = [7e-8 0 0; 0 6e-5 0; 0 0 6e-5]; R_noise = 0.2;
Pcov = [1e-4 0 0; 0 1e-4 0; 0 0 1e-4];
dOCV = gradient(p.rc.OCV_lookup, p.rc.SOC_lookupR);
X = [rc.SOC0; 0; 0]; voltageKalman = zeros(numSamples,1); socKalman = zeros(numSamples,1);
for k = 1:numSamples
    dt = timeStep(k); SOC = X(1);
    % Charging and discharging get different tables -- cells aren't symmetric.
    if cellCurrent(k) < 0, sfx='_c'; else, sfx='_d'; end
    % lookupRC clamps off the trailing-zero grid point (HPPC artifact, see params).
    Ri = lookupRC(p.rc, ['Ri' sfx], SOC);
    R1 = lookupRC(p.rc, ['R1' sfx], SOC);
    R2 = lookupRC(p.rc, ['R2' sfx], SOC);
    C1 = lookupRC(p.rc, ['C1' sfx], SOC);
    C2 = lookupRC(p.rc, ['C2' sfx], SOC);
    tao1 = R1*C1; tao2 = R2*C2;
    V_R1 = exp(-dt/tao1)*X(2) + R1*(1-exp(-dt/tao1))*cellCurrent(k);
    V_R2 = exp(-dt/tao2)*X(3) + R2*(1-exp(-dt/tao2))*cellCurrent(k);
    VOCV = interp1(p.rc.SOC_lookupR, p.rc.OCV_lookup, X(1), 'linear', 'extrap');
    Ut = VOCV - V_R1 - V_R2 - cellCurrent(k)*Ri;
    A = [1 0 0; 0 exp(-dt/tao1) 0; 0 0 exp(-dt/tao2)];
    B = [-dt/p.Q_cell; R1*(1-exp(-dt/tao1)); R2*(1-exp(-dt/tao2))];
    X = A*X + B*cellCurrent(k); Pcov = A*Pcov*A' + Q_noise;
    dOCV_v = interp1(p.rc.SOC_lookupR, dOCV, X(1), 'linear', 'extrap');
    H = [dOCV_v, -1, -1];
    K1 = Pcov*H' / (H*Pcov*H' + R_noise);
    X = X + K1*(cellVoltage(k) - Ut); Pcov = (eye(3) - K1*H)*Pcov;
    voltageKalman(k) = Ut; socKalman(k) = X(1);
end
print_err('Kalman', voltageKalman - cellVoltage);
fprintf('Final SOC three ways: open-loop %.1f%% | Kalman %.1f%% | BMS %.1f%%\n', ...
    socOpenLoop(end)*100, socKalman(end)*100, bmsSOC(end));
coulombDeltaSOC = (rc.SOC0 - socOpenLoop(end))*100; bmsDeltaSOC = bmsSOC(1) - bmsSOC(end);
fprintf('Energy throughput: coulomb dSOC=%.1f pts vs BMS dSOC=%.1f pts\n', coulombDeltaSOC, bmsDeltaSOC);

%% ---- GEAR RATIO SWEEP ----
% The core trick: wheel speed is what it is -- the car went as fast as it went. So for
% each candidate ratio we re-derive what the MOTOR would have been doing on those same
% laps (rpm scales up, torque scales down, shaft power is unchanged), then re-run the
% efficiency and battery draw at that new operating point.
% Efficiency from comp (scale measured motor op-point by ratio change; clean).
% SOC from July 11 (rescale measured current by the electrical-power delta).
shaftPower_baseline      = motorTorque_endurance .* motorSpeed_endurance * (2*pi/60);   % July 11 shaft power
efficiency_baseline      = emrax208_efficiency(abs(motorSpeed_endurance), abs(motorTorque_endurance), p);
electricalPower_baseline = motoring_regen_power(shaftPower_baseline, efficiency_baseline);
wheelPower  = zeros(size(shaftPower_baseline)); isForward = shaftPower_baseline >= 0;
wheelPower(isForward)  = shaftPower_baseline(isForward)  .* p.eta_drivetrain;
wheelPower(~isForward) = shaftPower_baseline(~isForward) ./ p.eta_drivetrain;

gearRatios = p.gears_to_test; numGears = numel(gearRatios);
resultsByRatio = struct('ratio',{},'avg_eff',{},'hi_eff',{},'SOC',{},'SOC98',{}, ...
                        'infeas_T',{},'infeas_rpm',{});
operatingPoints = struct('ratio',{},'rpm',{},'torque',{});
rcFullCharge = rc; rcFullCharge.SOC0 = 0.98; socCurves = cell(1,numGears);
fprintf('\n=== Endurance + Efficiency Breakdown by Gear Ratio ===\n');
for i = 1:numGears
    gearRatio = gearRatios(i);

    % --- EFFICIENCY from COMP (scale measured operating point by the ratio) ---
    ratioScale        = gearRatio / p.gear_current;
    motorSpeed_comp   = motorSpeed_compMeasured * ratioScale;
    motorTorque_comp  = motorTorque_compMeasured / ratioScale;
    shaftPower_comp   = motorTorque_comp .* motorSpeed_comp * (2*pi/60);   % ratio-invariant
    efficiency_comp   = emrax208_efficiency(abs(motorSpeed_comp), abs(motorTorque_comp), p);
    electricalPower_comp = motoring_regen_power(shaftPower_comp, efficiency_comp);
    motoringMask_comp = shaftPower_comp > 1000;   % existing gate (see MOTORING FILTER TODO)
    if any(motoringMask_comp)
        mechanicalEnergy = sum(shaftPower_comp(motoringMask_comp).*timeStep_comp(motoringMask_comp));
        avg_eff = mechanicalEnergy / ...
                  sum(electricalPower_comp(motoringMask_comp).*timeStep_comp(motoringMask_comp)) * 100;
        inSweetSpotMask = motoringMask_comp & (efficiency_comp >= p.eff_sweet);
        hi_eff = 100 * sum(shaftPower_comp(inSweetSpotMask).*timeStep_comp(inSweetSpotMask)) / mechanicalEnergy;
    else
        avg_eff = mean(efficiency_comp)*100; hi_eff = 0;
    end
    operatingPoints(i) = struct('ratio',gearRatio, ...
        'rpm',abs(motorSpeed_comp(motoringMask_comp)),'torque',abs(motorTorque_comp(motoringMask_comp)));

    % --- SOC from JULY 11 (rescale current by the electrical-power ratio) ---
    motorSpeed_atRatio   = wheelSpeed_endurance * gearRatio;
    shaftPower_atRatio   = motoring_regen_power(wheelPower, p.eta_drivetrain);
    angularSpeed_atRatio = motorSpeed_atRatio * (2*pi/60); motorTorque_atRatio = zeros(size(shaftPower_atRatio));
    nonZeroSpeed = angularSpeed_atRatio > 1e-3; motorTorque_atRatio(nonZeroSpeed) = shaftPower_atRatio(nonZeroSpeed) ./ angularSpeed_atRatio(nonZeroSpeed);
    efficiency_atRatio      = emrax208_efficiency(abs(motorSpeed_atRatio), abs(motorTorque_atRatio), p);
    electricalPower_atRatio = motoring_regen_power(shaftPower_atRatio, efficiency_atRatio);
    motoringMask_endurance  = shaftPower_atRatio > 1000;   % existing gate (see MOTORING FILTER TODO)
    currentScaleFactor = ones(size(electricalPower_baseline)); validBaseline = abs(electricalPower_baseline) > 1;
    currentScaleFactor(validBaseline) = electricalPower_atRatio(validBaseline) ./ electricalPower_baseline(validBaseline);
    packCurrent_atRatio = cellCurrent .* currentScaleFactor;
    packCurrent_atRatio = max(min(packCurrent_atRatio, max(cellCurrent)*1.5), min(cellCurrent)*1.5);
    [socAtRatio, ~]           = run_open_loop(packCurrent_atRatio, timeStep, rc);
    [socAtRatio_fullCharge, ~]= run_open_loop(packCurrent_atRatio, timeStep, rcFullCharge);
    socCurves{i}  = socAtRatio;

    % Feasibility: does the July 11 trace stay within motor limits at this ratio?
    torqueEnvelope = min(p.T_flat_cap, interp1(p.Prpm, p.Pkw, min(abs(motorSpeed_atRatio),p.redline), ...
        'linear','extrap') * 9549 ./ max(abs(motorSpeed_atRatio),50));
    infeasibleTorquePct = 100 * sum(motoringMask_endurance & abs(motorTorque_atRatio) > torqueEnvelope) / max(sum(motoringMask_endurance),1);
    infeasibleRpmPct    = 100 * sum(motoringMask_endurance & abs(motorSpeed_atRatio) > p.redline) / max(sum(motoringMask_endurance),1);

    % (Acceleration across ratios lives in the dedicated accel sim -- sweep_accel_sim
    %  / accel_model / accel_sim.slx. We don't duplicate a weaker point-mass calc here.)
    resultsByRatio(i) = struct('ratio',gearRatio,'avg_eff',avg_eff,'hi_eff',hi_eff,'SOC',socAtRatio(end)*100, ...
        'SOC98',socAtRatio_fullCharge(end)*100,'infeas_T',infeasibleTorquePct,'infeas_rpm',infeasibleRpmPct);
    fprintf(' %5.2f:1 -> Eff=%5.1f%% | HiEff=%5.1f%% | SOC=%5.2f%%\n', ...
        gearRatio, avg_eff, hi_eff, resultsByRatio(i).SOC);
end

%% ---- COMPARISON TABLE ----
% Print every ratio side by side, with deltas measured against the ratio on the car.
% Reference for the delta columns: the current car (p.gear_current) when it's in the
% tested set, otherwise the NEAREST tested ratio -- clearly labelled as a stand-in.
% (find() returned empty for a single/custom ratio set that omits 4.61, which then
% crashed the reference lookup; min() always returns a valid one.)
[~, currentRatioIndex] = min(abs([resultsByRatio.ratio] - p.gear_current));
referenceRatio    = resultsByRatio(currentRatioIndex).ratio;
referenceIsExact  = abs(referenceRatio - p.gear_current) < 1e-6;
if referenceIsExact
    referenceLabel = sprintf('%.2f', referenceRatio);
else
    referenceLabel = sprintf('%.2f*', referenceRatio);   % * = nearest stand-in, not the actual current ratio
end
fprintf('\n================ GEAR RATIO COMPARISON (efficiency: comp | SOC: July 11) ================\n');
fprintf(' Ratio  AvgEff  HiEff%%  FinalSOC   | vs %s:  dSOC   dHiEff\n', referenceLabel);
for i = 1:numGears
    tag = '  '; if i==currentRatioIndex, tag='>>'; end
    fprintf('%s %4.2f   %5.1f  %5.1f   %6.2f    |  %+5.2f  %+6.1f\n', ...
        tag, resultsByRatio(i).ratio, resultsByRatio(i).avg_eff, resultsByRatio(i).hi_eff, resultsByRatio(i).SOC, ...
        resultsByRatio(i).SOC-resultsByRatio(currentRatioIndex).SOC, ...
        resultsByRatio(i).hi_eff-resultsByRatio(currentRatioIndex).hi_eff);
end
fprintf(' RANGE  %4.1f-%4.1f %4.1f-%4.1f %4.2f-%4.2f\n', ...
    min([resultsByRatio.avg_eff]),max([resultsByRatio.avg_eff]), ...
    min([resultsByRatio.hi_eff]),max([resultsByRatio.hi_eff]), ...
    min([resultsByRatio.SOC]),max([resultsByRatio.SOC]));
if ~referenceIsExact
    fprintf(' (* %.2f is the nearest tested ratio to the current %.2f, which is not in this set.)\n', ...
        referenceRatio, p.gear_current);
end
% 98%-start final SOC: report the spread across whatever ratios were actually tested,
% plus the reference -- never hardcode 4.20/4.61/5.20, they may not be in a custom set.
[soc98High, idxHigh] = max([resultsByRatio.SOC98]);   % highest final SOC = lowest ratio (least energy)
[soc98Low,  idxLow ] = min([resultsByRatio.SOC98]);   % lowest  final SOC = highest ratio
if numGears == 1
    fprintf(' 98%%-start final SOC: %.1f%% (%.2f)\n', resultsByRatio(1).SOC98, resultsByRatio(1).ratio);
else
    fprintf(' 98%%-start final SOC: %.1f%% (%.2f) ... %.1f%% (%.2f) | reference %s: %.1f%%\n', ...
        soc98High, resultsByRatio(idxHigh).ratio, soc98Low, resultsByRatio(idxLow).ratio, ...
        referenceLabel, resultsByRatio(currentRatioIndex).SOC98);
end
fprintf(' For ACCELERATION across ratios, run sweep_accel_sim (the dedicated accel sim);\n');
fprintf('   this study does not duplicate a weaker point-mass accel calc.\n');
fprintf(' Eff/AvgEff = motor+inverter (physics model x eta_inverter) at each ratio''s operating\n');
fprintf('   points -- for RANKING ratios; a mild optimistic bound. MEASURED as-driven over\n');
fprintf('   endurance is ~78%%. Full battery->ground stack + per-stage: drivetrain_efficiency.m\n');
fprintf(' NOTE -- this efficiency is NOT the same source as drivetrain_efficiency.m''s. That\n');
fprintf('   file uses MEASURED telemetry (~0.86 pack->shaft); this uses the physics model x\n');
fprintf('   p.eta_inverter (~0.90). They are not independent: eta_inverter=0.95 was CHOSEN to\n');
fprintf('   reconcile the two, so their agreement is a calibration, not a validation.\n');
fprintf(' KNOWN BIAS -- CONSTANT Kt: emrax208_efficiency assumes torque per amp never droops.\n');
fprintf('   Real EMRAX Kt sags with saturation at high torque, so true copper loss (I^2*R) is\n');
fprintf('   HIGHER than modelled, and most so at high torque. LOWER ratios demand MORE motor\n');
fprintf('   torque -- so this model is most optimistic exactly where the low-ratio\n');
fprintf('   recommendation sits. Read low-ratio gains as an UPPER BOUND, not a promise.\n');
fprintf('=======================================================================================\n');

%% ================= FIGURES (tabbed dashboard + separate op-points window) =================
lib_figs(resultsByRatio, operatingPoints, gearRatios, socCurves, time, cellVoltage, voltageKalman, ...
    socOpenLoop, socKalman, bmsSOC, p);

%% ---- EXPORT ----
% Results to CSV so they can go straight into a slide without re-running MATLAB.
writetable(struct2table(resultsByRatio), 'output/gear_ratio_results.csv');
fprintf('\nSaved: output/gear_ratio_results.csv + 2 figure windows (dashboard + operating-points map)\n');
fprintf('DATA: tire mu DERIVED (not measured); aero from Ford wind tunnel + aero lead;\n');
fprintf('chassis from tilt test; battery from HPPC. Remaining estimates: Crr, wheel k-factor.\n');

%% ---- LOCAL HELPERS ----
function y = lookupRC(rc, fieldName, soc)
%LOOKUPRC  Read an HPPC R/C table at a given SOC, without falling into the artifact.
%   Every R and C table ends in a hard 0 at SOC=1 -- the HPPC fit ran out of data at
%   the top of the grid. That is not a real zero resistance/capacitance, so we drop
%   the last grid point and clamp rather than interpolate toward it. The data in
%   params_cfr26 is left exactly as the test produced it.
    nValid  = numel(rc.SOC_lookupR) - 1;
    socGrid = rc.SOC_lookupR(1:nValid);
    tbl     = rc.(fieldName);
    y = interp1(socGrid, tbl(1:nValid), ...
                min(max(soc, socGrid(1)), socGrid(end)), 'linear');
end

function print_err(name, err)
    absErr = abs(err);
    fprintf('%s: RMSE=%.4f V | Mean=%.4f | P95=%.4f | Max=%.4f V\n', ...
        name, sqrt(mean(err.^2)), mean(err), prctile(absErr,95), max(absErr));
end

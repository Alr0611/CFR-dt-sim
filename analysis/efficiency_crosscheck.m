%% EFFICIENCY_CROSSCHECK -- the physics model vs MEASURED efficiency, same data.
%
% Two ways to get motor efficiency (NOT independent -- see the note below):
%   1. PHYSICS: copper + iron loss from the EMRAX 208 HV datasheet,
%      lib/emrax208_efficiency.m, times the p.eta_inverter real-world haircut.
%   2. MEASURED: don't model it, measure it. Per-sample efficiency =
%      mechanical power out / electrical power in = (motor torque x motor speed)
%      / (pack V x pack I), binned over torque x speed.
%      Implemented in analysis/measured_efficiency_map.m.
%
% READ THIS BEFORE CALLING IT A VALIDATION. These two are NOT independent:
%   (a) p.eta_inverter = 0.95 was CHOSEN so that the physics model would land on
%       this measured number. Of course they agree -- one was fitted to the other.
%       That makes this a CALIBRATION CHECK, not independent validation.
%   (b) the 'measured' torque channel (PM100DX_torqueFeedback) is the INVERTER'S
%       OWN ESTIMATE of torque, from measured current run through its internal
%       motor model. Not a transducer. So the 'measured' side is partly
%       model-derived, and shares assumptions (notably Kt) with the physics side.
% What this script IS good for: catching drift. If someone moves a motor constant
% or the haircut, these two stop lining up and you find out.
% What would make it a real validation: a dyno pull with a shaft torque transducer.
%
% Expected result (and why the sim uses eta_inverter = 0.95):
%   - steady-state measured motor+inverter eff ~ 0.86
%   - physics (motor only) on the same points  ~ 0.91
%   - 0.91 x 0.95 = 0.86  -> the haircut closes the gap.

clear; clc;
here = fileparts(mfilename('fullpath'));
repo = fileparts(here);
addpath(repo, fullfile(repo,'lib'), here);
p = params_cfr26();

crosscheckData = readtable(fullfile(repo,'data','comp_june20_data.csv'));
motorSpeed  = abs(crosscheckData.PM100DX_motorSpeed);
motorTorque = abs(crosscheckData.PM100DX_torqueFeedback);
packVoltage = crosscheckData.BMSB_packVoltage;
packCurrent = crosscheckData.BMSB_packCurrent;
packPower       = abs(packVoltage.*packCurrent);
mechanicalPower = motorTorque .* motorSpeed * 2*pi/60;

%% ---- 0. why the measured map is motor+inverter (the gear ratio cancels) ----
% Rigid driveline -> axleSpeed = motorRPM/ratio. Forming "wheel power" as
% axleSpeed * (motorTorque*ratio) gives back motor shaft power -- the ratio
% cancels. Prove that ratio straight from the data:
axleSpeed      = abs(crosscheckData.VCREAR_wheelSpeedRL);
validRatioMask = motorSpeed>500 & axleSpeed>20;
measuredRatio  = median(motorSpeed(validRatioMask)./axleSpeed(validRatioMask));
fprintf('measured motor:axle speed ratio = %.3f (gear ratio on car: %.2f)\n', measuredRatio, p.gear_current);
fprintf('-> the gear ratio cancels out, so the measured map is MOTOR+INVERTER eff.\n\n');

%% ---- 1. the measured efficiency map from the telemetry (all motoring points) ----
measuredMapAll = measured_efficiency_map(motorSpeed, motorTorque, packVoltage, packCurrent);
fprintf('=== MEASURED MAP, all motoring points (mech power out / elec power in) ===\n');
fprintf('  energy-weighted overall eff : %.3f\n', measuredMapAll.eff_overall);
fprintf('  populated bins (>=20 pts)   : %d of %d\n\n', nnz(~isnan(measuredMapAll.eff)), numel(measuredMapAll.eff));

%% ---- 2. steady-state only: strip the transient artifact ----
% During accel, pack power also spins up inertia -- the instantaneous ratio
% books that as "loss". Keep only points where rpm AND torque hold still.
steadyWindow = 11;                                  % ~1.1 s at 10 Hz
steadyMask   = movstd(motorSpeed,steadyWindow)<40 & movstd(motorTorque,steadyWindow)<3;
measuredMapSteady = measured_efficiency_map(motorSpeed, motorTorque, packVoltage, packCurrent, 'keep', steadyMask, 'minSamples', 10);
steadyMotoringMask = measuredMapSteady.point.keep;  % the actual steady motoring mask

% linear fit  packElec = mech/eta + P0  over steady points:
% slope -> motor+inverter eff, intercept -> constant accessory draw
% (pumps/fans/LV) that shouldn't be booked against the motor.
fitDesignMatrix = [mechanicalPower(steadyMotoringMask) ones(nnz(steadyMotoringMask),1)];
fitCoeffs       = fitDesignMatrix \ packPower(steadyMotoringMask);
measuredMotorInverterEff = 1/fitCoeffs(1);  accessoryDraw = fitCoeffs(2);
steadyCount     = nnz(steadyMotoringMask);
motoringCount   = nnz(measuredMapAll.point.keep);   % all motoring points, same data
steadyPct       = 100*steadyCount/max(motoringCount,1);   % D3: report the fraction, not just the count
fprintf('=== STEADY-STATE only (%d pts, %.1f%% of %d motoring) ===\n', steadyCount, steadyPct, motoringCount);
fprintf('  fit: motor+inverter eff = %.3f | accessory draw = %.0f W\n', measuredMotorInverterEff, accessoryDraw);
if steadyCount < 100   % D3: flag a thin in-band sample
    fprintf('  [LOW CONFIDENCE: < 100 steady-state points -- a deliberate steady-state / dyno run\n');
    fprintf('   would firm up this number]\n');
end

%% ---- 3. the physics model on the exact same points ----
physicsMotorOnly = emrax208_efficiency(motorSpeed, motorTorque, rmfield(p,'eta_inverter'));  % motor physics only
physicsRealWorld = physicsMotorOnly * p.eta_inverter;                                        % what the sim uses
fprintf('  physics (motor only)        = %.3f\n', mean(physicsMotorOnly(steadyMotoringMask)));
fprintf('  x eta_inverter %.2f         = %.3f\n', p.eta_inverter, mean(physicsRealWorld(steadyMotoringMask)));
fprintf('  model - measured            = %+.4f\n\n', mean(physicsRealWorld(steadyMotoringMask)) - measuredMotorInverterEff);

%% ---- 4. binned map comparison (model evaluated on the same measured grid) ----
torqueEdges = 2.5:15:152.5;  rpmEdges = 15:600:6015;
torqueBin = discretize(motorTorque, torqueEdges);  speedBin = discretize(motorSpeed, rpmEdges);
mappedMask = measuredMapAll.point.keep & ~isnan(torqueBin) & ~isnan(speedBin);
linearIndex = sub2ind([10 10], torqueBin(mappedMask), speedBin(mappedMask));
binCount = accumarray(linearIndex, 1, [100 1]);
physicsMap = reshape(accumarray(linearIndex, physicsRealWorld(mappedMask), [100 1])./max(binCount,1), [10 10]);
physicsMap(reshape(binCount,[10 10]) < 20) = NaN;
sharedBins = ~isnan(measuredMapAll.eff) & ~isnan(physicsMap);
fprintf('=== BINNED MAPS (raw race data, %d shared bins) ===\n', nnz(sharedBins));
fprintf('  mean measured : %.3f   mean model : %.3f   mean |diff| : %.3f\n', ...
        mean(measuredMapAll.eff(sharedBins)), mean(physicsMap(sharedBins)), ...
        mean(abs(measuredMapAll.eff(sharedBins)-physicsMap(sharedBins))));
fprintf('  (raw-data bins read LOW -- transient inertia pollution. The\n');
fprintf('   steady-state fit above is the number that means something.)\n');

%% ---- 5. figure ----
figHandle = figure('Position',[80 80 1500 420], 'Color','w');
subplot(1,3,1); surf(measuredMapAll.tqCenters, measuredMapAll.rpmCenters, measuredMapAll.eff.');
  title('MEASURED (mech out / elec in, raw)'); xlabel('Torque (Nm)'); ylabel('Speed (rpm)');
  zlabel('\eta'); zlim([.5 1]); shading interp; colorbar; view(135,30);
subplot(1,3,2); surf(measuredMapAll.tqCenters, measuredMapAll.rpmCenters, physicsMap.');
  title('PHYSICS MODEL (physics x inverter)'); xlabel('Torque (Nm)'); ylabel('Speed (rpm)');
  zlabel('\eta'); zlim([.5 1]); shading interp; colorbar; view(135,30);
subplot(1,3,3); surf(measuredMapAll.tqCenters, measuredMapAll.rpmCenters, (physicsMap-measuredMapAll.eff).');
  title('MODEL - MEASURED (transient gap)'); xlabel('Torque (Nm)'); ylabel('Speed (rpm)');
  zlabel('\Delta\eta'); shading interp; colorbar; view(135,30);
outdir = fullfile(repo,'output');
if ~exist(outdir,'dir'), mkdir(outdir); end
saveas(figHandle, fullfile(outdir,'efficiency_crosscheck.png'));
fprintf('\nsaved output/efficiency_crosscheck.png\n');

%% ---- verdict ----
fprintf('\n=== VERDICT ===\n');
fprintf('Measured (steady) %.3f vs model %.3f. Agreement within %.1f pt.\n', ...
        measuredMotorInverterEff, mean(physicsRealWorld(steadyMotoringMask)), ...
        100*abs(mean(physicsRealWorld(steadyMotoringMask))-measuredMotorInverterEff));
fprintf('NOT independent confirmation -- p.eta_inverter was calibrated to THIS number,\n');
fprintf('and the "measured" torque is the inverter''s own model estimate, not a\n');
fprintf('transducer. So this shows the calibration still HOLDS (which is how you catch\n');
fprintf('drift), not that the model is externally validated. Only %d steady points.\n', steadyCount);
fprintf('A dyno pull with a shaft torque transducer is what would actually settle it.\n');

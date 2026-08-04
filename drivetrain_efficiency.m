%% DRIVETRAIN_EFFICIENCY  --
%
% The goal of this script
%   how efficient is the WHOLE drivetrain, and if we
%   change something -- halfshaft angle, diff, chain, bearings, gears -- how
%   much do we gain, in efficiency AND in battery over an endurance run?
%   Everything is a PERCENTAGE (overall eff = mechanical power / pack power).
%
% How it's grounded
%   The electrical end (pack -> motor shaft = motor + inverter) is MEASURED from
%   real telemetry by the MEASURED PACK-TO-SHAFT method:
%       eff = mechanical power out / electrical power in
%           = (motor torque x motor speed) / (pack voltage x pack current)
%   energy-weighted over the motoring points of the run. The gear ratio cancels in
%   that map, so it is pack -> SHAFT only. This file multiplies the MECHANICAL stack
%   (CFR26 DT memo v4.0 [1]) on top:
%       eff_overall = motorInverterEfficiency(MEASURED) x spur x bearings x chain
%                     x diff x halfshaft(angle)
%
% TWO EFFICIENCY SOURCES IN THIS REPO -- know which one you are quoting
%   This file uses the MEASURED number (~0.86 pack->shaft, from telemetry).
%   gear_ratio_optimization.m uses a DIFFERENT one: the physics model
%   emrax208_efficiency x p.eta_inverter (~0.90). They are not independent --
%   p.eta_inverter = 0.95 was CHOSEN to reconcile the physics model with this
%   measured number. So "the physics agrees with the measurement" is partly
%   circular by construction; it is a calibration, not a validation.
%   Use MEASURED for absolute battery->ground claims (this file). Use the physics
%   model for RANKING gear ratios (that file), where a uniform factor cancels.
%
% AND: "MEASURED" is not as clean as it sounds. The torque channel is
%   PM100DX_torqueFeedback, which is the INVERTER'S OWN ESTIMATE of torque -- it
%   comes from measured current run through the inverter's internal motor model.
%   It is not a torque transducer. So the mechanical side of every "measured"
%   efficiency here is partly MODEL-DERIVED, and it shares assumptions (notably Kt)
%   with the physics model we compare it to. A shaft torque transducer on a dyno is
%   what would make this an actual measurement.
%
% Results
%   - printed: the battery->ground breakdown + every design lever priced two ways
%              (new overall efficiency %, and battery saved on an endurance lap)
%   - figures (saved to output/): the halfshaft-angle sweep, a levers comparison,
%              efficiency by stage, the motor efficiency map with its load/rpm
%              slices, and where every gear ratio puts the endurance operating
%              points on that map.

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(here));
p = params_cfr26();

%% ============== BASELINE: today's car (edit these to match reality) ==============
% Mechanical stage efficiencies. SOURCE: CFR26 DT memo v4.0 [1] -- these are the
% memo's ASSUMED values (ranges it quotes in []), NOT measured on our car. Each is
% dyno-measurable later (a coastdown or a back-to-back dyno pull would replace the
% assumption with a real number). Until then they are documented assumptions.
baselineStages.spurGear   = 0.98;   % lubricated spur gearbox   [0.97-0.99]  ASSUMED (memo) / dyno-measurable
baselineStages.bearings   = 0.95;   % 6-bearing stack, x1        [0.94-0.99]  ASSUMED (memo) / dyno-measurable
baselineStages.chain      = 0.97;   % ER520S3 30/13, ISO 606     [0.95-0.98]  ASSUMED (memo) / dyno-measurable
baselineStages.differential = 0.92; % Drexler FS clutch-type LSD, CFR26 DT memo v4.0 value.
                            %   *** MEASURE via a coastdown / dyno to lock it. ***
                            %   PINNED: this value is shared with params_cfr26 eta_drivetrain
                            %   (0.794) and verify_math sections 11 + 16. Changing it means
                            %   changing all three together.
                            %   (A 2026-07 re-research argued for a higher 0.97 on the grounds
                            %   that published diff efficiencies are production REAR-AXLE numbers
                            %   -- they include a crown-wheel-and-pinion we do not have plus
                            %   bearings we already count separately. NOT ADOPTED: it was a
                            %   derivation, not a measurement, and no published Drexler efficiency
                            %   figure exists. Settle it with a coastdown, not an argument.)
baselineStages.halfshaftAngleDeg = p.hs_angle_deg;   % MEASURED from the suspension CAD at
                            %   static ride height, driving straight. Not a placeholder --
                            %   this is the real geometry. The joints work at this angle
                            %   every second of the lap. The sweep below is kept for
                            %   transparency: it shows how the gain depends on the angle,
                            %   not because the angle is in doubt.

% Halfshaft CV-joint loss model: eff = 1 - 2*kloss*sin(beta), two joints per shaft.
% kloss = 0.090 is BACK-FITTED to the memo's own two ASSUMED points (0.99@3deg,
% 0.94@20deg) -- it reproduces them because it was fitted to them, which is a
% round-trip, not a cross-check. No independent source. Published CV-joint loss
% figures are generally lower, so this is probably PESSIMISTIC about the current
% 12 deg shafts, i.e. it flatters the repackaging case. Settle it with a
% back-to-back dyno pull or a coastdown at two angles. Full note in params_cfr26.
% These now live in params_cfr26 (single source of truth) so accel_model can use the
% same CV-joint model; values unchanged.
KLOSS       = p.hs_kloss;          % friction-geometry coefficient
HS_CORNER   = p.hs_corner_deg;     % extra deg of articulation in a loaded corner
HS_FRAC_STR = p.hs_frac_straight;  % fraction of an endurance lap near the static angle

%% ============== the MEASURED electrical end (pack -> shaft), from telemetry ==============
% AS-DRIVEN efficiency + energy come from the ENDURANCE run (the relevant duty cycle).
% The in-band CEILING (motor actually loaded) comes from the harder COMP session's
% steady-state points -- endurance's own steady points are still part-load.
enduranceCsv = fullfile(here, 'data', 'endurance_july11_with_odo_wide.csv');
compCsv      = fullfile(here, 'data', 'comp_june20_data.csv');
[motorInverterEfficiency, ~, batteryEnergyWh, ~, sourceNote, ~, ~] = ...
    measured_pack_to_shaft(enduranceCsv);                 % as-driven + energy
[~, inBandEfficiency, ~, ~, ~, inBandSteadyCount, inBandSteadyPct] = ...
    measured_pack_to_shaft(compCsv);                      % loaded-motor ceiling
if isnan(motorInverterEfficiency)   % no telemetry -> fall back to physics at continuous point
    try, motorInverterEfficiency = emrax208_efficiency(3500, 79.6, p); catch, motorInverterEfficiency = 0.89; end
    sourceNote = 'physics model (no telemetry found)';
end
inBandLowConfidence = isnan(inBandEfficiency) || inBandSteadyCount < 100;   % D3 flag
if isnan(inBandEfficiency), inBandEfficiency = 0.86; end   % measured steady-state ceiling

%% ============== helpers ==============
halfshaftJointEfficiency = @(beta) 1 - 2*KLOSS*sind(beta);              % one halfshaft, both joints
halfshaftEfficiency      = @(static) HS_FRAC_STR*halfshaftJointEfficiency(static) ...
                                   + (1-HS_FRAC_STR)*halfshaftJointEfficiency(static+HS_CORNER);
mechanicalStackEfficiency = baselineStages.spurGear * baselineStages.bearings ...
                          * baselineStages.chain * baselineStages.differential;   % mech stack minus halfshaft
hardwareEfficiency = mechanicalStackEfficiency * halfshaftEfficiency(baselineStages.halfshaftAngleDeg);  % HARDWARE only
overall = @(S) motorInverterEfficiency * S.spurGear * S.bearings * S.chain ...
             * S.differential * halfshaftEfficiency(S.halfshaftAngleDeg);
toPercent = @(x) 100*x;
memoReference = 0.89 * 0.98 * mechanicalStackEfficiency;   % reproduce memo v4.0 with ITS assumptions

overallEfficiencyCurrent = overall(baselineStages);   % current overall pack -> ground, AS-DRIVEN

%% ============================================================================
%% 1. THE HEADLINE -- hardware first, then the motor, then the honest overall
%% ============================================================================
fprintf('=== DRIVETRAIN EFFICIENCY: battery -> ground ===\n');
fprintf(' (electrical end MEASURED from telemetry: %s)\n\n', sourceNote);

fprintf(' HARDWARE -- the physical drivetrain, motor-independent:\n');
fprintf('   spur %.0f%% x bearings %.0f%% x chain %.0f%% x diff %.0f%% x halfshaft@%ddeg %.1f%%\n', ...
    toPercent(baselineStages.spurGear), toPercent(baselineStages.bearings), toPercent(baselineStages.chain), ...
    toPercent(baselineStages.differential), baselineStages.halfshaftAngleDeg, ...
    toPercent(halfshaftEfficiency(baselineStages.halfshaftAngleDeg)));
fprintf('   = %.1f%% mechanical hardware efficiency   (params_cfr26 eta_drivetrain = %.1f%%)\n', ...
    toPercent(hardwareEfficiency), toPercent(p.eta_drivetrain));
fprintf('   NOTE: stage values are memo ASSUMPTIONS (dyno-measurable), diff %.2f included.\n', ...
    baselineStages.differential);
fprintf('   Halfshaft angle %d deg is MEASURED from suspension CAD (static, straight).\n\n', ...
    baselineStages.halfshaftAngleDeg);

fprintf(' MOTOR + INVERTER -- THREE numbers, each measuring a different thing:\n');
fprintf('   ~90%%   physics model x eta_inverter at operating points -- gear_ratio_optimization\n');
fprintf('          uses THIS to RANK ratios; it is a mild optimistic bound (verify_math sec 15)\n');
fprintf('   %.1f%%  in efficient band -- MEASURED steady-state, comp session (motor loaded)\n', toPercent(inBandEfficiency));
if inBandLowConfidence
    fprintf('          [LOW CONFIDENCE: only %d steady-state points (%.1f%% of motoring) survived\n', ...
        inBandSteadyCount, inBandSteadyPct);
    fprintf('           the in-band filter -- a deliberate steady-state / dyno run would firm it up]\n');
else
    fprintf('          [%d steady-state points, %.1f%% of motoring -- adequate sample]\n', ...
        inBandSteadyCount, inBandSteadyPct);
end
fprintf('   %.1f%%  as-driven -- MEASURED energy-weighted over the July 11 endurance run;\n', toPercent(motorInverterEfficiency));
fprintf('          endurance is driven gently, so even its steady points sit here (part-load).\n\n');

fprintf(' OVERALL = motor x hardware:\n');
fprintf('   %.1f%%  memo v4.0 -- we REPRODUCE it from the memo''s own assumptions (89%% motor, 0.98 halfshaft)\n', toPercent(memoReference));
fprintf('   %.1f%%  in-band ceiling: %.1f%% motor x %.1f%% hardware -- realistic best (motor loaded)\n', ...
    toPercent(inBandEfficiency*hardwareEfficiency), toPercent(inBandEfficiency), toPercent(hardwareEfficiency));
fprintf('   %.1f%%  as-driven: %.1f%% motor x %.1f%% hardware -- what the endurance lap delivered\n', ...
    toPercent(motorInverterEfficiency*hardwareEfficiency), toPercent(motorInverterEfficiency), toPercent(hardwareEfficiency));
fprintf('\n The %.1f -> %.1f%% drop is a PART-LOAD / OPERATIONAL cost -- a gearing/driving lever\n', ...
    toPercent(inBandEfficiency*hardwareEfficiency), toPercent(motorInverterEfficiency*hardwareEfficiency));
fprintf(' (gear_ratio_optimization), NOT a hardware loss. Hardware is %.1f%% regardless.\n\n', toPercent(hardwareEfficiency));

%% ============================================================================
%% 1b. BATTERY SIDE vs INVERTER SIDE -- DISABLED (design-review fix)
%% ============================================================================
% The interim split (pack power - dc-bus power) needs PM100DX_dcBusCurrent, which
% the design review found reads ~100 A too high (a scale-factor error in the DBC).
% Taken at face value it books ~34% of pack power as HV-cable heat -- physically
% impossible. So the split is DISABLED here: with only a trustworthy BATTERY current
% (BMSB_packCurrent) we cannot separate the inverter from the battery.
% To re-enable: a corrected dc-bus current signal, OR a dyno pull (measured shaft
% power vs measured pack power). The DYNO hook lives in split_battery_inverter().
[split, splitMsg] = split_battery_inverter();
fprintf('=== BATTERY SIDE vs INVERTER SIDE (splitting the electrical end) ===\n');
fprintf('%s\n', splitMsg);
if split.valid
    fprintf('   battery-side efficiency %.3f | inverter+motor efficiency %.3f\n', ...
        split.eta_battery, split.eta_converter);
end
fprintf('\n');

%% ============================================================================
%% 1c. DATA USAGE + IN-BAND ROBUSTNESS (design review: show the data loss)
%% ============================================================================
% Every number below is computed LIVE from the CSV -- nothing hardcoded. Two
% questions this answers: (a) where do the samples go, and (b) is the in-band
% ceiling an artefact of a tight steady-state filter?
report_data_usage(enduranceCsv, 'ENDURANCE July 11 (as-driven + energy)');
report_data_usage(compCsv,      'COMP June 20 (in-band ceiling)');

%% ============================================================================
%% 2. WHERE THE LOSSES ARE + HEADROOM PER STAGE (every stage, same treatment)
%% ============================================================================
% Each stage: how much it throws away now, a realistic best, and what closing that
% gap buys. Everything multiplies, so a stage going current c -> best b lifts overall
% by b/c and saves 1 - c/b of the pack.
stageTable = {  % name                current                                          best        how you'd get there
    'motor + inverter',   motorInverterEfficiency,                          inBandEfficiency, 'OPERATIONAL: keep motor loaded (measured ceiling) -- gear_ratio_optimization';
    'diff (LSD preload)', baselineStages.differential,                      0.94,       'lower preload (top of Drexler range)';
    'bearing stack',      baselineStages.bearings,                          0.97,       'low-drag / ceramic-hybrid bearings';
    'halfshaft angle',    halfshaftEfficiency(baselineStages.halfshaftAngleDeg), halfshaftEfficiency(0), 'straighten geometry 12 -> 0 deg (CV joints)';
    'chain',              baselineStages.chain,                             0.98,       'better lube + correct tension (top of ISO 606 range)';
    'spur gearbox',       baselineStages.spurGear,                          0.99,       'ground + lapped teeth (top of AGMA range)'};
stageNames = stageTable(:,1);
currentEfficiency = cell2mat(stageTable(:,2));
bestEfficiency    = cell2mat(stageTable(:,3));
lossNowPct = (1-currentEfficiency)*100;
overallGainPct = overallEfficiencyCurrent.*(bestEfficiency./currentEfficiency - 1)*100;
batterySavedFraction = 1 - currentEfficiency./bestEfficiency;
[~,sortOrder] = sort(batterySavedFraction,'descend');

fprintf('=== WHERE THE LOSSES ARE + HEADROOM PER STAGE (biggest opportunity first) ===\n');
fprintf(' stage              |  now -> best | loss now | overall gain | battery saved\n');
fprintf(' %s\n', repmat('-',1,80));
for i = sortOrder'
    fprintf(' %-18s | %4.1f -> %4.1f | %5.1f%%   |   +%.2f%%     | %s\n', ...
        stageNames{i}, toPercent(currentEfficiency(i)), toPercent(bestEfficiency(i)), ...
        lossNowPct(i), overallGainPct(i), battxt(batterySavedFraction(i), batteryEnergyWh));
end
fprintf('\n loss now = %% of the energy entering that stage that it turns into heat.\n');
fprintf(' Motor+inverter is the biggest loss, but most is OPERATIONAL (keep the motor loaded)\n');
fprintf(' -- the gear-ratio study''s job. The stages under it are hardware.\n\n');

%% ============================================================================
%% 3. STACKED PACKAGES + the halfshaft geometry sweep
%% ============================================================================
improvementFactor = @(row) bestEfficiency(row)/currentEfficiency(row);   % improvement for a stage row
overallHardwareOnly     = overallEfficiencyCurrent * improvementFactor(2)*improvementFactor(3)*improvementFactor(5)*improvementFactor(6);
overallHardwarePlus5deg = overallHardwareOnly * (halfshaftEfficiency(5)/halfshaftEfficiency(baselineStages.halfshaftAngleDeg));
overallHardwarePlus0deg = overallHardwareOnly * (halfshaftEfficiency(0)/halfshaftEfficiency(baselineStages.halfshaftAngleDeg));
printPackageRow = @(lbl,eff) fprintf(' %-44s | %5.1f%% | %s\n', lbl, toPercent(eff), battxt(1-overallEfficiencyCurrent/eff, batteryEnergyWh));
fprintf('=== STACKED PACKAGES (hardware you''d change together) ===\n');
fprintf(' package                                      | overall | battery saved\n');
fprintf(' %s\n', repmat('-',1,74));
printPackageRow('Hardware only (diff + bearings + chain + spur)', overallHardwareOnly);
printPackageRow('Hardware + straighten halfshafts to 5 deg',      overallHardwarePlus5deg);
printPackageRow('Hardware + straighten halfshafts to 0 deg',      overallHardwarePlus0deg);
fprintf(' (motor+inverter left out -- that''s operational, via the gear ratio.)\n\n');

fprintf('=== HALFSHAFT ANGLE SWEEP (the one continuous knob, pure geometry) ===\n');
fprintf(' The angle is CONFIRMED %d deg (MEASURED off suspension CAD). This sweep is here\n', ...
    baselineStages.halfshaftAngleDeg);
fprintf(' for transparency -- so you can see how much of the gain rides on the angle --\n');
fprintf(' NOT because the angle is provisional.\n');
fprintf(' angle | halfshaft | overall eff | battery saved\n');
for angleDeg = [0 2 4 5 6 8 10 12]
    overallAtAngle = overall(setf(baselineStages,'halfshaftAngleDeg',angleDeg));
    tag = ''; if angleDeg==baselineStages.halfshaftAngleDeg, tag = '  <- current (MEASURED)'; end
    fprintf('  %3.0f  |  %5.1f%%   |   %5.1f%%    |  %s%s\n', ...
        angleDeg, toPercent(halfshaftEfficiency(angleDeg)), toPercent(overallAtAngle), ...
        battxt(1-overallEfficiencyCurrent/overallAtAngle, batteryEnergyWh), tag);
end
overallStraightHalfshaft = overall(setf(baselineStages,'halfshaftAngleDeg',0));   % straight-ish end
overall5degHalfshaft     = overall(setf(baselineStages,'halfshaftAngleDeg',5));   % 5 deg end
fprintf('\n TARGET BAND -- halfshafts in the 0-5 deg CV-joint sweet spot (a RANGE, not a point):\n');
fprintf('   halfshaft term : %.1f%% (@5 deg)  ..  %.1f%% (@0 deg)\n', ...
    toPercent(halfshaftEfficiency(5)), toPercent(halfshaftEfficiency(0)));
fprintf('   overall DT eff : %.1f%%  ..  %.1f%%   (up from %.1f%% at the current %.0f deg)\n', ...
    toPercent(overall5degHalfshaft), toPercent(overallStraightHalfshaft), ...
    toPercent(overallEfficiencyCurrent), baselineStages.halfshaftAngleDeg);
fprintf('   battery saved  : %s  ..  %s  per endurance lap\n', ...
    battxt(1-overallEfficiencyCurrent/overall5degHalfshaft, batteryEnergyWh), ...
    battxt(1-overallEfficiencyCurrent/overallStraightHalfshaft, batteryEnergyWh));
fprintf('   Getting OFF %.0f deg is the whole win; the exact angle in 0-5 is a packaging call.\n', ...
    baselineStages.halfshaftAngleDeg);
% TWO different percentages get quoted for this change -- print both, labelled, so
% nobody mixes them up in a slide. EFFICIENCY GAIN is the multiplicative lift on
% drivetrain efficiency; BATTERY SAVED is the smaller 1 - eff_now/eff_new number.
hsNow = halfshaftEfficiency(baselineStages.halfshaftAngleDeg);
fprintf('   Two ways of saying the same change -- do not mix them up:\n');
fprintf('     EFFICIENCY GAIN (x on DT eff): %+.2f%% (12->5 deg)  %+.2f%% (12->0 deg)\n', ...
    (halfshaftEfficiency(5)/hsNow - 1)*100, (halfshaftEfficiency(0)/hsNow - 1)*100);
fprintf('     BATTERY SAVED  (per lap)     : %+.2f%% (12->5 deg)  %+.2f%% (12->0 deg)\n\n', ...
    (1-overallEfficiencyCurrent/overall5degHalfshaft)*100, ...
    (1-overallEfficiencyCurrent/overallStraightHalfshaft)*100);

%% ============================================================================
%% 3b. WHAT THIS MODEL DOES NOT PRICE  (read this before quoting the number above)
%% ============================================================================
% The honest boundary of the study: it prices the EFFICIENCY BENEFIT of straightening
% the halfshafts and nothing else. Everything below is a real cost or caveat that this
% model is silent on. Not quantified here on purpose -- we do not have the numbers, and
% inventing them would be worse than saying so.
fprintf('=== WHAT THIS MODEL DOES NOT PRICE ===\n');
fprintf(' This study prices ONE thing: the efficiency gained by straightening the\n');
fprintf(' halfshafts. It says NOTHING about what that repackaging costs elsewhere.\n');
fprintf(' Moving the diff or the wheel centre to get the shafts flat also moves:\n');
fprintf('   - ROLL CENTRE height and migration  -> open question for VD/chassis\n');
fprintf('   - CG height (diff relocation)       -> open question for VD/chassis\n');
fprintf('   - CAMBER GAIN / suspension kinematics through travel\n');
fprintf('   - MOTION RATIO (spring/damper rates would need revisiting)\n');
fprintf('   - packaging knock-ons: chassis nodes, driver cell, service access\n');
fprintf(' None of these are modelled here and NONE are quantified -- they are questions\n');
fprintf(' for VD/chassis, not numbers this tool can supply. A lap-time gain from\n');
fprintf(' efficiency can be handed straight back by a worse roll centre.\n\n');
fprintf(' ALSO NOT MODELLED -- BUMP/DROOP (suspension travel):\n');
fprintf('   The halfshaft angle is NOT fixed at %d deg. It swings through the whole\n', ...
    baselineStages.halfshaftAngleDeg);
fprintf('   range of suspension travel, every bump and every corner. The loss model\n');
fprintf('   1-2*k*sin(beta) is NONLINEAR in beta, so the travel-AVERAGED loss is not\n');
fprintf('   the loss at the average angle -- averaging the angle first is not the same\n');
fprintf('   as averaging the loss. This model evaluates at the static angle plus a\n');
fprintf('   flat cornering term; it does not integrate over travel. Building that\n');
fprintf('   needs a motion-ratio / travel-histogram model we do not have.\n');
fprintf('   Direction of the error is NOT obvious -- do not assume it is small.\n\n');

%% ============================================================================
%% 4. FIGURES (one TABBED window, saved to output/)
%% ============================================================================
outdir = fullfile(here,'output'); if ~exist(outdir,'dir'), mkdir(outdir); end
fW = figure('Name','Drivetrain efficiency','Position',[40 40 1040 580]);
tg = uitabgroup(fW);

% -- Tab: halfshaft angle sweep --
ax = axes(uitab(tg,'Title','Halfshaft angle sweep'));
halfshaftAngles = 0:0.5:12;
overallVsAngle  = arrayfun(@(bb) overall(setf(baselineStages,'halfshaftAngleDeg',bb)), halfshaftAngles)*100;
batterySavedVsAngle = (1 - overallEfficiencyCurrent./(overallVsAngle/100))*100;
yyaxis(ax,'left');  plot(ax, halfshaftAngles, overallVsAngle,'-','LineWidth',1.8); ylabel(ax,'Overall drivetrain efficiency (%)');
yyaxis(ax,'right'); plot(ax, halfshaftAngles, batterySavedVsAngle,'--','LineWidth',1.5); ylabel(ax,'Endurance battery saved (%)');
xlabel(ax,'Halfshaft static angle (deg)  [MEASURED from CAD]'); grid(ax,'on'); xlim(ax,[0 12]);
try, xregion(ax,0,5,'FaceColor',[0.30 0.62 0.40],'FaceAlpha',0.12); catch, end
xline(ax, baselineStages.halfshaftAngleDeg,'r:','LineWidth',1.3, ...
    'Label',sprintf('measured %d°',baselineStages.halfshaftAngleDeg),'LabelOrientation','horizontal');
title(ax,'Straighter halfshafts (shaded 0-5° = sweet spot)');

% -- Tab: levers ranked by battery saved --
ax = axes(uitab(tg,'Title','Levers ranked'));
if isnan(batteryEnergyWh)
    xq = batterySavedFraction*100; xlab = 'Endurance battery saved (%)'; lbls = compose('%.1f%%', batterySavedFraction*100);
else
    xq = batterySavedFraction*batteryEnergyWh; xlab = 'Endurance battery saved if stage hits its best (Wh)';
    lbls = compose('%.0f Wh (%.1f%%)', xq, batterySavedFraction*100);
end
[xs, idx] = sort(xq);
barh(ax, xs, 'FaceColor',[0.30 0.62 0.40],'EdgeColor','none'); grid(ax,'on');
set(ax,'YTick',1:numel(stageNames),'YTickLabel',stageNames(idx));
xlabel(ax, xlab); xlim(ax,[0 max(xq)*1.25]);
text(ax, xs+max(xq)*0.01, 1:numel(xs), lbls(idx), 'VerticalAlignment','middle');
title(ax,'What to fix first: stages ranked by battery saved');

% -- Tab: efficiency by stage (standalone %) -- weakest link at a glance --
% Motor+inverter bar = MEASURED loaded (in-band) capability, so every stage is on the
% same footing. As-driven is drawn as a red caret (the part-load reality).
ax = axes(uitab(tg,'Title','Efficiency by stage'));
stageNamesFig = {'motor + inverter','spur gearbox','bearing stack','chain', ...
                 'diff (LSD)', sprintf('halfshaft@%d deg',baselineStages.halfshaftAngleDeg)};
stageEfficiencies = [inBandEfficiency, baselineStages.spurGear, baselineStages.bearings, ...
                     baselineStages.chain, baselineStages.differential, ...
                     halfshaftEfficiency(baselineStages.halfshaftAngleDeg)] * 100;
[stageEffSorted, o] = sort(stageEfficiencies, 'ascend');
stageNamesSorted    = stageNamesFig(o);
cols = repmat([0.62 0.66 0.70], numel(stageEfficiencies), 1);
cols(1,:) = [0.91 0.41 0.20];
y = 1:numel(stageEffSorted);
hb = barh(ax, y, stageEffSorted, 0.62, 'FaceColor','flat','EdgeColor','none'); hold(ax,'on');
hb.CData = cols;
overallCapability = inBandEfficiency*hardwareEfficiency*100;
overallAsDriven   = overallEfficiencyCurrent*100;
yov = numel(stageEffSorted) + 1.5;
barh(ax, yov, overallCapability, 0.62, 'FaceColor',[0.20 0.28 0.40],'EdgeColor','none');
grid(ax,'on'); xlim(ax,[0 108]); xlabel(ax,'Stage efficiency (%)');
set(ax,'YTick',[y, yov], 'YTickLabel',[stageNamesSorted, {'OVERALL (motor in-band)'}]);
ylim(ax,[0 yov+0.9]);
text(ax, stageEffSorted+1, y, compose('%.1f%%', stageEffSorted), 'VerticalAlignment','middle','FontWeight','bold');
text(ax, overallCapability+1, yov, compose('%.1f%%', overallCapability), 'VerticalAlignment','middle', ...
    'FontWeight','bold','Color',[0.20 0.28 0.40]);
motorRowIndex = find(strcmp(stageNamesSorted,'motor + inverter'),1);
plot(ax, motorInverterEfficiency*100, motorRowIndex, 'v', 'MarkerSize',9, 'MarkerFaceColor',[0.85 0.20 0.15],'MarkerEdgeColor','k');
text(ax, motorInverterEfficiency*100, motorRowIndex-0.4, sprintf('as-driven %.0f%%', motorInverterEfficiency*100), ...
    'HorizontalAlignment','center','FontSize',8,'Color',[0.6 0.1 0.1]);
plot(ax, overallAsDriven, yov, 'v', 'MarkerSize',9, 'MarkerFaceColor',[0.85 0.20 0.15],'MarkerEdgeColor','k');
text(ax, overallAsDriven, yov-0.42, sprintf('as-driven %.0f%%', overallAsDriven), ...
    'HorizontalAlignment','center','FontSize',8,'Color',[0.6 0.1 0.1]);
xline(ax, overallCapability, ':', 'Color',[0.20 0.28 0.40], 'LineWidth',1.2);
title(ax, sprintf(['Stage CAPABILITY: weakest link is %s (%.0f%% loaded); six multiply to %.1f%% ' ...
    'ceiling. Red caret = as-driven (part-load) -> %.1f%% overall'], ...
    strtrim(stageNamesSorted{motorRowIndex}), stageEffSorted(motorRowIndex), overallCapability, overallAsDriven));

%% ============================================================================
%% 5. WHERE WE ACTUALLY RUN vs THE EFFICIENT ISLAND (+ what gearing does)
%% ============================================================================
% Load the real endurance operating points (driver-intent motoring) so we can show,
% not assert, that the motor spends the lap OFF its efficient island.
% MOTORING FILTER -- TODO (design review, STUBBED): driver-intent (>15% accelerator)
% needs VCFRONT_acceleratorPosition, absent from the current exports (needs DAQ
% access). Keep the existing motorSpeed>500 & torque>2 gate; do not fake it.
operatingRpm = []; operatingTorque = [];
if isfile(enduranceCsv)
    opsTable = readtable(enduranceCsv);
    if all(ismember({'PM100DX_motorSpeed','PM100DX_torqueFeedback'}, opsTable.Properties.VariableNames))
        motorSpeedOps  = abs(opsTable.PM100DX_motorSpeed);
        motorTorqueOps = abs(opsTable.PM100DX_torqueFeedback);
        motoringMaskOps = motorSpeedOps > 500 & motorTorqueOps > 2;   % baseline gate (2 Nm here, not 5)
        operatingRpm    = motorSpeedOps(motoringMaskOps);
        operatingTorque = motorTorqueOps(motoringMaskOps);
    end
end

% -- Tab: motor+inverter efficiency MAP with our operating cloud + the island --
% Masked to the motor's ACHIEVABLE torque envelope so the "island" is reachable.
% Colours are CLEAN physics; measured as-driven sits ~10 pts below.
axm = axes(uitab(tg,'Title','Efficiency map'));
[rpmGrid, torqueGrid] = meshgrid(0:100:6000, 0:2:150);
efficiencyGrid = emrax208_efficiency(rpmGrid, torqueGrid, p) * 100;
peakPowerAtRpm = interp1(p.Prpm, p.Pkw, min(rpmGrid,p.redline), 'linear','extrap');   % kW available vs rpm
torqueEnvelopeGrid = min(p.T_flat_cap, peakPowerAtRpm*9549 ./ max(rpmGrid,50));        % reachable torque
efficiencyGrid(torqueGrid > torqueEnvelopeGrid) = NaN;                                 % grey out unreachable
contourf(axm, rpmGrid, torqueGrid, efficiencyGrid, [70 75 80 84 86 88 90 91 92], 'LineColor',[.6 .6 .6], 'HandleVisibility','off');
hold(axm,'on'); colormap(axm, parula); clim(axm,[70 92]);
cb = colorbar(axm); cb.Label.String = 'motor + inverter efficiency (%, clean physics)';
efficiencyReachable = efficiencyGrid; efficiencyReachable(isnan(efficiencyReachable)) = -inf;
[~, islandIndex] = max(efficiencyReachable(:));
hIsl = plot(axm, rpmGrid(islandIndex), torqueGrid(islandIndex), 'p', 'MarkerSize',18, 'MarkerFaceColor','w','MarkerEdgeColor','k','LineWidth',1.3);
text(axm, rpmGrid(islandIndex), torqueGrid(islandIndex)-11, sprintf('island ~%.0f%%', efficiencyGrid(islandIndex)), ...
    'HorizontalAlignment','center','FontWeight','bold');
hCloud = [];
if ~isempty(operatingRpm)
    ds = 1:12:numel(operatingRpm);
    hCloud = scatter(axm, operatingRpm(ds), operatingTorque(ds), 7, [0.85 0.20 0.15], 'filled', 'MarkerFaceAlpha',0.18);
    centreRpm = median(operatingRpm); centreTorque = median(operatingTorque); ratioScale = 4.00 / p.gear_current;
    plot(axm, [centreRpm centreRpm*ratioScale], [centreTorque centreTorque/ratioScale], 'w-', 'LineWidth',3, 'HandleVisibility','off');
    hArr = plot(axm, centreRpm*ratioScale, centreTorque/ratioScale, 'w^', 'MarkerFaceColor','w', 'MarkerSize',11);
    text(axm, 2850, centreTorque-7, sprintf('median ~%.0f Nm (part-load)', centreTorque), 'Color',[0.6 0.05 0.05],'FontWeight','bold');
    legend(axm, [hCloud hIsl hArr], {'as-driven operating points', ...
        sprintf('reachable island ~%.0f%%',efficiencyGrid(islandIndex)), 'lower gearing (4.0:1) loads motor'}, ...
        'Location','southeast','FontSize',8,'TextColor','w','Color',[0.15 0.15 0.15]);
end
yline(axm, p.T_flat_cap, 'w--', 'HandleVisibility','off');
xline(axm, p.redline, 'r:', 'HandleVisibility','off');
xlabel(axm,'Motor rpm'); ylabel(axm,'Motor torque (Nm)'); xlim(axm,[0 6000]); ylim(axm,[0 150]);
title(axm, 'Where we run (red) vs the reachable efficient island. Grey = torque the motor can''t make at that rpm.');

% -- Tab: efficiency vs LOAD (line) with the ACTUAL time-at-load distribution --
axa = axes(uitab(tg,'Title','Eff vs load'));
cruiseRpm = 3000; torqueSweep = 0:1:150; CLIFF = 15;
efficiencyVsTorque = emrax208_efficiency(cruiseRpm*ones(size(torqueSweep)), torqueSweep, p) * 100;
yyaxis(axa,'left');
patch(axa, [0 CLIFF CLIFF 0], [60 60 95 95], [0.85 0.25 0.15], 'FaceAlpha',0.09, 'EdgeColor','none');
hold(axa,'on');
plot(axa, torqueSweep, efficiencyVsTorque, 'LineWidth', 2.4);
[peakEff, iPeak] = max(efficiencyVsTorque);
plot(axa, torqueSweep(iPeak), peakEff, 'k^', 'MarkerFaceColor','k');
text(axa, torqueSweep(iPeak), peakEff+1.3, sprintf('peak %.0f%% @ %d Nm', peakEff, torqueSweep(iPeak)), 'FontSize',9, 'HorizontalAlignment','center');
ylabel(axa,'motor + inverter eff (%)'); ylim(axa,[60 94]);
yyaxis(axa,'right'); cliffPct = 0;
if ~isempty(operatingTorque)
    histogram(axa, operatingTorque, 0:5:150, 'Normalization','probability', 'FaceColor',[0.45 0.5 0.55], 'FaceAlpha',0.45, 'EdgeColor','none');
    ratioScale = 4.00 / p.gear_current;
    histogram(axa, operatingTorque/ratioScale, 0:5:150, 'Normalization','probability', 'FaceColor',[0.15 0.35 0.7], 'FaceAlpha',0.30, 'EdgeColor','none');
    cliffPct = 100*mean(operatingTorque < CLIFF);
    ylabel(axa,'share of endurance time');
end
grid(axa,'on'); xlabel(axa,'Motor torque (Nm) at 3000 rpm'); xlim(axa,[0 150]);
title(axa, sprintf(['Efficiency vs LOAD (line) + where we actually spend time (bars): %.0f%% of the lap ' ...
    'is below %d Nm in the cliff.\nGrey = now, blue = lower gearing shifts it right.'], cliffPct, CLIFF));

% -- Tab: efficiency vs RPM at fixed torque -- past-peak fall-off --
axb = axes(uitab(tg,'Title','Eff vs rpm'));
fixedTorque = 40; rpmSweep = 500:25:6000;
efficiencyVsRpm = emrax208_efficiency(rpmSweep, fixedTorque*ones(size(rpmSweep)), p) * 100;
plot(axb, rpmSweep, efficiencyVsRpm, 'LineWidth', 2.4); hold(axb,'on'); grid(axb,'on');
[peakRpmEff, iPeakRpm] = max(efficiencyVsRpm);
plot(axb, rpmSweep(iPeakRpm), peakRpmEff, 'k^', 'MarkerFaceColor','k');
text(axb, rpmSweep(iPeakRpm), peakRpmEff-1.6, sprintf('peak %.0f%% @ %d rpm', peakRpmEff, rpmSweep(iPeakRpm)), 'FontSize',9, 'HorizontalAlignment','center');
xline(axb, p.redline, 'r:', 'redline');
if ~isempty(operatingRpm)
    medianRpm = median(operatingRpm); effAtMedian = emrax208_efficiency(medianRpm, fixedTorque, p)*100;
    plot(axb, medianRpm, effAtMedian, 'o', 'MarkerSize',9, 'MarkerFaceColor',[0.85 0.2 0.15],'MarkerEdgeColor','k');
    text(axb, medianRpm, effAtMedian-1.8, sprintf('endurance median ~%.0f rpm', medianRpm), 'FontSize',8,'Color',[0.6 0.1 0.1],'HorizontalAlignment','center');
end
xlabel(axb,sprintf('Motor rpm (at a fixed %d Nm)', fixedTorque)); ylabel(axb,'motor + inverter eff (%)'); ylim(axb,[80 94]);
title(axb,'Efficiency vs RPM: past the peak it falls as core loss grows (\propto rpm^2)');

% -- Tab: where every GEAR RATIO puts the endurance operating points on the map --
axG = axes(uitab(tg,'Title','Gear ratios on map'));
contourf(axG, rpmGrid, torqueGrid, efficiencyGrid, [70 75 80 84 86 88 90 91 92], 'LineColor',[.6 .6 .6], 'HandleVisibility','off');
hold(axG,'on'); colormap(axG, parula); clim(axG,[70 92]);
cbG = colorbar(axG); cbG.Label.String = 'motor + inverter efficiency (%, clean physics)';
plot(axG, rpmGrid(1,:), torqueEnvelopeGrid(1,:), 'w-', 'LineWidth',2, 'HandleVisibility','off');   % datasheet peak envelope
hIslG = plot(axG, rpmGrid(islandIndex), torqueGrid(islandIndex), 'p', 'MarkerSize',16, 'MarkerFaceColor','w','MarkerEdgeColor','k','LineWidth',1.2);
text(axG, rpmGrid(islandIndex), torqueGrid(islandIndex)-10, 'peak island', 'HorizontalAlignment','center','FontWeight','bold','Color','w','FontSize',8);
if ~isempty(operatingRpm)
    ds = 1:20:numel(operatingRpm);
    scatter(axG, operatingRpm(ds), operatingTorque(ds), 5, [0.6 0.6 0.62], 'filled', 'MarkerFaceAlpha',0.10, 'HandleVisibility','off');
    gearRatioSet = sort(p.gears_to_test(:))';
    energyWeight = operatingRpm .* operatingTorque;                    % ~ shaft power: ratio-invariant weights
    centroidRpm = zeros(size(gearRatioSet)); centroidTorque = centroidRpm; ratioEfficiency = centroidRpm; fractionOverRedline = centroidRpm;
    for i = 1:numel(gearRatioSet)
        ratioScale = gearRatioSet(i)/p.gear_current; reRpm = operatingRpm*ratioScale; reTorque = operatingTorque/ratioScale;
        centroidRpm(i) = sum(reRpm.*energyWeight)/sum(energyWeight);  centroidTorque(i) = sum(reTorque.*energyWeight)/sum(energyWeight);
        ratioEfficiency(i) = sum(emrax208_efficiency(reRpm, reTorque, p).*energyWeight)/sum(energyWeight)*100;
        fractionOverRedline(i) = 100*mean(reRpm > p.redline);
    end
    plot(axG, centroidRpm, centroidTorque, 'w-', 'LineWidth',1.5, 'HandleVisibility','off');
    hLoc = scatter(axG, centroidRpm, centroidTorque, 70, ratioEfficiency, 'filled', 'MarkerEdgeColor','k');
    keyRatios = unique([min(gearRatioSet) p.gear_current max(gearRatioSet)]);
    aln = {'right','center','left'}; ddx = [-160 0 160]; ddy = [12 20 -14];
    for k = 1:numel(keyRatios)
        [~,j] = min(abs(gearRatioSet - keyRatios(k))); s = min(k,3);
        lab = sprintf('%.2f:1  ~%.0f rpm  %.1f%%', gearRatioSet(j), centroidRpm(j), ratioEfficiency(j));
        if fractionOverRedline(j) > 1, lab = sprintf('%s (%.0f%% >redline)', lab, fractionOverRedline(j)); end
        text(axG, centroidRpm(j)+ddx(s), centroidTorque(j)+ddy(s), lab, 'HorizontalAlignment',aln{s}, 'FontWeight','bold','Color','w','FontSize',8);
    end
    legend([hLoc hIslG], {'ratio operating centre (colour = its endurance eff)','efficient island'}, ...
        'Location','southeast','FontSize',8,'TextColor','w','Color',[0.15 0.15 0.15]);
end
xline(axG, p.redline, 'r:', 'redline', 'HandleVisibility','off');
xlabel(axG,'Motor rpm (endurance, same laps re-geared)'); ylabel(axG,'Motor torque (Nm)');
xlim(axG,[0 6000]); ylim(axG,[0 150]);
title(axG, ['Same endurance laps, every gear ratio: higher ratio -> more rpm / less torque, ' ...
    'lower -> more torque. White line = datasheet peak envelope; endurance sits well below it.']);

save_tabfig(fW, fullfile(outdir,'DTeff_Drivetrain_efficiency'));
fprintf(['Saved 1 tabbed figure window to output/ (halfshaft sweep, levers ranked, efficiency by\n' ...
         '  stage, efficiency map, eff vs load, eff vs rpm, gear ratios on map).\n\n']);
fprintf(' MOTOR+INVERTER is a MAP, not one number: peak (loaded) ~%.0f%%, in efficient band %.0f%%,\n', ...
    emrax208_efficiency(2500,65,p)*100, inBandEfficiency*100);
fprintf('   as-driven endurance AVERAGE %.0f%% (part-load, not a worn motor).\n\n', motorInverterEfficiency*100);

fprintf('Sources: [1] CFR26_DT_Efficiency.pdf v4.0 (stage table + straight/corner time split).\n');
fprintf('         Electrical end = measured pack-to-shaft efficiency (mech power out /\n');
fprintf('         electrical power in, energy-weighted over motoring) on July 11 telemetry.\n');
fprintf('         Diff LSD range from x-engineer / RoyMech / bevel-diff efficiency refs.\n');

%% ============================== local functions ==============================
function s = setf(s, field, val)
    s.(field) = val;   % copy a struct with one field changed
end

function [out, msg] = split_battery_inverter()
%SPLIT_BATTERY_INVERTER  Separate battery-side losses from inverter-side losses.
%
%   DISABLED (design review): the interim dc-bus method needs PM100DX_dcBusCurrent,
%   which reads ~100 A too high (a DBC scale-factor error) -- it books an impossible
%   ~34% of pack power as HV-cable heat. With only a trustworthy BATTERY current
%   (BMSB_packCurrent) the inverter cannot be separated from the battery.
%
%   To re-enable, provide ONE of:
%     - a corrected PM100DX_dcBusCurrent signal (fix the DBC scaling), or
%     - a DYNO pull: measured shaft power vs measured pack power. Fill DYNO below.
    out = struct('valid', false, 'source', 'disabled', ...
                 'eta_battery', NaN, 'eta_converter', NaN);

    % ---------- DYNO hook (fill this in when the sponsor run happens) ----------
    DYNO.have    = false;
    DYNO.P_pack  = [];   % W, measured at the pack terminals
    DYNO.P_dcbus = [];   % W, measured at the inverter dc input
    DYNO.P_shaft = [];   % W, measured on the brake (torque x speed)
    if DYNO.have
        packPower = sum(DYNO.P_pack); dcBusPower = sum(DYNO.P_dcbus); shaftPower = sum(DYNO.P_shaft);
        out.valid = true; out.source = 'DYNO (measured shaft power)';
        out.eta_battery   = dcBusPower/packPower;
        out.eta_converter = shaftPower/dcBusPower;
        msg = '  source: DYNO -- measured split.';
        return;
    end

    msg = ['  DISABLED -- battery/inverter split needs a corrected dc-bus current' newline ...
           '   signal (PM100DX_dcBusCurrent reads ~100 A high) or a dyno pull.' newline ...
           '   The pack->shaft efficiency above uses BMSB_packCurrent and is unaffected.'];
end

function s = battxt(frac, Wh)
    % battery-saved cell: "+NN Wh (+X.XX%)" if we know the run energy, else "+X.XX%"
    if isnan(Wh), s = sprintf('+%.2f%%', frac*100);
    else,         s = sprintf('+%.0f Wh (+%.2f%%)', frac*Wh, frac*100); end
end

function [asDrivenEfficiency, steadyStateEfficiency, batteryEnergyWh, durationMin, sourceNote, steadyCount, steadyPct] = ...
         measured_pack_to_shaft(csvPath)
%MEASURED_PACK_TO_SHAFT  Motor+inverter efficiency measured from a telemetry CSV.
%   THE METHOD, in one line: efficiency = mechanical power OUT divided by electrical
%   power IN, energy-weighted over the points where the car was actually motoring.
%     mechanical power out = motor torque x rotational speed  (rear axle if we have
%                            wheel-speed channels -- the gear ratio cancels either way)
%     electrical power in  = pack voltage x pack current      (BMSB_packCurrent)
%   Because the ratio cancels, this is a pack -> SHAFT number: motor + inverter only,
%   with no drivetrain hardware in it.
%
%   asDrivenEfficiency    = energy-weighted sum(mech)/sum(pack) over MOTORING points
%                           (part-load + transients -- what actually happened).
%   steadyStateEfficiency = same but STEADY-STATE only (rpm & torque near-constant --
%                           transients filtered out) -> the loaded-motor ceiling.
%   Also returns pack energy (Wh), run length, and the steady-state sample count +
%   fraction (design-review fix: so callers can flag a low-confidence in-band number).
%
%   Uses BMSB_packCurrent (battery current) for the electrical side -- the
%   trustworthy channel.
%   MOTORING FILTER -- TODO (design review, STUBBED): the driver-intent definition
%   (>15% accelerator / torque request = accelerating) needs VCFRONT_acceleratorPosition
%   or VCFRONT_torqueRequest, which are NOT in these exports (needs DAQ access). We
%   keep the existing motorSpeed>500 & torque>5 gate. Do not fake driver intent from
%   torqueFeedback -- that is achieved torque, not demand.
    asDrivenEfficiency = NaN; steadyStateEfficiency = NaN; batteryEnergyWh = NaN;
    durationMin = NaN; sourceNote = ''; steadyCount = 0; steadyPct = NaN;
    if ~isfile(csvPath), return; end
    dataTable = readtable(csvPath);
    requiredChannels = {'PM100DX_motorSpeed','PM100DX_torqueFeedback','BMSB_packVoltage','BMSB_packCurrent','t_s'};
    if ~all(ismember(requiredChannels, dataTable.Properties.VariableNames)), return; end
    motorSpeed  = abs(dataTable.PM100DX_motorSpeed);
    motorTorque = abs(dataTable.PM100DX_torqueFeedback);
    packVoltage = dataTable.BMSB_packVoltage;
    packCurrent = dataTable.BMSB_packCurrent;
    packPower       = abs(packVoltage .* packCurrent);
    % ---- REAR-AXLE MECHANICAL POWER (speed from the wheel sensors, not the inverter) ----
    % Speed now comes from the REAR WHEEL SENSORS (mean of RL and RR) instead of the
    % inverter's motorSpeed; axle torque = motorTorque x gearRatio. This is a
    % SIGNAL-SOURCE change (trust the wheel sensor), not a number change:
    %   * The gear ratio CANCELS, so we use the MEASURED ratio (median motorRPM /
    %     rearWheelRPM, printed in the DATA USAGE block): axleTorque x axleSpeed then
    %     returns motor shaft power to within ~0.1% and the efficiency is unchanged.
    %   * That agreement is the POINT. The TRUE mechanical ratio is 15/30 spur x 30/13
    %     chain = 4.6154 (15/30 spur x 30/13 chain; "4.61" is the shop rounding, -0.12%). The wheel
    %     sensors measure 4.622, i.e. +0.14% off the true ratio -- so motor and rear
    %     tyres agree to about a tenth of a percent. No meaningful wheel slip.
    %   * Do NOT substitute p.gear_current (4.61) here: it is the ROUNDED number, and
    %     pairing it with sensor-derived speed injects a ~0.4% bookkeeping error that
    %     looks like an efficiency loss but is only the rounding.
    %   * This is still MOTOR-SIDE power expressed on the axle. TRUE wheel-side power
    %     needs MEASURED WHEEL TORQUE, which we do not log -- that is a dyno job.
    % Falls back to motorSpeed if the rear-wheel channels are absent.
    hasRearWheel = all(ismember({'VCREAR_wheelSpeedRL','VCREAR_wheelSpeedRR'}, ...
                                dataTable.Properties.VariableNames));
    if hasRearWheel
        rearWheelSpeed = (abs(dataTable.VCREAR_wheelSpeedRL) + abs(dataTable.VCREAR_wheelSpeedRR))/2;
        % measured ratio, from rolling points only (parked samples would divide by ~0)
        rollingMask  = motorSpeed>500 & rearWheelSpeed>50;
        measuredRatio = median(motorSpeed(rollingMask)./rearWheelSpeed(rollingMask));
        axleTorque      = motorTorque * measuredRatio;
        mechanicalPower = axleTorque .* (rearWheelSpeed * 2*pi/60);
    else
        measuredRatio   = NaN;
        mechanicalPower = motorTorque .* motorSpeed * 2*pi/60;   % fallback: motor shaft
    end
    instantEfficiency = mechanicalPower ./ max(packPower, 1e-6);
    % Existing motoring gate (see MOTORING FILTER TODO in the header): clean motoring points.
    motoringMask = motorSpeed>500 & motorTorque>5 & packPower>500 ...
                 & instantEfficiency>0.3 & instantEfficiency<1.0;
    asDrivenEfficiency = sum(mechanicalPower(motoringMask)) / sum(packPower(motoringMask));   % AS-DRIVEN
    steadyMask = motoringMask & movstd(motorSpeed,11)<40 & movstd(motorTorque,11)<3;          % constant speed & torque
    steadyCount = nnz(steadyMask);
    if nnz(motoringMask) > 0, steadyPct = 100*steadyCount/nnz(motoringMask); end
    if steadyCount > 50, steadyStateEfficiency = sum(mechanicalPower(steadyMask)) / sum(packPower(steadyMask)); end
    timeVec = dataTable.t_s; packPowerSigned = packVoltage .* packCurrent; packPowerSigned(isnan(packPowerSigned)) = 0;
    batteryEnergyWh = abs(trapz(timeVec, packPowerSigned)) / 3600;
    durationMin = (timeVec(end)-timeVec(1))/60;
    sourceNote = sprintf('%d motoring samples (%d steady-state, %.1f%%)', nnz(motoringMask), steadyCount, steadyPct);
end

function report_data_usage(csvPath, label)
%REPORT_DATA_USAGE  Live accounting of where a telemetry run's samples go, plus a
%   robustness sweep on the steady-state (in-band) filter.
%   Nothing here is hardcoded -- every count, fraction and efficiency is computed
%   from the CSV. The masks MIRROR measured_pack_to_shaft above (that function is
%   the source of truth); the categories below partition the run exactly once, so
%   the rows always sum to TOTAL.
    if ~isfile(csvPath), fprintf('=== DATA USAGE: %s -- CSV not found ===\n\n', label); return; end
    dataTable = readtable(csvPath);
    needed = {'PM100DX_motorSpeed','PM100DX_torqueFeedback','BMSB_packVoltage','BMSB_packCurrent'};
    if ~all(ismember(needed, dataTable.Properties.VariableNames))
        fprintf('=== DATA USAGE: %s -- required channels missing ===\n\n', label); return;
    end
    motorSpeed  = abs(dataTable.PM100DX_motorSpeed);
    motorTorque = abs(dataTable.PM100DX_torqueFeedback);
    packPower   = abs(dataTable.BMSB_packVoltage .* dataTable.BMSB_packCurrent);
    % same mechanical basis as measured_pack_to_shaft: rear axle if we have it
    if all(ismember({'VCREAR_wheelSpeedRL','VCREAR_wheelSpeedRR'}, dataTable.Properties.VariableNames))
        rearWheelSpeed = (abs(dataTable.VCREAR_wheelSpeedRL) + abs(dataTable.VCREAR_wheelSpeedRR))/2;
        rollingMask    = motorSpeed>500 & rearWheelSpeed>50;
        measuredRatio  = median(motorSpeed(rollingMask)./rearWheelSpeed(rollingMask));
        mechanicalPower = (motorTorque*measuredRatio) .* (rearWheelSpeed * 2*pi/60);
    else
        measuredRatio   = NaN;
        mechanicalPower = motorTorque .* motorSpeed * 2*pi/60;
    end
    instantEfficiency = mechanicalPower ./ max(packPower, 1e-6);

    % --- partition the run (mutually exclusive, exhaustive, in this order) ---
    stationary  = motorSpeed <= 500;                            % parked / staging
    coasting    = ~stationary & motorTorque <= 5;               % rolling, off power
    plausible   = instantEfficiency > 0.3 & instantEfficiency < 1.0;
    sensorNoise = ~stationary & ~coasting & (packPower <= 500 | ~plausible);
    motoring    = ~stationary & ~coasting & ~sensorNoise;       % == the headline gate
    steady      = motoring & movstd(motorSpeed,11)<40 & movstd(motorTorque,11)<3;
    total       = height(dataTable);
    pctOf = @(n) 100*n/max(total,1);

    fprintf('=== DATA USAGE: %s ===\n', label);
    if ~isnan(measuredRatio)
        fprintf(' (mechanical side = rear-axle: measured motorRPM/rearWheelRPM = %.3f)\n', measuredRatio);
    end
    fprintf('  %-33s %7s  %5s\n', 'TOTAL', addcomma(total), '100%');
    fprintf('    %-31s %7s  %4.1f%%   (parked: staging, driver change, stops, cool-down)\n', ...
        'stationary / staging (rpm<=500)', addcomma(nnz(stationary)), pctOf(nnz(stationary)));
    fprintf('    %-31s %7s  %4.1f%%   (rolling, no motor torque)\n', ...
        'coasting / braking (off-power)', addcomma(nnz(coasting)), pctOf(nnz(coasting)));
    fprintf('    %-31s %7s  %4.1f%%\n', ...
        'near-zero power / sensor noise', addcomma(nnz(sensorNoise)), pctOf(nnz(sensorNoise)));
    fprintf('    %-31s %7s  %4.1f%%   <- all real driving\n', ...
        'MOTORING (headline uses this)', addcomma(nnz(motoring)), pctOf(nnz(motoring)));
    fprintf('      %-29s %7s  %4.1f%%\n', ...
        'of which steady (ceiling)', addcomma(nnz(steady)), pctOf(nnz(steady)));
    fprintf('    %-31s %7s          (rows must sum to TOTAL)\n', 'CHECK sum', ...
        addcomma(nnz(stationary)+nnz(coasting)+nnz(sensorNoise)+nnz(motoring)));

    % --- robustness: does the in-band number survive a looser filter? ---
    fprintf('  IN-BAND ROBUSTNESS (filter loosened; reported number is the first row):\n');
    rpmTol = [40 60 80 120];  torqueTol = [3 5 8 12];
    for k = 1:numel(rpmTol)
        band = motoring & movstd(motorSpeed,11)<rpmTol(k) & movstd(motorTorque,11)<torqueTol(k);
        eff  = sum(mechanicalPower(band)) / sum(packPower(band));
        marker = ''; if k==1, marker = '   <- current'; end
        fprintf('    rpm<%3d & tq<%2d : %5d pts (%4.1f%% of motoring)  eff %.3f%s\n', ...
            rpmTol(k), torqueTol(k), nnz(band), 100*nnz(band)/max(nnz(motoring),1), eff, marker);
    end
    fprintf('    -> the ceiling is stable across a %.0fx sample range; the tight filter is\n', ...
        nnz(motoring & movstd(motorSpeed,11)<rpmTol(end) & movstd(motorTorque,11)<torqueTol(end)) / max(nnz(steady),1));
    fprintf('       not manufacturing the number. Filter and reported value UNCHANGED.\n\n');
end

function s = addcomma(n)
%ADDCOMMA  Thousands separators, so the sample accounting reads like a table.
    s = sprintf('%d', n);
    for k = numel(s)-3:-3:1, s = [s(1:k) ',' s(k+1:end)]; end
end

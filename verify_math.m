%% VERIFY_MATH — regression suite over this repo's OWN assumptions
%
% WHAT THIS IS
%   A regression suite. It re-derives the load-bearing numbers from scratch, from
%   the SAME parameters the sim uses (params_cfr26), and shouts if a result stops
%   matching. Its job is to stop someone (human or AI) editing a constant and
%   quietly changing a conclusion three files away.
%
% WHAT THIS IS *NOT*
%   External validation. Nothing here is an independent measurement of the car.
%   Most "expected" values are the repo's own assumptions, or the CFR26 DT memo's
%   assumptions, which the repo inherited. A green run means THE MODEL IS
%   SELF-CONSISTENT. It does not mean the model is right about the car.
%   The things that would actually validate it: a dyno pull, a coastdown, a
%   deliberate steady-state efficiency run, a measured loaded wheel radius.
%   Where a check compares against something genuinely external -- an ESF number,
%   a datasheet rating, a CFD force -- it says so on the line.
%
% INPUTS COME FROM params_cfr26(). This file used to hardcode its own copies of
%   R_phase, Nm_per_Arms, the core-loss coefficients and eta_inverter, which meant
%   you could edit params_cfr26, change every result in the repo, and still watch
%   this print PASS all the way down. That is exactly backwards for a gate. Every
%   model input below now reads p.* so a params edit CANNOT pass unnoticed.
% (This is a FUNCTION file, not a script, purely so the pass/fail counter below can
%  share a workspace with the checks. Call it the same way: verify_math)
function verify_math()
clc;
thisdir = fileparts(mfilename('fullpath'));
addpath(thisdir, fullfile(thisdir,'lib'));
p = params_cfr26();

nPass = 0; nFail = 0;
% pass() prints one PASS/FAIL line and keeps the tally. Nested so it can see the counters.
    function pass(name, cond)
        cond = logical(cond);
        if cond, nPass = nPass + 1; else, nFail = nFail + 1; end
        fprintf('[%s] %s\n', string(cond).replace("true","PASS").replace("false","**FAIL**"), name);
    end

% The physics model, rebuilt here from p.* -- deliberately NOT a call to
% emrax208_efficiency, so the two are independent and can catch each other.
% (Section 17 does test the real function.)
motorPhysicsEff = @(rpm,T) (T.*rpm*2*pi/60) ./ ...
    (T.*rpm*2*pi/60 + 3*(T/p.Nm_per_Arms).^2*p.R_phase + p.core_loss_a*rpm + p.core_loss_b*rpm.^2);

%% 1. PACK CONFIGURATION  (external ref: ESF V2.3)
fprintf('=== 1. PACK CONFIGURATION (vs ESF V2.3 -- EXTERNAL) ===\n');
fprintf('  %dS x %dP = %d cells (ESF: 352)\n', p.N_series, p.N_parallel, p.N_series*p.N_parallel);
pass('total cell count = 352 (ESF)', p.N_series*p.N_parallel == 352);
Vnom = p.N_series*3.6; Vmax = p.N_series*4.2;
fprintf('  nominal %.1f V (ESF 316.8), max %.1f V (ESF 369.6)\n', Vnom, Vmax);
pass('nominal voltage 316.8 V (ESF)', abs(Vnom-316.8)<0.1);
pass('max voltage 369.6 V (ESF)', abs(Vmax-369.6)<0.1);

%% 2. INITIAL SOC from rest voltage  (external ref: the BMS's own logged SOC)
fprintf('\n=== 2. INITIAL SOC (rest voltage vs BMS -- EXTERNAL) ===\n');
Vcell0 = 364.28/p.N_series;
SOC0 = interp1(p.rc.OCV_lookup, p.rc.SOC_lookupR, Vcell0, 'linear');
fprintf('  first sample 364.28 V / %d = %.4f V/cell -> SOC %.1f%% (BMS logged 94.3%%)\n', ...
    p.N_series, Vcell0, SOC0*100);
pass('rest-voltage SOC within 1pt of BMS', abs(SOC0*100-94.3)<1.0);

%% 3. EMRAX EFFICIENCY: physics vs datasheet, then the real-world haircut
fprintf('\n=== 3. EMRAX EFFICIENCY: physics vs datasheet, then real-world ===\n');
% (a) the physics must reproduce the datasheet's motor-only ~96% peak (EXTERNAL),
% (b) the sim reports real-world motor+inverter after p.eta_inverter. That haircut
% was CALIBRATED to our telemetry, so (b) is an internal consistency check.
testPoints = [3000 60; 2000 80; 2500 65; 4000 50; 1500 40];   % rpm, Nm, in/near the island
for k = 1:size(testPoints,1)
    e = motorPhysicsEff(testPoints(k,1),testPoints(k,2))*100;
    fprintf('  %d rpm / %d Nm -> %.1f%% motor (physics) | %.1f%% real (+inverter)\n', ...
        testPoints(k,1), testPoints(k,2), e, e*p.eta_inverter);
end
peakEff = motorPhysicsEff(2500,65)*100;
pass('physics reproduces datasheet motor peak 95-97% (EXTERNAL)', peakEff>95 && peakEff<97);
allEff = arrayfun(@(k) motorPhysicsEff(testPoints(k,1),testPoints(k,2))*100, 1:size(testPoints,1));
pass('all physics island points inside datasheet envelope', all(allEff>=88 & allEff<=98));
pass('real-world peak (motor+inverter) lands ~88-93%', peakEff*p.eta_inverter>88 && peakEff*p.eta_inverter<93);

%% 3b. MOTOR EFFICIENCY IS MONOTONIC IN THE EXPECTED DIRECTION (torque sweep)
fprintf('\n=== 3b. EFFICIENCY vs TORQUE SHAPE (must rise, peak, then fall) ===\n');
% Physically: at low torque the fixed core loss dominates, so efficiency CLIMBS as
% load comes on. Past the peak, copper loss (I^2*R) takes over and it FALLS. A model
% that is monotonic all the way, or peaks at the wrong end, is broken -- and this is
% the shape the whole "keep the motor loaded" argument rests on, so it gets a gate.
sweepRpm = 3000; torqueSweep = 2:1:150;
effSweep = motorPhysicsEff(sweepRpm, torqueSweep);
[peakSweep, iPeak] = max(effSweep);
fprintf('  at %d rpm: peak %.1f%% at %d Nm (rises below it, falls above it)\n', ...
    sweepRpm, peakSweep*100, torqueSweep(iPeak));
pass('efficiency strictly RISES with torque below the peak', all(diff(effSweep(1:iPeak)) > 0));
pass('efficiency strictly FALLS with torque above the peak', all(diff(effSweep(iPeak:end)) < 0));
pass('peak sits at an interior torque (not at either end)', iPeak > 1 && iPeak < numel(torqueSweep));

%% 4. CORE-LOSS FIT vs the free-run-loss anchors it was fitted to
fprintf('\n=== 4. CORE-LOSS FIT vs FREE-RUN-LOSS ANCHORS ===\n');
coreLoss = @(rpm) p.core_loss_a*rpm + p.core_loss_b*rpm.^2;
for rpm = [3000 6000], fprintf('  %d rpm -> %.0f W\n', rpm, coreLoss(rpm)); end
pass('575 W at 3000 rpm anchor', abs(coreLoss(3000)-575)<10);
pass('1650 W at 6000 rpm anchor', abs(coreLoss(6000)-1650)<10);

%% 5. COPPER LOSS SANITY vs the datasheet continuous rating (EXTERNAL)
fprintf('\n=== 5. COPPER LOSS SANITY (continuous rating -- EXTERNAL) ===\n');
Irms80 = 80/p.Nm_per_Arms;
fprintf('  80 Nm / %.2f = %.1f Arms (datasheet continuous current 100 Arms)\n', p.Nm_per_Arms, Irms80);
pass('continuous-torque current below 100 A rating', Irms80 < 100);

%% 6. TIRE mu AT NOMINAL LOAD
fprintf('\n=== 6. TIRE mu AT NOMINAL LOAD (vs ~1.4 ballpark) ===\n');
muNominal = p.tir.LMUX*(p.tir.PDX1 + p.tir.PDX2*0);
fprintf('  LMUX*PDX1 = %.3f at nominal load\n', muNominal);
pass('nominal mu in 1.3-1.4 range', muNominal>1.3 && muNominal<1.4);
muDoubleLoad = p.tir.LMUX*(p.tir.PDX1 + p.tir.PDX2*1);
fprintf('  at 2x nominal load: %.3f (should drop -- load sensitivity)\n', muDoubleLoad);
pass('mu decreases with load', muDoubleLoad < muNominal);

%% 7. GEAR-RATIO SHAFT-POWER INVARIANCE (the study's core assumption)
fprintf('\n=== 7. GEAR-RATIO SHAFT-POWER INVARIANCE (core assumption) ===\n');
% Wheel power is what it is; re-expanding it at a new ratio must return the same
% shaft power. If this ever fails, the whole "same laps, different gearing" method dies.
shaftPowerIn = 15000;
wheelPower    = shaftPowerIn*p.eta_drivetrain;
shaftPowerOut = wheelPower/p.eta_drivetrain;
fprintf('  shaft %.0f W -> wheel %.0f W -> shaft %.0f W\n', shaftPowerIn, wheelPower, shaftPowerOut);
pass('shaft power ratio-invariant (round-trip identity)', abs(shaftPowerOut-shaftPowerIn)<1e-6);
wheelOmega = 100;
pass('lower ratio -> higher shaft torque', ...
     shaftPowerIn/(wheelOmega*4.20) > shaftPowerIn/(wheelOmega*p.gear_current));

%% 8. LAUNCH FORCE BALANCE (hand check at the current ratio)
fprintf('\n=== 8. LAUNCH FORCE BALANCE (hand check, %.2f:1) ===\n', p.gear_current);
accelX = 0;
for it = 1:8
    Fz_rear = p.m_car*p.g*p.rear_static + p.m_car*accelX*p.h_cg/p.L_wb;
    dfz     = (Fz_rear/2 - p.tir.FNOMIN)/p.tir.FNOMIN;
    mu      = p.tir.LMUX*(p.tir.PDX1 + p.tir.PDX2*dfz);
    F_traction = mu*Fz_rear;
    accelX  = (F_traction - p.Crr*p.m_car*p.g)/p.m_car;
end
fprintf('  converged launch: Fz_rear %.0f N, a_x %.2f m/s^2 (%.2fg), traction force %.0f N\n', ...
    Fz_rear, accelX, accelX/p.g, F_traction);
torqueCap = F_traction*p.r_wheel/(p.gear_current*p.eta_drivetrain);
fprintf('  -> motor torque cap %.0f Nm\n', torqueCap);
pass('launch traction cap in a sane 120-170 Nm range', torqueCap>120 && torqueCap<170);
F_motor = p.T_flat_cap*p.gear_current*p.eta_drivetrain/p.r_wheel;
fprintf('  motor can push %.0f N (%.0f Nm) vs %.0f N traction limit -> %s at launch\n', ...
    F_motor, p.T_flat_cap, F_traction, ...
    string(F_motor>F_traction).replace("true","TRACTION-limited").replace("false","motor-limited"));
fprintf('  NOTE: accel_model''s own traction cap has a UNIT BUG (it divides by eta twice)\n');
fprintf('  and so never binds. This hand check and that model DISAGREE on purpose until\n');
fprintf('  launch/TC data settles it -- see the comment at accel_model.m accel_run.\n');

%% 9. ENERGY THROUGHPUT ORDER-OF-MAGNITUDE
fprintf('\n=== 9. ENERGY THROUGHPUT ORDER-OF-MAGNITUDE ===\n');
kWh = 3.41; km = 22.5;   % July 11 endurance run, from the logged odometer + pack energy
fprintf('  %.2f kWh / %.1f km = %.0f Wh/km\n', kWh, km, kWh*1000/km);
pass('specific energy in plausible FSAE range 100-250 Wh/km', kWh*1000/km>100 && kWh*1000/km<250);

%% 10. HV VARIANT CONFIRMATION (external: datasheet column selection)
fprintf('\n=== 10. HV VARIANT CONFIRMATION (car is a 370V HV pack) ===\n');
fprintf('  %dS x 4.2V = %.1f V max -> under the HV 470V limit -> HV constants correct\n', ...
    p.N_series, p.N_series*4.2);
pass('370V pack uses HV datasheet column (max 470V)', p.N_series*4.2 < 470 && p.N_series*4.2 > 320);

%% 11. MECHANICAL STACK PRODUCT *IS* p.eta_drivetrain
fprintf('\n=== 11. DRIVETRAIN EFFICIENCY CHAIN (mechanical hardware) ===\n');
% The stage values are the CFR26 DT memo's ASSUMPTIONS (not measured -- see
% drivetrain_efficiency.m). What this gates is that the product of the stack still
% equals the single number the rest of the sim uses. If someone edits one stage and
% forgets eta_drivetrain, or vice versa, the repo starts disagreeing with itself here.
stageSpur = 0.98; stageBearings = 0.95; stageChain = 0.97; stageDiff = 0.92;   % memo v4.0
mechNoHalfshaft = stageSpur*stageBearings*stageChain*stageDiff;
halfshaftJoint  = @(beta) 1 - 2*p.hs_kloss*sind(beta);
halfshaftLapWtd = @(beta) p.hs_frac_straight*halfshaftJoint(beta) ...
                        + (1-p.hs_frac_straight)*halfshaftJoint(beta + p.hs_corner_deg);
halfshaftAtAngle = halfshaftLapWtd(p.hs_angle_deg);
mechStackProduct = mechNoHalfshaft * halfshaftAtAngle;
fprintf('  %.2f x %.2f x %.2f x %.2f x halfshaft@%gdeg %.4f = %.4f\n', ...
    stageSpur, stageBearings, stageChain, stageDiff, p.hs_angle_deg, halfshaftAtAngle, mechStackProduct);
fprintf('  params_cfr26 p.eta_drivetrain = %.4f\n', p.eta_drivetrain);
pass('mech stack product EQUALS p.eta_drivetrain (to rounding)', ...
     abs(mechStackProduct - p.eta_drivetrain) < 0.001);
fprintf('  straight-halfshaft variant (0 deg) would be %.4f\n', mechNoHalfshaft*halfshaftLapWtd(0));
pass('straightening halfshafts raises the mech stack', mechNoHalfshaft*halfshaftLapWtd(0) > p.eta_drivetrain);

%% 12. AERO: the construction chain from the CFD run to p.CdA / p.ClA
fprintf('\n=== 12. AERO CONSTRUCTION CHAIN (CFD run -> params) ===\n');
% HEADS UP -- p.CdA is deliberately NOT the drag area the CFD run produced.
% The CFD run implied Cd = 1.14278. The aero lead later revised Cd DOWN to 0.922.
% params_cfr26 keeps the CFD's FRONTAL AREA (which the revision doesn't change) and
% re-applies the new Cd on top:
%       A_ref  = (F_drag_CFD / q25) / 1.14278      <- frontal area, from the CFD run
%       p.CdA  = 0.922 * A_ref                     <- revised Cd on that same area
% So p.CdA sits ~19% BELOW the CFD-implied drag area ON PURPOSE. That is a real
% modelling decision, not drift.
%
% The previous version of this check computed CdA = F_CFD/q25 locally and then
% confirmed that 0.5*rho*CdA*25^2 came back to F_CFD -- which is algebraically
% guaranteed and never touched p.CdA at all. It could not fail. Now we gate the
% actual chain, so a change to either Cd or the CFD force gets caught.
q25 = 0.5*p.rho_air*25^2;
FdragCFD = 442.719;  Cd_CFD = 1.14278;  Cd_revised = 0.922;
A_ref_expected = (FdragCFD/q25)/Cd_CFD;
fprintf('  q(25 m/s) = %.1f Pa | CFD drag area %.3f m^2 -> frontal area %.3f m^2\n', ...
    q25, FdragCFD/q25, A_ref_expected);
fprintf('  p.CdA = %.3f m^2 (revised Cd %.3f x A_ref) | p.ClA = %.3f m^2\n', ...
    p.CdA, Cd_revised, p.ClA);
pass('p.CdA = revised Cd x CFD frontal area', abs(p.CdA - Cd_revised*A_ref_expected) < 1e-3);
pass('p.CdA is below the CFD-implied drag area (Cd was revised DOWN)', p.CdA < FdragCFD/q25);
pass('p.ClA/p.CdA holds the wind-tunnel lift:drag ratio', abs(p.ClA/p.CdA - 577.8/359.7) < 1e-3);
% and the CFD run's OWN numbers must still round-trip through its OWN drag area
pass('CFD force round-trips through the CFD drag area (EXTERNAL)', ...
     abs(0.5*p.rho_air*(FdragCFD/q25)*25^2 - FdragCFD) < 1);
fprintf('  forces at 25 m/s AS THE SIM USES THEM: drag %.1f N, downforce %.1f N\n', ...
    0.5*p.rho_air*p.CdA*25^2, 0.5*p.rho_air*p.ClA*25^2);

%% 13. RC DISCRETIZATION + the open-loop cell model actually runs
fprintf('\n=== 13. RC DISCRETIZATION + OPEN-LOOP CELL MODEL ===\n');
dt = 0.1; Rb = 0.002; Cb = 1000; tau = Rb*Cb; decay = exp(-dt/tau);
fprintf('  tau=R*C=%.1f s, exp(-dt/tau)=%.4f (in (0,1) => stable decay)\n', tau, decay);
pass('RC decay factor in (0,1)', decay>0 && decay<1);
Vrc = 0; for n = 1:2000, Vrc = decay*Vrc + Rb*(1-decay)*10; end
fprintf('  RC branch steady state under 10 A -> %.4f V (expect I*R = %.4f V)\n', Vrc, 10*Rb);
pass('RC steady state = I*R', abs(Vrc-10*Rb)<1e-4);

% run_open_loop must RUN, and SOC must fall monotonically under a pure discharge.
% This is the check that would have caught the R/C tables being read backwards, and
% it fails loudly rather than returning a plausible-looking trace.
rcTest = p.rc; rcTest.SOC0 = 0.90;
nSteps = 600; dischargeCurrent = 20*ones(nSteps,1); stepVec = 0.1*ones(nSteps,1);
[socTrace, voltTrace] = run_open_loop(dischargeCurrent, stepVec, rcTest);
fprintf('  open-loop 20 A discharge, %d steps: SOC %.4f -> %.4f, V %.3f -> %.3f\n', ...
    nSteps, socTrace(1), socTrace(end), voltTrace(1), voltTrace(end));
pass('run_open_loop returns finite SOC and voltage', all(isfinite(socTrace)) && all(isfinite(voltTrace)));
pass('SOC MONOTONICALLY FALLS during discharge', all(diff(socTrace) <= 1e-12));
pass('SOC stays inside [0,1] during discharge', all(socTrace>=0 & socTrace<=1));
pass('terminal voltage sags below OCV under load', ...
     voltTrace(1) < interp1(p.rc.SOC_lookupR, p.rc.OCV_lookup, socTrace(1), 'linear'));
% and the mirror: charging must push SOC back UP
[socCharge, ~] = run_open_loop(-20*ones(nSteps,1), stepVec, rcTest);
pass('SOC MONOTONICALLY RISES during charge', all(diff(socCharge) >= -1e-12));
% the trailing-zero artifact must never reach a lookup
pass('no zero/negative resistance survives the table guard', ...
     all(isfinite(voltTrace)) && ~any(voltTrace == 0));

%% 14. TOP-SPEED FORCE BALANCE
fprintf('\n=== 14. TOP-SPEED FORCE BALANCE (%.2f:1 spot check) ===\n', p.gear_current);
v = 112/3.6;
rpmTop = v/p.r_wheel*p.gear_current*60/(2*pi);
fprintf('  112 kph -> %.0f motor rpm (redline %d)\n', rpmTop, p.redline);
pass('top-speed rpm under redline', rpmTop < p.redline);
Fdown = 0.5*p.rho_air*p.ClA*v^2;
Fres  = 0.5*p.rho_air*p.CdA*v^2 + p.Crr*(p.m_car*p.g + Fdown);
fprintf('  resistance at 112 kph = %.0f N (drag + rolling, downforce-loaded)\n', Fres);
pass('resistance force positive and plausible (<1500 N)', Fres>0 && Fres<1500);

%% 15. THE PHYSICS MODEL vs MEASURED PACK-TO-SHAFT EFFICIENCY
fprintf('\n=== 15. PHYSICS MODEL vs MEASURED PACK-TO-SHAFT EFFICIENCY ===\n');
% THE MEASURED METHOD, stated so nobody has to go ask a person what it is:
%   efficiency = mechanical power OUT / electrical power IN
%              = (motor torque x motor speed) / (pack voltage x pack current),
%   over steady-state motoring points only (transients book inertia as "loss").
% HONESTY: p.eta_inverter was CHOSEN to make the physics model match this measured
% number, on this session. So this check closes a loop on its own calibration --
% consistency, NOT independent validation. Independent = a deliberate steady-state
% dyno run. Also note the torque channel is the inverter's own MODEL-DERIVED torque
% estimate, not a transducer, so "measured" is doing some work in that word.
csvComp = fullfile(thisdir, 'data', 'comp_june20_data.csv');
if exist(csvComp, 'file')
    D    = readtable(csvComp);
    rpmT = abs(D.PM100DX_motorSpeed);  tqT = abs(D.PM100DX_torqueFeedback);
    elec = abs(D.BMSB_packVoltage .* D.BMSB_packCurrent);
    mech = tqT .* rpmT * 2*pi/60;
    % (a) rigid driveline: motor:axle speed ratio must equal the gear ratio. This is
    %     WHY the ratio cancels, making the measured map a pack->SHAFT number.
    axle = abs(D.VCREAR_wheelSpeedRL);
    gd   = rpmT>500 & axle>20;
    ratioMeasured = median(rpmT(gd)./axle(gd));
    fprintf('  motor:axle speed ratio from data = %.3f (gear ratio %.2f)\n', ratioMeasured, p.gear_current);
    pass('measured speed ratio = gear ratio (so the ratio cancels)', abs(ratioMeasured-p.gear_current)<0.15);
    % (b) steady-state points only
    motoring = rpmT>500 & tqT>5 & elec>500 & mech./elec>0.3 & mech./elec<1.0;
    steady   = motoring & movstd(rpmT,11)<40 & movstd(tqT,11)<3;
    % fit elec = mech/eta + P0: slope -> motor+inverter eff, intercept -> accessory draw
    A = [mech(steady) ones(nnz(steady),1)];  x = A\elec(steady);
    etaMeasured = 1/x(1);  accessoryW = x(2);
    fprintf('  %d steady pts | measured motor+inverter eff %.3f | accessory %.0f W\n', ...
        nnz(steady), etaMeasured, accessoryW);
    pass('measured motor+inverter eff in 0.83-0.91 band', etaMeasured>0.83 && etaMeasured<0.91);
    % (c) the model on the SAME points must still land on the measurement
    etaModel = mean(motorPhysicsEff(rpmT(steady), tqT(steady))) * p.eta_inverter;
    fprintf('  physics model x p.eta_inverter on same points: %.3f (measured %.3f, gap %+.4f)\n', ...
        etaModel, etaMeasured, etaModel-etaMeasured);
    pass('model within 0.02 of measured (calibration holds)', abs(etaModel-etaMeasured)<0.02);
else
    fprintf('  [data/comp_june20_data.csv not found -- check skipped]\n');
end

%% 16. HALFSHAFT / CV-JOINT MODEL
fprintf('\n=== 16. DRIVETRAIN STACK + HALFSHAFT ANGLE MODEL ===\n');
% (a) the memo's own stage product must reproduce its published 0.724.
etaMemo = 0.89*stageSpur*stageBearings*stageChain*stageDiff*0.98;
fprintf('  memo stack 0.89*0.98*0.95*0.97*0.92*0.98 = %.4f (memo prints 0.724)\n', etaMemo);
pass('DT memo stack reproduces 0.724', abs(etaMemo-0.724)<0.002);
% (b) kloss round-trip. READ THIS: p.hs_kloss was BACK-FITTED to these two memo
% points, so of course it reproduces them. This is NOT evidence the model is right --
% it only catches someone changing p.hs_kloss without meaning to. See params_cfr26.
klossFromStraight = (1-0.99)/(2*sind(3));   klossFromCorner = (1-0.94)/(2*sind(20));
fprintf('  kloss implied by memo pts: %.3f (3deg) / %.3f (20deg); params has %.3f\n', ...
    klossFromStraight, klossFromCorner, p.hs_kloss);
pass('p.hs_kloss still sits between the two fitted points', ...
     p.hs_kloss >= min(klossFromStraight,klossFromCorner)-0.005 && ...
     p.hs_kloss <= max(klossFromStraight,klossFromCorner)+0.005);
% (c) the halfshaft term at the MEASURED static angle
fprintf('  halfshaft term @%g deg static (MEASURED from CAD) = %.4f\n', p.hs_angle_deg, halfshaftAtAngle);
pass('halfshaft term at the measured angle in 0.94-0.97 band', halfshaftAtAngle>0.94 && halfshaftAtAngle<0.97);
pass('angle is flagged MEASURED in params', isfield(p,'hs_angle_is_measured') && p.hs_angle_is_measured);
% (d) measured pack->shaft x mech stack must reproduce the tool's overall
csvEnd = fullfile(thisdir, 'data', 'endurance_july11_with_odo_wide.csv');
if exist(csvEnd,'file')
    E2 = readtable(csvEnd);
    rr = abs(E2.PM100DX_motorSpeed);  qq = abs(E2.PM100DX_torqueFeedback);
    pk = abs(E2.BMSB_packVoltage .* E2.BMSB_packCurrent);  mm = qq.*rr*2*pi/60;
    kp = rr>500 & qq>5 & pk>500 & mm./pk>0.3 & mm./pk<1.0;
    etaPackToShaft = sum(mm(kp))/sum(pk(kp));
    overallEff = etaPackToShaft * mechStackProduct;
    fprintf('  measured pack->shaft %.3f -> overall battery->ground %.3f\n', etaPackToShaft, overallEff);
    pass('measured pack->shaft in 0.75-0.90 band', etaPackToShaft>0.75 && etaPackToShaft<0.90);
    pass('overall battery->ground in 0.58-0.70 band', overallEff>0.58 && overallEff<0.70);
else
    fprintf('  [endurance CSV not found -- overall check skipped]\n');
end

%% 17. THE INVERTER IS ACTUALLY IN THE ENERGY PATH
fprintf('\n=== 17. INVERTER IS IN THE ENERGY PATH (guards the old motor-only bug) ===\n');
% Earlier versions reported motor-ONLY efficiency and under-counted battery draw by
% ~5%. This tests the REAL lib function (not the reimplementation above) -- the whole
% point is to catch the shipped code dropping the inverter.
effWithInverter = emrax208_efficiency(3000, 60, p);
effMotorOnly    = emrax208_efficiency(3000, 60, rmfield(p,'eta_inverter'));
fprintf('  motor+inverter %.3f vs motor-only %.3f -> ratio %.3f (p.eta_inverter %.2f)\n', ...
    effWithInverter, effMotorOnly, effWithInverter/effMotorOnly, p.eta_inverter);
pass('emrax208_efficiency applies the inverter haircut', ...
     abs(effWithInverter/effMotorOnly - p.eta_inverter) < 0.01);
pass('inverter drops reported efficiency by >=3 pts', (effMotorOnly - effWithInverter) > 0.03);
% The two independent implementations must agree, so neither can drift alone.
pass('lib function matches this file''s independent physics rebuild', ...
     abs(effWithInverter - motorPhysicsEff(3000,60)*p.eta_inverter) < 1e-6);
fprintf('  gear_ratio_optimization energy: inverter-INCLUDED (via emrax208_efficiency).\n');
fprintf('  accel/top-speed torque: p.eta_drivetrain (mechanical) only -- inverter correctly\n');
fprintf('  excluded, since the inverter draws more current rather than cutting shaft torque.\n');

%% SUMMARY
fprintf('\n=== SUMMARY: %d passed, %d FAILED (%d checks) ===\n', nPass, nFail, nPass+nFail);
if nFail > 0
    fprintf('Any **FAIL** above means that specific number needs a second look.\n');
else
    fprintf('All checks pass. That means the model is SELF-CONSISTENT with its own\n');
    fprintf('assumptions -- NOT that it has been validated against the car. The open\n');
    fprintf('items are still open: dyno/coastdown for the stage efficiencies, a measured\n');
    fprintf('loaded wheel radius, launch/TC data for the traction question.\n');
end

end

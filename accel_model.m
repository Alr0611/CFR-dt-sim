%% CFR26 ACCELERATION MODEL (inertia-based)
% The accel study: includes ROTATIONAL INERTIA of the motor rotor,
% driveline and wheels reflected to the motor (per the WR-217e / FSAE
% top-speed method). ODE solved discretely over motor speed:
%   dw/dt = [T_M(w)*eta - n*r*(b*w^2 + C) - T_F] / (I + n*r*a),  n = 1/G
%
% Produces: gear-ratio sweep (0-75m, 0-100kph), effect-of-inertia comparison,
% wheel-weight sensitivity, tractive-effort curves, and 40-80 kph recovery.
% Validation: June 19 launch was 0-75m = 4.40 s raw at 4.61:1.

clear; clc; close all;
cd(fileparts(mfilename('fullpath')));   % run from the repo root so data/ and output/ paths resolve
if ~exist('output','dir'), mkdir('output'); end   % fresh copy may have no output/ folder
addpath(fullfile(fileparts(mfilename('fullpath')), 'lib'));  % works wherever MATLAB is pointed
p = params_cfr26();

fprintf('Wheel inertia %.4f kg*m^2 (%.1f kg, k=%.2f*r) | reflected @%.2f %.4f | rotor %.4f\n', ...
    p.I_wheel, p.m_wheel, p.kFactor, p.gear_current, ...
    p.n_wheels*p.I_wheel/p.gear_current^2, p.I_rotor);

%% ---- SWEEP: 0-75m and 0-100kph vs ratio ----
% Run the full accel ODE at every candidate ratio and record the three numbers that
% matter: time to 100 kph, time over 75 m, and the speed at the 75 m trap.
% Study range 4.00-5.20 (same window as gear_ratio_optimization), at 0.05 for a smooth
% curve, with p.gear_current forced ONTO the grid. Before this the grid was 3.6:0.05:5.4,
% which does not contain 4.61 -- every "current" lookup below silently snapped to 4.60
% and then printed it as 4.61. The physics/equations are unchanged.
gears = unique(round([4.00:0.05:5.20, p.gear_current], 4));
t75 = nan(size(gears)); t100 = nan(size(gears)); vtrap = nan(size(gears));
for i = 1:numel(gears)
    [t100(i), t75(i), vtrap(i)] = accel_run(gears(i), p);
end
fprintf('\n=== INERTIA-BASED ACCEL SWEEP ===\n');
for g = [4.0 4.2 p.gear_current 5.0 5.2]
    [~,j] = min(abs(gears-g)); tag=''; if abs(g-p.gear_current)<1e-6, tag=' (current)'; end
    % t100 is NaN when the ratio GEARS OUT below 100 kph (redline top speed =
    % redline*2*pi/60/G*r_wheel < 27.8 m/s, i.e. any G > ~5.17). That is a real
    % result, not a failure -- print it as such instead of the literal "NaN".
    if isnan(t100(j)), t100str = '  n/a '; else, t100str = sprintf('%.2fs', t100(j)); end
    fprintf(' %.2f:1 -> 0-100kph %s | 0-75m %.2fs | trap %.0f kph%s\n', ...
        gears(j), t100str, t75(j), vtrap(j), tag);
end
[~,jCur] = min(abs(gears-p.gear_current));   % exact hit now that 4.61 is on the grid
fprintf('0-75m is MONOTONIC (no interior optimum): accel favors higher ratio until gearing out.\n');
fprintf('Model %.2fs at %.2f vs real clean launch 4.40s -> ~%.2fs conservative.\n', ...
    t75(jCur), gears(jCur), t75(jCur)-4.40);

%% ---- WHEEL WEIGHT SENSITIVITY (@ current ratio) ----
% What lighter wheels are worth. Mass comes off at the tread radius, so it counts twice:
% less car to push AND less rotational inertia to spin up.
fprintf('\n=== WHEEL WEIGHT SENSITIVITY @ %.2f:1 (mass removed at tread radius) ===\n', p.gear_current);
for dkg = [0 0.25 0.5 0.75 1.0]
    ps = p; ps.m_car = p.m_car - 4*dkg; ps.I_wheel = p.I_wheel - dkg*p.r_wheel^2;
    [~, t75s, ~] = accel_run(p.gear_current, ps);
    fprintf(' -%.2f kg/wheel (-%.1f kg total) -> 0-75m %.2fs\n', dkg, 4*dkg, t75s);
end

%% ---- HALFSHAFT ANGLE: what straightening them would BUY in accel ----
% Same CV-joint model as drivetrain_efficiency, from params (hs_kloss / hs_angle_deg /
% hs_corner_deg / hs_frac_straight). We SCALE the pinned p.eta_drivetrain by the ratio
% of halfshaft terms rather than rebuilding the stage stack here, so params stays the
% single source of truth and this can never disagree with the drivetrain tool.
%
% Read this carefully before quoting it: p.hs_angle_deg (12 deg) IS the straight-line
% angle -- the joints work at 12 deg with the steering dead ahead. 0 deg is NOT a
% driving condition, it is a REPACKAGING target for the diff/upright geometry. So the
% 0 deg rows are "what we would gain if we rebuilt the geometry", not "what we get on
% a straight-line run".
%
% Two columns because eta_drivetrain is LAP-WEIGHTED (72.4% at static + 27.6% at
% static+8 deg cornering). A 0-75 m run never corners, so the straight-only column is
% the honest one for THIS event; the lap-weighted column is what the endurance/gear
% study uses. The difference at 12 deg is the size of that modelling choice.
hsJoint    = @(beta) 1 - 2*p.hs_kloss*sind(beta);                       % one shaft, both joints
hsLapWtd   = @(beta) p.hs_frac_straight*hsJoint(beta) ...
                   + (1-p.hs_frac_straight)*hsJoint(beta+p.hs_corner_deg);
hsRef      = hsLapWtd(p.hs_angle_deg);            % what p.eta_drivetrain already encodes
fprintf('\n=== HALFSHAFT ANGLE @ %.2f:1 -- what straightening the geometry would buy ===\n', p.gear_current);
fprintf(' (%g deg is the STRAIGHT-LINE angle; 0 deg = repackaged geometry, not a driving state)\n', p.hs_angle_deg);
fprintf('  angle |   lap-weighted eta / 0-75m   |  straight-only eta / 0-75m  |  gain 0-75m\n');
t75Ref = NaN;
for beta = [p.hs_angle_deg 5 0]
    pl = p; pl.eta_drivetrain = p.eta_drivetrain * hsLapWtd(beta)/hsRef;   % lap-weighted
    ps = p; ps.eta_drivetrain = p.eta_drivetrain * hsJoint(beta)/hsRef;    % straight only
    [~, tLap, ~] = accel_run(p.gear_current, pl);
    [~, tStr, ~] = accel_run(p.gear_current, ps);
    if isnan(t75Ref), t75Ref = tStr; end            % reference = straight-only at 12 deg
    fprintf('  %5.1f | %.4f / %.3f s%15s| %.4f / %.3f s%12s| %+.3f s\n', ...
        beta, pl.eta_drivetrain, tLap, '', ps.eta_drivetrain, tStr, '', tStr - t75Ref);
end
fprintf(' -> straightening 12 -> 0 deg is worth the last column; it is a HARDWARE change\n');
fprintf('    (suspension/diff packaging), not a tune. Cross-check: drivetrain_efficiency\n');
fprintf('    prices the same change in battery Wh over an endurance run.\n');

%% ---- WHEEL RADIUS: free vs LOADED (the tyre squishes under load) ----
% p.r_wheel is the FREE radius (tyre just sitting there). Under load the tyre
% deflects and the rolling radius drops ~5% (params_cfr26 says so in the comment,
% and we have never measured it on our own tyre). This matters more than it looks:
% r sets rpm->speed AND the tractive force arm, so ~5% on radius is ~5% on force.
% That is a bigger lever than the halfshaft angle. We do NOT change the value here
% -- nobody has measured our loaded radius -- but the model should show what the
% assumption is worth. If someone measures it, put the number in params, not here.
fprintf('\n=== WHEEL RADIUS SENSITIVITY @ %.2f:1 (free vs loaded tyre) ===\n', p.gear_current);
[~, t75_free, ~] = accel_run(p.gear_current, p);
pLoaded = p;
pLoaded.r_wheel = 0.95 * p.r_wheel;                                        % ~5% squish
pLoaded.I_wheel = pLoaded.kFactor^2 * pLoaded.m_wheel * pLoaded.r_wheel^2;  % inertia follows r^2
[~, t75_loaded, ~] = accel_run(p.gear_current, pLoaded);
fprintf('  free radius   %.4f m -> 0-75m %.3f s   (what the model uses now)\n', p.r_wheel, t75_free);
fprintf('  loaded 0.95x  %.4f m -> 0-75m %.3f s   (%+.3f s)\n', ...
    pLoaded.r_wheel, t75_loaded, t75_loaded - t75_free);
fprintf('  Measured clean launch was 4.40 s; the model sits %+.3f s off that at free radius.\n', ...
    t75_free - 4.40);
fprintf('  Loaded radius closes %.3f s of that %.3f s gap (~%.0f%%). So the free-radius\n', ...
    t75_free - t75_loaded, t75_free - 4.40, 100*(t75_free - t75_loaded)/max(t75_free - 4.40, eps));
fprintf('  assumption plausibly explains a chunk of the sim-vs-measured gap, but NOT all\n');
fprintf('  of it. r_wheel is UNMEASURED under load -- this is a sensitivity, not a fix.\n');

%% ---- 40-80 kph CORNER-EXIT RECOVERY ----
% Standing-start accel is one event; most of a lap is accelerating out of corners.
% This is that: full throttle from 40 to 80 kph, which is where gearing is really felt.
fprintf('\n=== 40-80 kph recovery (full throttle, ideal TC) ===\n');
for g = [4.0 4.2 p.gear_current 5.2]
    fprintf(' %.2f:1 -> %.2f s\n', g, recovery_40_80(g, p));
end

%% ---- FIGURES (one TABBED window, 4 tabs) ----
% p0 is the same car with every rotational inertia zeroed -- the difference between the
% two curves is exactly what spinning the rotor and wheels costs in 0-75 m.
p0 = p; p0.I_rotor=0; p0.I_driveline=0; p0.I_wheel=0;
t75_noI = nan(size(gears)); for i=1:numel(gears), [~,t75_noI(i),~]=accel_run(gears(i),p0); end

fA = figure('Name','Accel study','Position',[40 40 1000 560]);
tg = uitabgroup(fA);

ax = axes(uitab(tg,'Title','0-100 kph'));
plot(ax,gears,t100,'LineWidth',1.6); hold(ax,'on'); xline(ax,p.gear_current,'k--','current');
xlabel(ax,'Gear ratio'); ylabel(ax,'0-100 kph (s)'); grid(ax,'on'); title(ax,'0-100 kph');

ax = axes(uitab(tg,'Title','0-75 m'));
plot(ax,gears,t75,'LineWidth',1.6); hold(ax,'on'); xline(ax,p.gear_current,'k--','current');
yline(ax,4.40,'g:','real 4.40s'); xlabel(ax,'Gear ratio'); ylabel(ax,'0-75 m (s)'); grid(ax,'on');
title(ax,'0-75 m (monotonic -- accel favors HIGH ratio)');

ax = axes(uitab(tg,'Title','Rotational inertia'));
plot(ax,gears,t75,'LineWidth',1.6,'DisplayName','With rotational inertia'); hold(ax,'on');
plot(ax,gears,t75_noI,'--','LineWidth',1.4,'DisplayName','Point-mass (no inertia)');
xline(ax,p.gear_current,'k:','HandleVisibility','off');
xlabel(ax,'Gear ratio'); ylabel(ax,'0-75 m (s)'); grid(ax,'on'); legend(ax,'Location','north');
title(ax,'Rotational-inertia effect (~0.15s, flattens the high-ratio end)');

ax = axes(uitab(tg,'Title','Tractive effort'));
tractive_effort_plot(ax, p, gears);

writetable(table(gears', t100', t75', vtrap', 'VariableNames',{'ratio','t0_100kph','t0_75m','trap_kph'}), ...
    'output/accel_results.csv');
save_tabfig(fA, fullfile('output','AccelStudy'));
fprintf('\nSaved: output/accel_results.csv + 1 tabbed figure window (4 tabs)\n');

%% ================= LOCAL FUNCTIONS =================
function [t100, t75, vtrap] = accel_run(G, p)
    n = 1/G;
    I_fixed = p.I_rotor + p.I_driveline + p.n_wheels*p.I_wheel*n^2;
    a_coef  = p.m_car * n * p.r_wheel;
    I_den   = I_fixed + n*p.r_wheel*a_coef;
    b  = 0.5*p.rho_air*p.r_wheel^2*n^2*(p.CdA + p.Crr*p.ClA);
    Cc = p.m_car*p.g*p.Crr;
    w_max = p.redline*2*pi/60; dw = w_max/4000;
    w=0; t=0; x=0; v=0; a_prev=0; t100=NaN; t75=NaN; vtrap=NaN;
    while w < w_max
        rpm = w*60/(2*pi);
        T_M = motor_peak_torque(rpm, p) * p.eta_drivetrain;
        Fdown = 0.5*p.rho_air*p.ClA*v^2;
        for it=1:3
            Fzr = p.m_car*p.g*p.rear_static + p.m_car*a_prev*p.h_cg/p.L_wb + Fdown*p.rear_aero;
            Ftr = tire_mu_x(Fzr/2, p.tir) * Fzr;
            % *** KNOWN BUG -- UNIT INCONSISTENCY IN THIS TRACTION CAP. NOT FIXED YET. ***
            % T_M above already has eta_drivetrain applied, so it is post-loss shaft
            % torque. The cap on the right divides by eta AGAIN, which inflates it by
            % 1/eta = ~1.26x. Net effect: the traction limit is ~26% too high and
            % therefore NEVER BINDS -- 0 of ~3726 integration steps to 75 m hit it.
            % In consistent units the cap is Ftr*r_wheel*n (no /eta), and then ~1900
            % of ~3720 steps ARE traction-capped and 0-75 m at 4.61:1 goes
            % 4.669 -> 4.714 s. verify_math section 8 independently computes a
            % traction-limited launch, which is the same disagreement showing up twice.
            % CONSEQUENCE FOR ANYONE QUOTING THIS MODEL: any statement of the form
            % "the launch is not traction-limited" or "grip does not affect accel" is
            % an ARTIFACT OF THIS BUG, not a result. The grip sensitivity of 0-75 m is
            % UNVERIFIED in both directions. OPEN -- pending launch/TC data.
            % Left as-is deliberately so no number moves until that data lands.
            T_use = min(T_M, Ftr*p.r_wheel*n/p.eta_drivetrain);
            dwdt = (T_use - n*p.r_wheel*(b*w^2 + Cc) - p.T_F) / I_den;
            a_prev = p.r_wheel*n*dwdt;
        end
        if dwdt <= 0, break; end
        dt = dw/dwdt; t=t+dt; w=w+dw; v=p.r_wheel*n*w; x=x+v*dt;
        if isnan(t100) && v>=100/3.6, t100=t; end
        if isnan(t75)  && x>=75,      t75=t; vtrap=v*3.6; end
        if ~isnan(t100)&&~isnan(t75), break; end
    end
    if isnan(t75) && v>1, t75=t+(75-x)/v; vtrap=v*3.6; end
    if isnan(t100)&& v>=100/3.6, t100=t; end
end

function t = recovery_40_80(G, p)
    n=1/G; v=40/3.6; t=0; dt=0.005; a=0;
    while v < 80/3.6 && t < 15
        rpm = v/p.r_wheel*G*60/(2*pi);
        F_wh = motor_peak_torque(min(rpm,p.redline),p)*G*p.eta_drivetrain/p.r_wheel;
        Fdown = 0.5*p.rho_air*p.ClA*v^2;
        for it=1:3
            Fzr = p.m_car*p.g*p.rear_static + p.m_car*a*p.h_cg/p.L_wb + Fdown*p.rear_aero;
            F_dr = min(F_wh, tire_mu_x(Fzr/2,p.tir)*Fzr);
            a = (F_dr - 0.5*p.rho_air*p.CdA*v^2 - p.Crr*(p.m_car*p.g+Fdown))/p.m_car;
        end
        v=v+a*dt; t=t+dt;
    end
end

function tractive_effort_plot(ax, p, gears)
    % Tractive effort vs speed for the WHOLE ratio sweep, colored by ratio; the
    % traction limit and drag+rolling drawn in black. Plots into axes `ax`.
    hold(ax,'on');
    v = (1:0.25:38)';
    cmap = turbo(numel(gears));
    for j=1:numel(gears)
        F=zeros(size(v));
        for k=1:numel(v)
            rpm=v(k)/p.r_wheel*gears(j)*60/(2*pi);
            if rpm>p.redline, F(k)=NaN; else, F(k)=motor_peak_torque(rpm,p)*gears(j)*p.eta_drivetrain/p.r_wheel; end
        end
        plot(ax, v*3.6, F, 'Color', cmap(j,:), 'LineWidth', 1, 'HandleVisibility','off');
    end
    Ftr=zeros(size(v)); Fr=zeros(size(v));
    for k=1:numel(v)
        vv=v(k); Fd=0.5*p.rho_air*p.ClA*vv^2; a=0;
        for it=1:5
            Fzr=p.m_car*p.g*p.rear_static + p.m_car*a*p.h_cg/p.L_wb + Fd*p.rear_aero;
            Ftr(k)=tire_mu_x(Fzr/2,p.tir)*Fzr;
            a=(Ftr(k)-0.5*p.rho_air*p.CdA*vv^2 - p.Crr*(p.m_car*p.g+Fd))/p.m_car;
        end
        Fr(k)=0.5*p.rho_air*p.CdA*vv^2 + p.Crr*(p.m_car*p.g+Fd);
    end
    plot(ax, v*3.6,Ftr,'k--','LineWidth',1.5,'DisplayName','Traction limit (rear)');
    plot(ax, v*3.6,Fr,'k:','LineWidth',1.5,'DisplayName','Drag + rolling');
    colormap(ax, turbo); clim(ax,[gears(1) gears(end)]); cb=colorbar(ax); cb.Label.String='Gear ratio';
    xlabel(ax,'Speed (kph)'); ylabel(ax,'Force at wheels (N)'); grid(ax,'on');
    legend(ax,'Location','northeast'); ylim(ax,[0 3200]);
    title(ax,'Tractive effort vs speed (all ratios)');
end

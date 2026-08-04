%% START -- open this file, hit Run (F5). That's it. That's the whole setup.
%
%  This works no matter where MATLAB is currently pointed, and no matter
%  whether you grabbed this from GitHub or from the Teams folder (where the
%  folders have friendlier names). It finds itself, sets the paths, and
%  tells you what you can run.

% Find where THIS file lives and make that the working folder. Everything
% else (data, physics functions) is found relative to here, so nobody ever
% has to fight the "Current Folder" dropdown again.
here = fileparts(mfilename('fullpath'));
cd(here);
addpath(genpath(here));   % put every subfolder on the path in one go

fprintf('\n');
fprintf('  CFR27 GEAR RATIO STUDY -- you''re all set. Paths are sorted.\n');
fprintf('  ------------------------------------------------------------\n');
fprintf('  Type one of these and hit enter:\n\n');
fprintf('    gear_ratio_optimization   the main study (efficiency + pack charge, 4 figures)\n');
fprintf('    accel_model               0-75m / 0-100kph accel study\n');
fprintf('    drivetrain_efficiency     overall battery->ground eff + every design lever (halfshaft angle...)\n');
fprintf('    verify_math               regression suite: recomputes everything from params\n');
fprintf('\n');
fprintf('  more:\n');
fprintf('    fatigue_spectrum          endurance driveline torque spectrum\n');
fprintf('    accel_fatigue             accel torque spectrum (the one that fatigues the DT)\n');
fprintf('    brake_analysis            friction / no-regen energy check\n');
fprintf('    efficiency_crosscheck     physics model vs MEASURED efficiency\n');
fprintf('    open_system(''accel_sim'') the accel model in Simulink\n');
fprintf('\n');
fprintf('  Python tools (run from a terminal at the repo root, not MATLAB):\n');
fprintf('    python tools/emeter_unpack.py       unpack the competition e-meter zip + list channels\n');
fprintf('    python tools/emeter_benchmark.py    rank the field on endurance energy economy\n');
fprintf('    python tools/lap_feasibility.py     best lap -> 22-lap feasibility (energy/thermal/driver)\n');
fprintf('\n');
fprintf('  Figures and CSVs land in the output folder.\n');
fprintf('  Every constant lives in params_cfr26.m with a note saying where it came from.\n');
fprintf('\n');

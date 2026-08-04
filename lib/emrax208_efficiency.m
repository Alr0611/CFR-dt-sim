function eff = emrax208_efficiency(rpm, torque_abs, p)
%EMRAX208_EFFICIENCY  REAL-WORLD drivetrain-electrical efficiency (battery->shaft).
%   Starts from physics (copper + iron loss, every term from the EMRAX 208 HV
%   datasheet -- reproduces the ~96% peak the spec quotes), then applies the
%   real-world haircut p.eta_inverter for the inverter + switching/windage/heat
%   losses the datasheet leaves out. So this returns what ACTUALLY reaches the
%   shaft in the car (~90% typical), not the bench-best motor-only number.
%   Calibrated against measured telemetry -- see p.eta_inverter in params. Note
%   that is a CALIBRATION, not an independent validation: eta_inverter was picked
%   to make this model agree with the measured pack-to-shaft number.
%
%   KNOWN BIAS -- CONSTANT Kt (torque per amp).
%   This uses a fixed p.Nm_per_Arms, so current is assumed to scale linearly with
%   torque forever. A real EMRAX does not behave that way: Kt DROOPS at high torque
%   as the iron saturates, so making a given torque takes MORE current than the
%   linear number says. Copper loss goes as I^2, so underestimating current
%   underestimates copper loss, and it does so WORST at high torque.
%   Why that matters here: a LOWER gear ratio demands MORE motor torque for the same
%   wheel force. So this model is most optimistic exactly where the gear study's
%   lower-ratio recommendation lives -- the bias points in the direction of the
%   recommendation. Treat low-ratio efficiency gains as an upper bound.
%   Fixing it properly needs a measured Kt-vs-current curve for the 208; we do not
%   have one, so no saturation model is built here.
    % Current the torque demands, then the I^2*R heat in the windings.
    Irms     = torque_abs / p.Nm_per_Arms;   % constant-Kt -- see bias note above
    P_copper = 3 * Irms.^2 * p.R_phase;
    % Iron/core loss: grows with rpm (hysteresis ~rpm, eddy ~rpm^2), load-independent.
    P_core   = p.core_loss_a*rpm + p.core_loss_b*rpm.^2;
    % Useful shaft power, then efficiency = useful / (useful + the two losses).
    P_mech   = torque_abs .* rpm * (2*pi/60);
    eff = P_mech ./ max(P_mech + P_copper + P_core, 1e-6);   % motor alone (physics)
    % Real-world haircut: the datasheet number is motor-only on a bench; the car also
    % runs everything through the inverter.
    if isfield(p, 'eta_inverter')
        eff = eff * p.eta_inverter;     % + inverter & real-world losses -> real
    end
    eff = min(max(eff, 0.05), 0.97);    % clip degenerate near-zero-load points
end

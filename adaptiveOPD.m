function [T_req, dbg] = adaptiveOPD(apps, v, P, state)
% adaptive OPD: target decel + uFz + paddle shaping
% apps= accelerator pedal position (0–1)
% v= vehicle speed [m/s]
% P= parameter struct (vehicle, maps, OPD settings)
% state= memory flags (ex regen hysteresis)
%
% output:
%   T_req= requested torque (positive = drive, negative = regen)
%   dbg= debug values for plotting / logging

%% VEHICLE CONSTANTS
m  = P.veh.m; % vehicle mass [kg]
g  = P.veh.g; % gravity
Rw = P.veh.Rw; % effective wheel radius [m]

mu = P.veh.mu; % tire-road friction coefficient (for μFz limit)

%% PEDAL NORMALIZATION
dead = P.maps.deadband;
% x = normalized positive-drive pedal
%clamps apps below deadband to 0
%scales 0–1 after removing deadband
x = max(0, min(1, (apps - dead)/(1 - dead)));

% u= normalized lift-off (regen) pedal
%amount of "lift" after deadband
u = max(0, min(1, (dead - apps)/max(dead,1e-6)));

%% REGEN GATE (HYSTERESIS)
% Prevents rapid regen on/off flickering when hovering around 0 pedal.
if ~state.regen_en
    %currently *not* in regen
    % Enable regen only when pedal < enable threshold
    if apps < P.opd.apps_enable
        regen_ok = true;
    else
        regen_ok = false;
    end
else
    % now *are* in regen.
    % disable regen only when pedal rises above disable threshold
    if apps > P.opd.apps_disable
        regen_ok = false;
    else
        regen_ok = true;
    end
end

%% DRIVING (POSITIVE TORQUE)
if x > 0
    % Standard drive map:
    %T= Tmax_const * drive_map(x)
    T_req = P.mot.Tmax_const * P.maps.f_drive(x);

%% REGEN
elseif regen_ok

    % DECEL TARGET:
    % j_target = desired deceleration [m/s²]
    % formula makes regen stronger at high speed
    % and fade to near-zero as speed approaches 0.
    %
    % -(0.50*g)= ~0.5g braking at speed
    % v/(v+6)= speed shaping (tune: 6 ~ fade speed)
    j_target = -(0.50 * 9.81) * (v / (v + 6));

    % convert decel to required force
    F_need = m * j_target; % negative (we want braking)

    % uFz limit on rear axle:
    % only rear wheels regen → rear normal force = 0.5*m*g
    Fz_rear = 0.5 * m * g;
    F_mu    = mu * Fz_rear; %maximum usable braking force before tire slip

    %clamp decel force by uFz envelope
    F_req = max(-F_mu, min(0, F_need));

    % convert force to torque
    %(negative torque = regen)
    T_req = max(-P.mot.Tmax_const, F_req * Rw);

    % PADDLE / LIFT SHAPING
    % Scale regen based on how much the driver lifted off:
    % u in [0..1]= normalized lift
    % k_regen= overall regen level scaler
    % gammaR= shaping exponent (early or late pickup)
    T_req = T_req * P.opd.k_regen * (u^P.opd.gammaR_user);

%% NO DRIVE, NO REGEN
else
    T_req = 0;
end

%% DEBUG STRUCT
dbg = struct('regen_ok', regen_ok, ...
             'omega',   v/max(Rw,1e-6)); % wheel speed

end

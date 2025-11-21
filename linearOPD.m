function [T_req, dbg] = linearOPD(apps, v, P, state)
%% LINEAROPD
% "Simple" one-pedal strategy:
%  -Drive torque is linear: pedal is +T
%  -Regen torque is linear: lift  is -T
%  -No speed-adaptive fade (unlike adaptiveOPD)
%  -Has regen hysteresis (apps_enable / apps_disable)
%% Inputs:
%   apps = accelerator pedal position (0–1)
%   v = vehicle speed [m/s]
%   P = parameter struct (vehicle + OPD settings)
%   state = struct with regen hysteresis flag: state.regen_en
%% Outputs:
%   T_req = requested torque (Nm)
%   dbg = debug info (regen_ok flag, wheel omega)

%% NORMALIZE PEDAL & LIFT
Rw   = P.veh.Rw;
dead = P.maps.deadband;

% x = drive request (remove deadband, clamp 0..1)
x = max(0, min(1, (apps - dead) / (1 - dead)));

% u = lift request (how much the pedal is below deadband)
u = max(0, min(1, (dead - apps) / max(dead,1e-6)));

%% REGEN HYSTERESIS LOGIC
% Hysteresis prevents regen flickering around small pedal values.
% ENTER regen when apps < apps_enable
% EXIT  regen when apps > apps_disable

if ~state.regen_en
    % currently NOT in regen therefore must go below apps_enable to enter
    regen_ok = (apps < P.opd.apps_enable);
else
    % currently IN regen therefore must rise above apps_disable to exit
    regen_ok = ~(apps > P.opd.apps_disable);
end

%% TORQUE REQUEST LOGIC

if x > 0
    %% DRIVE TORQUE
    % Simple: torque = Tmax_const * drive_map(x)
    T_req = P.mot.Tmax_const * P.maps.f_drive(x);

elseif regen_ok
    %% REGEN TORQUE
    % Linear regen:
    % -Scale by 0.65 of max torque (tunable baseline)
    % -Raise lift request to gammaR_user (soft/firm initial feel)
    %  -Multiply by k_regen paddle scale
    T_req = -0.65 * P.mot.Tmax_const * (P.maps.f_regen(u)^P.opd.gammaR_user) * P.opd.k_regen;
else
    %% NEUTRAL ZONE (no torque)
    T_req = 0;
end

%% DEBUG STRUCT
dbg = struct('regen_ok', regen_ok,'omega',    v / max(Rw, 1e-6));

end

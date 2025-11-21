function [T_cmd, caps, slip, Pregen] = applyLimitStack(T_req, v, P, ~)
%% APPLYLIMITSTACK
% This is a compact "stack" of torque limits:
%   1) Torque–speed limit (Pmax / w) and hardware torque cap
%   2) DC-side electrical limits (pack voltage & max DC current)
%   3) uFz traction envelope for rear axle
%   4) Soft derates (temp, SOC, track limits) applied multiplicatively
%   5) Slip gating (placeholder toy model)
%
%% Returns:
%   T_cmd= torque after all limits
%   caps= struct of individual caps for logging
%   slip= slip estimate (placeholder)
%   Pregen= regen electrical power into pack
% --------------------------------------------------------------------

%% BASIC VEHICLE PARAMETERS
Rw = P.veh.Rw;                  % wheel radius
w  = v / max(Rw, 1e-6);         % wheel angular speed [rad/s]

m  = P.veh.m;
g  = P.veh.g;

%% 1) TORQUE–SPEED / HW TORQUE CAP
% Two things limit torque at high speed:
%   (a) a constant mechanical Tmax
%   (b) power-limited torque: Pmax_mech / w

% Always choose the smaller of the two.
T_cap_speed = sign(T_req) * min(abs(P.mot.Tmax_const), P.mot.Pmax_mech / max(w, P.mot.w_floor));

%% 2) DC-SIDE ELECTRICAL CAPS
% Discharge = drive torque (T_req >= 0)
% Charge = regen torque (T_req < 0)
% Uses simple battery model: V = Voc ± I * R

if T_req >= 0
    %DRIVE / DISCHARGE LIMITS
    % Lowest allowable voltage when discharging
    V = max(P.pack.Vmin, P.pack.Voc - P.pack.Idc_dis_max * P.pack.R);

    % Electrical power available = eff * V * I
    Pmax_elec = P.mot.eta_mot * V * P.pack.Idc_dis_max;

    % Convert electrical → torque
    T_cap_dc =  Pmax_elec / max(w, P.mot.w_floor);

else
    %REGEN / CHARGE LIMITS
    % Upper voltage limit when charging
    V = min(P.pack.Vmax, P.pack.Voc + P.pack.Idc_chg_max * P.pack.R);

    Pmax_elec = P.mot.eta_gen * V * P.pack.Idc_chg_max;

    % Negative torque for regen
    T_cap_dc = -Pmax_elec / max(w, P.mot.w_floor);
end

%% 3) uFz TRACTION ENVELOPE (REAR AXLE)
% Tire force cap at the rear axle:
%   F = u * (0.5 * m * g)
% Convert force to torque using wheel radius.
T_cap_mu = P.veh.mu * (0.5 * m * g) * Rw;
if T_req < 0
    T_cap_mu = -T_cap_mu; % regen torque is negative
end

%% COMBINE HARD PHYSICAL LIMITS
% DRIVE torque: take the minimum (most restrictive positive)
% REGEN torque: take the maximum (most restrictive negative)
if T_req >= 0
    T_cap_phys = min([T_cap_speed, T_cap_dc, T_cap_mu]);
else
    T_cap_phys = max([T_cap_speed, T_cap_dc, T_cap_mu]);
end

%% 4) SOFT DERATES (SCALING FACTORS) 
% Combine temp, SOC, and track derates multiplicatively.
% This shrinks the physical cap smoothly.

% multiply temp, soc, and track mode derate
soft = P.derate.f_temp * P.derate.f_soc  * P.derate.f_track;

T_cap = soft * T_cap_phys; %capped torque value

%% 5) SLIP GATE (PLACEHOLDER)
% slip logic is not implemented – slip = 0 always
slip = 0;

% If regen commanded AND slip is excessive, kill regen
% (placeholder logic; slip never < -0.2 so this never triggers)
if (v > 0.5) && (T_req < 0) && (slip < -0.20)
    T_cap = 0;
end

%% FINAL TORQUE SELECTION
% Limit requested torque by the allowed cap
T_cmd = sign(T_req) * min(abs(T_req), abs(T_cap));

%% REGEN POWER CALCULATION
% Mechanical power
Pmech = w * T_cmd;

% Regen power (positive means into the pack)
% only valid when torque is negative
Pregen = (Pmech < 0) * (-P.mot.eta_gen * Pmech);

%% CAPS OUTPUT STRUCT
caps = struct( 'speed', T_cap_speed,'dc',T_cap_dc,'mu', T_cap_mu,'soft',  soft,'phys',  T_cap_phys,'final', T_cap );
end

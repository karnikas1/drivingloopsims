function [T_cmd, caps] = apply_caps(T_req, u, P)
% APPLY_CAPS  Apply all torque caps + derates to requested torque.
% Inputs:
%   T_req= requested torque from pedal map / OPD [Nm]
%   u= struct of live signals:
%   u.omega= wheel speed [rad/s]
%   u.soc= pack state of charge [0..1]
%   u.temps= [T_motor, T_inverter, T_pack] [°C]
%   P= parameter struct (battery, power, torque, temp limits)
%
% outputs:
%   T_cmd= final torque command after caps + derates (pre-rate-filter)
%   caps= struct with all intermediate caps/derates for logging / plots
% -----------------------------------------------------------------

%% SAFE DEFAULTS FOR LOGGING
% If any branches are skipped, these ensure caps.* fields still exist.

T_power_regen = 0;% Regen power cap torque [Nm] (negative)
T_chg_current = 0;% Charge-current cap torque [Nm] (negative)
T_regen_mech  = 0;% Mechanical regen cap [Nm] (negative)
C_regen       = 0;% Regen envelope torque cap [Nm] (negative)

V_at_Idis     = 0;% Pack voltage at discharge current cap [V]
V_at_Ichg     = 0;% Pack voltage at charge current cap [V]
Idis_cap      = 0;% Discharge current cap [A]
Ichg_cap      = 0;% Charge current cap [A]

%% INPUTS
w   = max(u.omega, 1); % wheel speed [rad/s], avoid divide-by-zero
SOC = u.soc;% state of charge [0..1]
eta = P.Drivetrain.eta_m; % overall mechanical to electrical efficiency

% Battery model (simple R-internal model)
Voc = P.Batt.V_oc; % open-circuit voltage [V]
Rin = P.Batt.R_int; % internal resistance [Ω]

%% SOC-BASED POWER BUDGETS
% As SOC gets low / high, we taper available drive and regen power
% Drive (discharge) SOC taper: goes 1 to 0 as SOC approaches dis_min
Dsoc_dis = clip((SOC - P.SOC.dis_min) / (P.SOC.dis_taper - P.SOC.dis_min), 0, 1);

%regen (charge) SOC taper: goes 1 to 0 as SOC approaches chg_max
Dsoc_chg = clip((P.SOC.chg_max - SOC) / (P.SOC.chg_max - P.SOC.chg_taper), 0, 1);

% Final power limits [W]
Pdis_max = P.Power.max*1000  * Dsoc_dis;% drive power cap
Pchg_max = P.Regen.P_base*1000 * Dsoc_chg;% regen power cap

%% DRIVE ENVELOPE CAPS (T >= 0)
% 1) Power cap: T = P / w 
T_power_drive = Pdis_max / w;

% 2) RPM-based cap: power rating means torque falls with speed
T_rpm = min(P.Torque.max, (P.Power.max*1000)/w);

% 3) Phase current cap: I_phase_max aka torque via k_t
T_phase_drive = P.Motor.k_t * P.Inv.I_max;

% 4) Pack current + voltage sag cap using simple R-internal model
%    Choose discharge current so that I ≤ I_max, and V >= V_min under sag
Idis_cap  = min(P.Pack.I_max, max(0,(Voc - P.Batt.V_min)/Rin));
V_at_Idis = Voc - Rin*Idis_cap;

% Convert (V, I, eff) to torque: P = eff * V * I = T * w
T_pack_drive = (eta * V_at_Idis * Idis_cap) / w;

% Overall drive envelope = most restrictive (smallest) positive torque
C_drive = min([T_power_drive, T_rpm, T_phase_drive, T_pack_drive]);

%% REGEN ENVELOPE CAPS (T <= 0)
% Low-speed comfort cutoff: kill regen below a certain RPM.
if u.omega < P.Regen.cutoff_rpm*2*pi/60
    C_regen = 0;

else
    % 1) Power cap in regen (negative torque)
    T_power_regen = -(Pchg_max / w);

    % 2) Charge current + voltage rise:
    %    Choose I so that: I ≤ Ichg_max, and V <= V_max when charging
    Ichg_cap  = min(P.Regen.i_chg_max, max(0,(P.Batt.V_max - Voc)/Rin));
    V_at_Ichg = Voc + Rin*Ichg_cap;
    T_chg_current = -(eta * V_at_Ichg * Ichg_cap) / w;

    % 3) Mechanical regen cap (driver comfort / rear axle limit)
    T_regen_mech  = -P.Regen.max_nm;

    % Regen envelope = "most negative" torque (largest magnitude)
    % max of negatives gives the one closest to zero (tightest limit).
    C_regen = max([T_power_regen, T_chg_current, T_regen_mech]);
end

%% THERMAL DERATES
% Smooth derates based on temps for motor, inverter, and pack.
% derate() outputs 1 (no derate) & 0 (fully off).

g   = P.Temp.gamma;  % shaping exponent for derate curves
dM  = derate(P.Temp.Motor.warn, P.Temp.Motor.limit, u.temps(1), g); %motor
dI  = derate(P.Temp.Inv.warn,   P.Temp.Inv.limit,   u.temps(2), g); %inverter
dP  = derate(P.Temp.Pack.warn,  P.Temp.Pack.limit,  u.temps(3), g); %pack

% Overall thermal derate: take minimum (most restrictive)
Dtherm = min([dM, dI, dP]);

%% EXTRA LOW-SOC DRIVE TAPER
% Optional additional drive-only taper at very low SOC
if isfield(P.SOC,'drive_scale_low') && ...
   ~isempty(P.SOC.drive_scale_low) && P.SOC.drive_scale_low > 0

    % Blend from 0 to 1 between dis_min and drive_scale_low
    Dsoc_blend = clip( ...
        (SOC - P.SOC.dis_min) / ...
        (P.SOC.drive_scale_low - P.SOC.dis_min), 0, 1);
else
    Dsoc_blend = 1;% no extra taper
end

%% APPLY ENVELOPE + DERATES
% Choose which envelope to use based on sign of requested torque.
if T_req >= 0
    cap_env = C_drive;   % drive caps
else
    cap_env = C_regen;   % regen caps
end

% 1) Clip T_req to the chosen envelope.
% 2) Apply thermal derate and low-SOC drive taper.
T_limited = sign(T_req) * ...
            min(abs(T_req), abs(cap_env)) * ...
            Dtherm * Dsoc_blend;

%% FINAL OUTPUTS
T_cmd = T_limited;

% Pack everything into caps struct for logs / plots.
caps = struct();

% Drive caps
caps.env.T_power_drive = T_power_drive;
caps.env.T_rpm = T_rpm;
caps.env.T_phase_drive = T_phase_drive;
caps.env.T_pack_drive = T_pack_drive;
caps.env.C_drive = C_drive;

% Regen caps
caps.env.T_power_regen = T_power_regen;
caps.env.T_chg_current = T_chg_current;
caps.env.T_regen_mech = T_regen_mech;
caps.env.C_regen = C_regen;

% Derates
caps.derate.Dtherm = Dtherm;
caps.derate.Dsoc_dis = Dsoc_dis;
caps.derate.Dsoc_chg = Dsoc_chg;
caps.derate.Dsoc_blend = Dsoc_blend;

% Battery state at caps
caps.batt.Voc = Voc;
caps.batt.Rint = Rin;
caps.batt.V_at_Idis = V_at_Idis;
caps.batt.V_at_Ichg = V_at_Ichg;
caps.batt.Idis_cap = Idis_cap;
caps.batt.Ichg_cap = Ichg_cap;
end

%% HELPER FUNCTIONS
function y = clip(x,a,b)
% CLIP  Limit x to the closed interval [a, b].
y = min(max(x,a),b);
end

function d = derate(warn, lim, T, gamma)
% DERATE: Smoothly scale from 1 to 0 as T moves from warn to limit.
%   T <= warn: d = 1 (no derate)
%   T >= lim: d = 0 (fully off)
%   in between: power-law curve controlled by gamma.
if T <= warn
    d = 1;
elseif T >= lim
    d = 0;
else
    d = ((lim - T) / (lim - warn))^gamma;
end
end


% TESTING CALLS FOR COMMAND WINDOW 
% clear; close all;
% P = params_default();
%
% %% Trial 1 — linear, gain = 1.0 (baseline)
% P.Mode.accel.type       = 'linear';
% P.Mode.accel.pedal_gain = 1.0;
% pedal_map_sweep(P,'accel',[10 40 80],true);   % with caps
%
% %% Trial 2 — linear, gain = 1.5 (snappier tip-in)
% P.Mode.accel.type       = 'linear';
% P.Mode.accel.pedal_gain = 1.5;
% pedal_map_sweep(P,'accel',[10 40 80],true);
%
% %% Trial 3 — adaptive, gain = 1.0 (soft low-speed)
% P.Mode.accel.type       = 'adaptive';
% P.Mode.accel.pedal_gain = 1.0;
% pedal_map_sweep(P,'accel',[10 40 80],true);
%
% %% Optional Trial 4 — adaptive, gain = 1.5
% P.Mode.accel.type       = 'adaptive';
% P.Mode.accel.pedal_gain = 1.5;
% pedal_map_sweep(P,'accel',[10 40 80],true);


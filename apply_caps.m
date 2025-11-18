function [T_cmd, caps] = apply_caps(T_req, u, P)
% Caps + derates with battery voltage sag. Returns T_cmd (pre-filter) and rich caps info.

%Safe defaults so logging never errors
T_power_regen = 0;    % Nm (negative)
T_chg_current = 0;    % Nm (negative)
T_regen_mech  = 0;    % Nm (negative)
C_regen       = 0;    % Nm (negative cap envelope)

V_at_Idis     = 0;    % V
V_at_Ichg     = 0;    % V
Idis_cap      = 0;    % A
Ichg_cap      = 0;    % A

% Inputs / shorthands
w   = max(u.omega, 1);        % rad/s
SOC = u.soc;                  % 0..1
eta = P.Drivetrain.eta_m;

% Battery open-circuit voltage (could be SOC-dependent; simple for now)
Voc = P.Batt.V_oc;
Rin = P.Batt.R_int;

% SOC-dependent power budgets 
Dsoc_dis = clip( (SOC - P.SOC.dis_min) / (P.SOC.dis_taper - P.SOC.dis_min), 0, 1 );
Dsoc_chg = clip( (P.SOC.chg_max - SOC) / (P.SOC.chg_max - P.SOC.chg_taper), 0, 1 );
Pdis_max = P.Power.max*1000 * Dsoc_dis;     % W  (drive)
Pchg_max = P.Regen.P_base*1000 * Dsoc_chg;  % W  (regen)

% Drive envelope caps 
% Power cap & rpm envelope
T_power_drive = Pdis_max / w;
T_rpm         = min(P.Torque.max, (P.Power.max*1000)/w);
% Phase current -> torque
T_phase_drive = P.Motor.k_t * P.Inv.I_max;

% Pack current + voltage sag: choose I limited by I_max AND V >= Vmin
Idis_cap  = min(P.Pack.I_max, max(0,(Voc - P.Batt.V_min)/Rin));
V_at_Idis = Voc - Rin*Idis_cap;
T_pack_drive = (eta * V_at_Idis * Idis_cap) / w;

C_drive = min([T_power_drive, T_rpm, T_phase_drive, T_pack_drive]);

% Regen envelope caps
% Low-speed comfort cutoff
if u.omega < P.Regen.cutoff_rpm*2*pi/60
    C_regen = 0;
else
    % Power budget
    T_power_regen = -(Pchg_max / w);
    % Charge current + voltage rise: I limited by Ichg_max AND V <= Vmax
    Ichg_cap  = min(P.Regen.i_chg_max, max(0,(P.Batt.V_max - Voc)/Rin));
    V_at_Ichg = Voc + Rin*Ichg_cap;
    T_chg_current = -(eta * V_at_Ichg * Ichg_cap) / w;
    % Mechanical regen cap
    T_regen_mech  = -P.Regen.max_nm;

    C_regen = max([T_power_regen, T_chg_current, T_regen_mech]);  % most negative wins
end

% Thermal derates (smooth)
g = P.Temp.gamma;
dM = derate(P.Temp.Motor.warn, P.Temp.Motor.limit, u.temps(1), g);
dI = derate(P.Temp.Inv.warn,   P.Temp.Inv.limit,   u.temps(2), g);
dP = derate(P.Temp.Pack.warn,  P.Temp.Pack.limit,  u.temps(3), g);
Dtherm = min([dM,dI,dP]);

% Optional: extra drive taper at very low SOC
if isfield(P.SOC,'drive_scale_low') && ~isempty(P.SOC.drive_scale_low) && P.SOC.drive_scale_low>0
    Dsoc_blend = clip((SOC - P.SOC.dis_min)/(P.SOC.drive_scale_low - P.SOC.dis_min),0,1);
else
    Dsoc_blend = 1;
end

%Final limit before rate filter
if T_req >= 0
    cap_env = C_drive;
else
    cap_env = C_regen;
end

T_limited = sign(T_req) * min(abs(T_req), abs(cap_env)) * Dtherm * Dsoc_blend;

% Outputs 
T_cmd = T_limited;

caps = struct();
caps.env.T_power_drive = T_power_drive;
caps.env.T_rpm         = T_rpm;
caps.env.T_phase_drive = T_phase_drive;
caps.env.T_pack_drive  = T_pack_drive;
caps.env.C_drive       = C_drive;

caps.env.T_power_regen = T_power_regen;
caps.env.T_chg_current = T_chg_current;
caps.env.T_regen_mech  = T_regen_mech;
caps.env.C_regen       = C_regen;

caps.derate.Dtherm     = Dtherm;
caps.derate.Dsoc_dis   = Dsoc_dis;
caps.derate.Dsoc_chg   = Dsoc_chg;
caps.derate.Dsoc_blend = Dsoc_blend;

caps.batt.Voc          = Voc;
caps.batt.Rint         = Rin;
caps.batt.V_at_Idis    = V_at_Idis;
caps.batt.V_at_Ichg    = V_at_Ichg;
caps.batt.Idis_cap     = Idis_cap;
caps.batt.Ichg_cap     = Ichg_cap;
end

% helpers
function y = clip(x,a,b), y = min(max(x,a),b); end
function d = derate(warn, lim, T, gamma)
if T <= warn, d = 1;
elseif T >= lim, d = 0;
else, d = ((lim - T) / (lim - warn))^gamma;
end
end


% %% Testing Calls for the Command Window
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
% %% Optional Trial 4 — adaptive, gain = 1.5 (keeps soft low-speed, stronger high-speed)
% P.Mode.accel.type       = 'adaptive';
% P.Mode.accel.pedal_gain = 1.5;
% pedal_map_sweep(P,'accel',[10 40 80],true);

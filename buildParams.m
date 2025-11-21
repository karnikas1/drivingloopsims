function P = buildParams()
%% BUILDPARAMS
% Centralized parameter definition file for the driving-loop simulator.
% All tunable constants live here so the sim is clean, readable, and
% version-controllable.  Nothing hard-coded inside algorithm functions.
%
%% Groups:
%   • Vehicle + road load
%   • Motor + drivetrain
%   • Battery / DC-link limits
%   • Pedal maps
%   • OPD behavior (hysteresis, paddles, coast zone)
%   • Derate factors (temp, SOC, track)
%   • Simulation scenario

%% VEHICLE & ROAD PARAMETERS 
P.veh.m   = 285; % vehicle mass [kg]
P.veh.g   = 9.81; % gravity [m/s^2]
P.veh.Rw  = 0.23; % effective wheel radius [m]

% Rolling resistance coefficient (typically 0.01–0.02 for small car)
P.veh.Crr = 0.015;

% Aero model: drag = 0.5 * rho * CxA * v^2
P.veh.rho  = 1.2;% air density [kg/m³]
P.veh.CxA  = 0.9 * 1.0;% drag area [m²] (Cx * frontal area)

% tire-road friction coefficient (dry asphalt ~1.3–1.7)
P.veh.mu   = 1.6;

%% MOTOR + DRIVETRAIN
% Constant-torque region limit
P.mot.Tmax_const = 230;% Nm at rear axle

% Maximum mechanical power rating
P.mot.Pmax_mech  = 80e3;% W (80 kW)

% Efficiencies
P.mot.eta_mot    = 0.92; % motor efficiency when driving
P.mot.eta_gen    = 0.90; % generator efficiency when regen

% Avoid divide-by-zero at standstill
P.mot.w_floor    = 50; % minimum ω used in torque limit [rad/s]

%% BATTERY / DC BUS PARAMETERS
% Simple pack model: V = Voc ± I*R
P.pack.Voc = 496; % open-circuit voltage [V]
P.pack.R = 0.06; % internal resistance [Ω]

% DC current limits (continuous / design values)
P.pack.Idc_dis_max = 300; % max discharge current [A]
P.pack.Idc_chg_max = 160; % max charge (regen) current [A]

% Voltage guardrails
P.pack.Vmin = 420; % minimum pack voltage under load [V]
P.pack.Vmax = 520; % maximum pack voltage when regen [V]

%% PEDAL MAP SHAPING 
P.maps.deadband = 0.06; % throttle deadband region (0–1)
P.maps.f_drive = @(x) x; % accel shape: linear
P.maps.f_regen = @(u) u; % lift/regen shape: linear

%% OPD (ONE-PEDAL DRIVING) SETTINGS
% throttle hysteresis:
% regen_enable: pedal must fall below this to ENTER regen
% regen_disable: must rise above this to EXIT regen
P.opd.apps_disable = 0.02; % turn regen OFF if throttle > 2%
P.opd.apps_enable = 0.01; % turn regen ON  if throttle < 1%

% lift for regen shaping (paddle magnitude & gamma exponent)
P.opd.k_regen = 1.00; % overall regen scaling (0–1)
P.opd.gammaR_user = 1.15; % >1 = softer regen near small lifts

% soft derates allowing global scaling of torque caps
P.derate.f_temp = 1.00; % placeholder (temp derate injected later)
P.derate.f_soc = 1.00; % SOC derate multiplier
P.derate.f_track = 1.00; % track mode multiplier

%% COAST ZONE (NEUTRAL BAND) 
% Creates a speed-dependent “zero torque” band around pedal = 0.
% Helps match paper’s approach where the neutral zone widens at speed.

P.opd.coast.enable = true; % master ON/OFF switch

% Neutral-zone half-width (pedal) at:
P.opd.coast.pedal_pad0 = 0.05; % low speed (0.05 to +-5% pedal dead zone)
P.opd.coast.pedal_pad1 = 0.08; % high speed (0.08 to +-8% dead zone)

P.opd.coast.v_blend = 8; % speed [m/s] where pad transitions from pad0 to pad1

% Edge softening (0 = hard clip, 1 = fully smoothed transition)
P.opd.coast.edge_soft = 0.5;

%% SIMULATION SETTINGS
P.sim.dt = 0.01; % timestep [s]
P.sim.t_end = 16; % duration of scenario [s]

% scenario generator for pedal input
P.scenario  = @(t) makeScenario(t); % apps(t) defined below

end


%% SCENARIO FUNCTION 
function apps = makeScenario(t)
% MAKE SCENARIO
% Synthetic time trace for accelerator pedal input:
%   0–4 s:   accelerate at 60%
%   4–12 s:  lift to 0 (regen)
%   12–14.5: blip to 40%
%   14.5–end: lift again

apps = zeros(size(t));

apps(t < 4)              = 0.60;
apps(t >= 4   & t < 12)  = 0.00;
apps(t >= 12  & t < 14.5)= 0.40;
apps(t >= 14.5)          = 0.00;

end

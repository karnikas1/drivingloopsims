function P = params_default()
% one stop shop for all the knobs

%% motor / inverter hard caps
P.Torque.max     = 230;   % Nm peak
P.Torque.cont    = 130;   % Nm continuous
P.Power.max      = 70;    % kW (FSAE rule)
P.RPM.max        = 6000;  % rpm limit
P.Pack.I_max     = 150;   % A DC fuse
P.Inv.I_max      = 180;   % A rms (phase current)

%% regen rules (don’t annoy the BMS or the driver)
P.Regen.cutoff_rpm  = 300;  % below this rpm, regen feels jerky -> off
P.Regen.soc_disable = 0.90; % disable regen when SOC is high
P.Regen.i_chg_max   = 100;  % A charge-current gate from BMS
P.Regen.max_nm      = 200;  % Nm cap so rear doesn’t get sketchy

%% temperatures: soft ramp before the “nope” line
P.Temp.Motor.warn = 70;  
P.Temp.Motor.limit = 90;
P.Temp.Inv.warn   = 85;   
P.Temp.Inv.limit   = 100;
P.Temp.Pack.warn  = 55;   
P.Temp.Pack.limit  = 65;

%% traction control (simple slip controller)
P.TC.enable      = true;
P.TC.slip_target = 0.10;  % ~10% slip ~= happy tire on dry
P.TC.gain        = 0.50;  % how hard we yank torque back
P.TC.min_speed   = 5;     % km/h; below this we just chill

% per-mode driver feel (map + small personality tweaks)
% Options for .type: 'linear' | 'adaptive' | 'power' | 'smoothstep' | 'bezier'
% ACCEL: snappy but controllable with a smooth finish
P.Mode.accel.type        = 'power';
P.Mode.accel.pedal_gain  = 1;     % overall sensitivity
P.Mode.accel.curve_s     = 0.7;    % <1 = snappier
P.Mode.accel.deadband    = 0.03;    % small deadband if you like

% AUTOCROSS: smooth modulation around mid-pedal
P.Mode.autoX.type        = 'linear';
P.Mode.autoX.pedal_gain  = 1.0;
P.Mode.autoX.deadband    = 0.02;

% ENDURO: very progressive start using a Bezier curve
P.Mode.enduro.type       = 'adaptive';
P.Mode.enduro.pedal_gain = 1;
P.Mode.enduro.bezier     = [0.25 0.05;   % c1  (low y keeps tip-in soft)
                            0.75 0.90];  % c2  (higher y accelerates toward end)
P.Mode.enduro.deadband   = 0.02;


%% simple filters so torque doesn’t step like a square wave
P.Filter.tipin  = 0.20;   % how fast we allow torque to rise
P.Filter.tipout = 0.30;   % how fast we let it fall

%% vehicle bits (good enough for a 1D plant)
P.Vehicle.mass    = 300;  % kg
P.Vehicle.r_wheel = 0.23; % m
P.Vehicle.gear    = 3.5;  % total reduction

%P.Drivetrain.eta_m = 0.90;   % motor+gear efficiency (regen use this too)
% drivetrain / motor constants
P.Drivetrain.eta_m = 0.92;   % mech↔elec efficiency
P.Motor.k_t        = 0.85;   % Nm/A (effective); tune from dyno or datasheet

% SOC tapers (0..1)
P.SOC.dis_min    = 0.10;     % below this, power tapers to 0
P.SOC.dis_taper  = 0.20;     % start tapering below this
P.SOC.chg_taper  = 0.80;     % start tapering regen above this
P.SOC.chg_max    = 0.95;     % no regen above this SOC

% optional: also scale DRIVE torque at very low SOC (set same as dis_taper, or 0 to disable)
P.SOC.drive_scale_low = 0.15;  % set [] or 0 to disable extra scaling

% regen power base (kW) – separate from drive power if you want
P.Regen.P_base = 50;        % kW regen budget (tune from BMS/inverter limits)

% thermal shape
P.Temp.gamma = 1.5;         % derate curve smoothness

% Battery model (voltage sag) 
P.Batt.V_oc      = 400;   % V open-circuit (can be SOC-dependent later)
P.Batt.R_int     = 0.06;  % ohm pack internal resistance (tune!)
P.Batt.V_min     = 300;   % min under load
P.Batt.V_max     = 420;   % max during regen

% Drivetrain / motor constants
P.Drivetrain.eta_m = 0.92;  % mech<->elec efficiency (both ways, for now)
P.Motor.k_t        = 0.85;  % Nm/A effective torque constant

% SOC tapers
P.SOC.dis_min    = 0.10;
P.SOC.dis_taper  = 0.20;
P.SOC.chg_taper  = 0.80;
P.SOC.chg_max    = 0.95;
P.SOC.drive_scale_low = 0.15;  % extra drive taper at very low SOC; [] or 0 to disable

% Regen base power budget (separate from drive if desired) 
P.Regen.P_base = 50;  % kW

% Thermal derate shape 
P.Temp.gamma = 1.5;

% 1D plant: aero + rolling
P.Plant.rho  = 1.225;      % air density kg/m^3
P.Plant.CdA  = 0.80;       % m^2 (tune!)
P.Plant.Crr  = 0.012;      % rolling resistance coeff

%Coast zone parameters
P.opd.coast.enable     = true;
P.opd.coast.pedal_pad0 = 0.05;
P.opd.coast.pedal_pad1 = 0.08;
P.opd.coast.v_blend    = 8;     % <-- underscore, not vblend
P.opd.coast.edge_soft  = 0.5;
%P.opd.v_fade = 6;   % m/s (~22 km/h). Increase = softer at low speed





end


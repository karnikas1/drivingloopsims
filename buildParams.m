function P = buildParams()
% All tunables live here (clean, versionable)

% Vehicle/road
P.veh.m   = 285;            % kg
P.veh.g   = 9.81;           % m/s^2
P.veh.Rw  = 0.23;           % m (effective)
P.veh.Crr = 0.015;          % rolling resistance
P.veh.rho = 1.2;  P.veh.CxA = 0.9*1.0;   % aero: rho, Cx*A
P.veh.mu  = 1.6;            % tire μ (dry)

% Motor/gear
P.mot.Tmax_const = 230;     % Nm (rear axle)
P.mot.Pmax_mech  = 80e3;    % W
P.mot.eta_mot    = 0.92;    % motoring eff
P.mot.eta_gen    = 0.90;    % regen eff
P.mot.w_floor    = 50;      % rad/s guard

% Pack / DC bus
P.pack.Voc = 496;           % V (example; put your real pack)
P.pack.R   = 0.06;          % ohm
P.pack.Idc_dis_max = 300;   % A
P.pack.Idc_chg_max = 160;   % A
P.pack.Vmin = 420;          % V
P.pack.Vmax = 520;          % V

% Pedal maps
P.maps.deadband  = 0.06;
P.maps.f_drive   = @(x) x;  % tip-in shape
P.maps.f_regen   = @(u) u;  % lift shape

% OPD gates / paddles / soft derates
P.opd.apps_disable = 0.02;  % no regen if throttle > 2%
P.opd.apps_enable  = 0.01;  % enable when throttle < 1% (hysteresis)
P.opd.k_regen      = 1.00;  % paddle magnitude (0..1)
P.opd.gammaR_user  = 1.15;  % paddle feel (>1 softer initial lift)
P.derate.f_temp    = 1.00;  % wire in tempDerateStep later
P.derate.f_soc     = 1.00;
P.derate.f_track   = 1.00;

% --- Coast zone (speed-aware neutral band) ---
P.opd.coast.enable      = true;    % toggle
P.opd.coast.pedal_pad0  = 0.05;    % neutral band half-width (±pad) at low speed [pedal units 0..1]
P.opd.coast.pedal_pad1  = 0.08;    % neutral band half-width at high speed
P.opd.coast.v_blend     = 8;       % m/s where pad transitions from pad0→pad1
P.opd.coast.edge_soft   = 0.5;     % 0..1: amount of soft edge (blend thickness as fraction of pad)


% Simulation & scenario
P.sim.dt    = 0.01;  P.sim.t_end = 16;
P.scenario  = @(t) makeScenario(t);   % apps profile

end

function apps = makeScenario(t)
% 0–4 s accel, 4–12 s lift (OPD), blip, then lift again
apps = zeros(size(t));
apps(t<4)             = 0.60;
apps(t>=4   & t<12)   = 0.00;
apps(t>=12  & t<14.5) = 0.40;
apps(t>=14.5)         = 0.00;
end
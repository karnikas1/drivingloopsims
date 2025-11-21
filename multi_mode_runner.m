function multi_mode_runner(Pin)
%% MULTI_MODE_RUNNER
% Compact runner to compare multiple driving "modes" (accel / autoX / enduro)
% using a shared driver trace and a simple thermal model.
%% Outputs (need to be enabled via knobs):
%  -One summary figure with 3 subplots:
%       1) Torque command vs time (per mode)
%       2) Vehicle speed vs time (per mode)
%       3) T_req vs T_cmd (regen vs commanded torque)
%  -Optional combined drive-cap and regen-cap stack plots (all modes)
%  -Optional per-mode detailed debug plots (limiter histograms, etc.)
%
%% to run:
%   >> multi_mode_runner

%% KNOBS & SETUP
% Parameters input / default
if nargin == 0
    P = params_default();
else
    P = Pin;
end

% Driving modes to simulate
modes  = {'accel','autoX','enduro'};

% Simulation horizon
nSteps = 600;
dt     = 0.01;    % [s]

% Simple 3-node thermal model (motor / inverter / pack)
Tamb = 25; % ambient temperature [°C]

% Heating gains: [K per kW·s] for motor / inverter / pack
% larger value = heats up faster per unit of power
kJ_per_W = [0.03 0.04 0.06];

% Cooling time constants [s] for each node
tau_s = [60  90  120];

% Initial node temperatures [°C] → [motor, inverter, pack]
T = [30 30 30];

% Initial state (shared across modes) 
v0 = 20; % initial speed [m/s] (~72 km/h)
omega0 = v0 / P.Vehicle.r_wheel; % initial wheel speed [rad/s]

% Driver traces (shared across modes)
% Pedal: ramp from 0 to 1 over first 100 steps, then hold WOT
pedal_trace = [linspace(0,1,100), ones(1,nSteps-100)];

% Brake: zero initially, then "stomp" at brake_idx
brake_trace = zeros(1,nSteps);
brake_idx = 300;
brake_trace(brake_idx:end) = 0.7; % brake pedal ~70%

% Option: drop pedal when brake is pressed (no left-foot overlap)
DROP_PEDAL_ON_BRAKE = true;
if DROP_PEDAL_ON_BRAKE
    pedal_trace(brake_idx:end) = 0.0;
end

% Plots to show
SHOW_SUMMARY_FIG = true; % main 3-panel figure
SHOW_COMBINED_CAPS = true; % aggregated drive/regen cap stacks
SHOW_PERMODE_DEBUG = false; % detailed per-mode debug (noisy)

%% PREALLOCATE STORAGE
tmpl = struct('T_req',0,'T_cmd',0,'F_front',0,'F_rear',0, 'omega',0,'v',0,'caps',struct());
results = struct();

%% MAIN MODE LOOP
for m = 1:numel(modes)
    mode  = modes{m};
    state = struct('last_T',0,'omega',omega0);
    out(1:nSteps) = tmpl;

    % Reset thermal state for each mode run (so each mode starts equally)
    T_mode = T;

    for k = 1:nSteps
        v_k = state.omega * P.Vehicle.r_wheel;

        % Build input struct for driving loop
        u = struct('pedal',pedal_trace(k), ...
                   'brake',brake_trace(k), ...
                   'wheelSpeeds',state.omega, ...
                   'soc',0.6,'vpack',400, ...
                   'temps',T_mode, ...    % use modeled temps
                   'dt',dt,'v',v_k,'omega',state.omega);

        % Run one step of driving loop for this mode
        [out(k), state] = run_driving_loop(u, P, state, mode);

        % SIMPLE THERMAL UPDATE (ALL POWER is HEAT)
        % electrical power estimate:
        % P_elec ≈ |T_cmd * w| / eff   (only positive drive power)
        Pelec = max(0, abs(out(k).T_cmd * state.omega) / max(P.Drivetrain.eta_m,1e-3)); % [W]

        % heat gain: (P [kW] * kJ_per_W) over dt [s] to [K]
        % Vector operations, one per node (motor/inv/pack)
        dT_heat = (Pelec/1000) .* kJ_per_W * u.dt; % [K]

        % cooling: exponential decay toward ambient
        dT_cool = -(T_mode - Tamb) .* (u.dt ./ tau_s);

        % update thermal nodes
        T_mode = T_mode + dT_heat + dT_cool;

        % next iteration will use updated temps via u.temps
    end

    % store mode output
    results.(mode) = out;
end

%% SUMMARY PLOT (TORQUE/SPEED/REGEN)
flds = fieldnames(results);

if SHOW_SUMMARY_FIG
    figure('Name','Summary — torque / speed / regen','Color','w'); 
    tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

    %% (1) Torque command per mode 
    nexttile; hold on; grid on
    for m = 1:numel(flds)
        plot([results.(flds{m}).T_cmd], ...
            'LineWidth',1.6,'DisplayName',flds{m});
    end
    xline(brake_idx,'r--','Brake stomp','LabelVerticalAlignment','bottom');
    ylabel('T_{cmd} [Nm]');
    title('Torque command per mode');
    legend('Location','best');

    %% (2) Vehicle speed per mode
    nexttile; hold on; grid on
    for m = 1:numel(flds)
        plot([results.(flds{m}).v]*3.6, ...
            'LineWidth',1.6,'DisplayName',flds{m});
    end
    xline(brake_idx,'r--','Brake stomp','LabelVerticalAlignment','bottom');
    ylabel('Speed [km/h]');
    title('Vehicle speed per mode');

    %% (3) T_req vs T_cmd (regen vs actual)
    nexttile; hold on; grid on
    for m = 1:numel(flds)
        plot([results.(flds{m}).T_req],':', ...
            'LineWidth',1.25,'DisplayName',[flds{m} ' T_{req}']);
        plot([results.(flds{m}).T_cmd],'-', ...
            'LineWidth',1.6, 'DisplayName',[flds{m} ' T_{cmd}']);
    end
    xline(brake_idx,'r--','Brake stomp','LabelVerticalAlignment','bottom');
    xlabel('time step');
    ylabel('Nm');
    title('Regen vs commanded torque');
    legend('Location','eastoutside');
end

%% OPTIONAL: PER-MODE DEBUG PLOTS
if SHOW_PERMODE_DEBUG
    for i = 1:numel(flds)
        m = flds{i};
        data = results.(m);
        try
            plot_caps_debug(data, m);
        catch ME
            warning("plot_caps_debug failed for %s: %s", m, ME.message);
        end
    end
end

%% OPTIONAL: COMBINED CAP STACK PLOTS 
% Two aggregated figures: one for drive caps, one for regen caps
if SHOW_COMBINED_CAPS
    try
        plot_caps_stack(results);          % drive caps (all modes)
    catch ME
        warning("plot_caps_stack failed: %s", ME.message);
    end
    try
        plot_caps_stack_regen(results);    % regen caps (all modes)
    catch ME
        warning("plot_caps_stack_regen failed: %s", ME.message);
    end
end
end

% To run:
%   multi_mode_runner




function multi_mode_runner(Pin)
% MULTI_MODE_RUNNER
% Compact runner for accel / autoX / enduro with minimal figures.
%
% Outputs:
%   - 1 summary figure (3 subplots: T_cmd, vehicle speed, regen vs command)
%   - Optional: 1 drive-cap stack (all modes) + 1 regen-cap stack (all modes)
%   - Optional: per-mode detailed debug (OFF by default)

%% knobs
% Inputs / timing
if nargin==0, P = params_default(); else, P = Pin; end
modes  = {'accel','autoX','enduro'};
nSteps = 600;
dt     = 0.01;

% super simple thermal model
Tamb = 25;
kJ_per_W  = [0.03 0.04 0.06];   % heating gains motor/inv/pack (K per kW·s)
tau_s     = [60  90  120];      % cooling time constants [s]
T = [30 30 30];                 % initial temps [°C] -> motor, inverter, pack


% Start state
v0      = 20;                         % m/s  (~72 km/h)
omega0  = v0 / P.Vehicle.r_wheel;

% Driver traces
pedal_trace = [linspace(0,1,100), ones(1,nSteps-100)];  % ramp to WOT
brake_trace = zeros(1,nSteps);
brake_idx   = 300;
brake_trace(brake_idx:end) = 0.7;                       % stomp brake

DROP_PEDAL_ON_BRAKE = true;
if DROP_PEDAL_ON_BRAKE
    pedal_trace(brake_idx:end) = 0.0;
end

% What to show
SHOW_SUMMARY_FIG     = true;   % single, tidy 3-panel summary
SHOW_COMBINED_CAPS   = true;   % drive caps + regen caps (two figures)
SHOW_PERMODE_DEBUG   = false;  % many windows – keep OFF unless you need them
%% -----------------------------------------------------------------------

% Preallocate result slots
tmpl = struct('T_req',0,'T_cmd',0,'F_front',0,'F_rear',0, ...
              'omega',0,'v',0,'caps',struct());
results = struct();

% ------------------------------ simulate --------------------------------
for m = 1:numel(modes)
    mode  = modes{m};
    state = struct('last_T',0,'omega',omega0);
    out(1:nSteps) = tmpl;

    for k = 1:nSteps
        v_k = state.omega * P.Vehicle.r_wheel;

u = struct('pedal',pedal_trace(k), ...
           'brake',brake_trace(k), ...
           'wheelSpeeds',state.omega, ...
           'soc',0.6,'vpack',400, ...
           'temps',T, ...               % << use modeled temps
           'dt',dt,'v',v_k,'omega',state.omega);


        [out(k), state] = run_driving_loop(u,P,state,mode);

        % electrical power estimate (use positive drive power only)
Pelec = max(0, abs(out(k).T_cmd * state.omega) / max(P.Drivetrain.eta_m,1e-3));  % W

% heat in K this step from power (very crude)
dT_heat = (Pelec/1000) .* kJ_per_W * u.dt;   % K; scale per node via .* later

% cool toward ambient
dT_cool = - (T - Tamb) .* (u.dt ./ tau_s);

% update node temps
T = T + dT_heat + dT_cool;

% write temps that the next step will see
if k < nSteps
    % nothing—just store into u in the next iteration
end


    end

    results.(mode) = out;
end

% ------------------------------ plotting --------------------------------
flds = fieldnames(results);

if SHOW_SUMMARY_FIG
    figure('Name','Summary — torque / speed / regen','Color','w'); 
    tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

    % 1) Torque command per mode
    nexttile; hold on; grid on
    for m = 1:numel(flds)
        plot([results.(flds{m}).T_cmd],'LineWidth',1.6,'DisplayName',flds{m});
    end
    xline(brake_idx,'r--','Brake stomp','LabelVerticalAlignment','bottom');
    ylabel('T_{cmd} [Nm]'); title('Torque command per mode'); legend('Location','best');

    % 2) Vehicle speed per mode
    nexttile; hold on; grid on
    for m = 1:numel(flds)
        plot([results.(flds{m}).v]*3.6,'LineWidth',1.6,'DisplayName',flds{m});
    end
    xline(brake_idx,'r--','Brake stomp','LabelVerticalAlignment','bottom');
    ylabel('Speed [km/h]'); title('Vehicle speed per mode');

    % 3) Regen vs commanded torque (overlay)
    nexttile; hold on; grid on
    for m = 1:numel(flds)
        plot([results.(flds{m}).T_req],':','LineWidth',1.25,'DisplayName',[flds{m} ' T_{req}']);
        plot([results.(flds{m}).T_cmd],'-','LineWidth',1.6, 'DisplayName',[flds{m} ' T_{cmd}']);
    end
    xline(brake_idx,'r--','Brake stomp','LabelVerticalAlignment','bottom');
    xlabel('time step'); ylabel('Nm'); title('Regen vs commanded torque'); 
    legend('Location','eastoutside'); 
end

% --- Optional: per-mode detailed debug (limiter histograms, etc.)
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

% tidy, combined cap stacks (just 2 figures total)
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

%To run: 
%multi_mode_runner



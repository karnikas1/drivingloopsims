function compare_tc_controllers()
%% COMPARE_TC_CONTROLLERS
% Comparison of two traction-control (TC) strategies:
% -P-controller (classic proportional slip trim)
% -Fuzzy controller (rule-based slip trimming)
%
% A slip "patch" is injected artificially by multiplying wheel speed for
% a short duration. This simulates a traction loss (e.g. wet paint strip).
%
%% Outputs:
% -Torque trims vs time (how aggressively each TC responds)
% -Slip tracking (how well each controller reduces slip)
% -Vehicle speed response under each controller
%
%% Purpose:
%   Evaluate stability, responsiveness, and smoothness of the two TC logics.

pretty();  % formatting helper

%% BASE PARAMETERS
P = params_default();
P.TC.slip_target = 0.08; % target slip ratio (< injected slip)

dt = 0.01;  
T  = 10; % 10-second test
n  = round(T/dt);

mode = 'autoX'; % pedal-map mode used by run_driving_loop()

%% DRIVER INPUTS 
% 0 → 0.6 pedal over first 2 seconds, then hold 0.6 for remainder.
pedal = [linspace(0,0.6,200), 0.6*ones(1,n-200)];
brake = zeros(1,n);

%% DEFINE SLIP PATCH 
% Inject artificial wheel slip by scaling wheel speed.
% Slip window: 3.5s to 5.0s
slip_on   = 3.5;
slip_off  = 5.0;

idx_on  = round(slip_on/dt);
idx_off = round(slip_off/dt);

slip_gain = 1.20; % wheel rotates 20% faster → ~20% slip

%% RUN BOTH CONTROLLERS
outP = run_with_tc(@tc_trim_p);      % proportional TC
outF = run_with_tc(@tc_trim_fuzzy);  % fuzzy-logic TC

%% PLOTTING
t = (0:n-1)*dt;

figure('Name','TC comparison'); 
tiledlayout(3,1,'TileSpacing','compact');

%% (1) TORQUE TRIMS 
nexttile; hold on;
plot(t,[outP.T_cmd],'LineWidth',1.8,'DisplayName','P controller');
plot(t,[outF.T_cmd],'LineWidth',1.8,'DisplayName','Fuzzy controller');
ylabel('T_{cmd} [Nm]');
title('Torque trims under slip');
legend; grid on;
xline([slip_on slip_off],'r--',{'slip on','slip off'});

%% (2) SLIP SERIES 
nexttile; hold on;
plot(t,slip_series(outP,P),'LineWidth',1.6,'DisplayName','P slip');
plot(t,slip_series(outF,P),'LineWidth',1.6,'DisplayName','Fuzzy slip');
ylabel('slip ratio');
title('Slip tracking');
legend; grid on;
xline([slip_on slip_off],'r--');

%% (3) VEHICLE SPEED
nexttile; hold on;
plot(t,[outP.v]*3.6,'LineWidth',1.6,'DisplayName','P speed');
plot(t,[outF.v]*3.6,'LineWidth',1.6,'DisplayName','Fuzzy speed');
ylabel('km/h'); xlabel('time [s]');
grid on; legend;
xline([slip_on slip_off],'r--');

%% INNER HELPER: run_one_TC_controller()
    function out = run_with_tc(tc_fun)
        %% RUN_WITH_TC
        % Executes full longitudinal simulation using a provided TC function:
        % tc_fun(T_req, u, P)
        %% Steps:
        %   1. Compute wheel speed (with injected slip window)
        %   2. Compute raw torque request from pedal map
        %   3. Apply traction-control trim (via tc_fun)
        %   4. Apply torque caps, filters, plant model
        %   5. Log results

        %% INITIALIZE STATE
        state.last_T = 0;
        v0 = 10/3.6; % start at 10 km/h
        state.omega = v0 / P.Vehicle.r_wheel;

        % Logging template
        tmpl = struct('T_req',0,'T_cmd',0,'F_front',0,'F_rear',0, ...
                      'omega',0,'v',0,'caps',struct('rpm',0,'power',0,'derate',1), ...
                      'wheelOmega',0);
        out(1:n) = tmpl;

        %% MAIN LOOP
        for k = 1:n

            v_k = state.omega * P.Vehicle.r_wheel;

            % Base wheel speed (motor to wheels)
            omega_w = state.omega;

            % Inject slip during patch
            if k >= idx_on && k <= idx_off
                omega_w = omega_w * slip_gain;
            end

            % Input struct for driving loop
            u = struct('pedal',pedal(k), 'brake',brake(k), ...
                       'wheelSpeeds',omega_w, ...
                       'soc',0.6,'vpack',400,'temps',[30 30 30], ...
                       'dt',dt,'v',v_k,'omega',state.omega);

            % Base torque request 
            mp = P.Mode.(mode); 
            mp.v = u.v;   % pass speed to pedal map

            % Motor torque speed cap: Pmax / w
            T_drive_cap = min(P.Torque.max, (P.Power.max*1000)/max(u.omega,1));

            % Pedal to torque map
            T_req = pedal_map(u.pedal, mp, T_drive_cap);

            % Safety override (APPS+BPS)
            if (u.brake > 0.05) && (u.pedal > 0.25)
                T_req = 0;
            end

            % Regenerative braking if off-pedal
            if u.pedal < 0.05
                T_req = -min(P.Regen.max_nm, regen_limit(u.omega, u.soc, u.vpack, P));
            end

            %% TRACTION CONTROL 
            % Apply selected TC trimming logic
            T_req = tc_fun(T_req, u, P);

            % Torque caps & dynamics
            [T_cmd, caps] = apply_caps(T_req, u, P); % physical caps
            T_cmd = torque_filter(T_cmd, state.last_T, P.Filter);  % smoothing
            [omega_new, v_new] = longitudinalPlant(T_cmd, state.omega, dt, P);

            % Log output
            o = tmpl;
            o.T_req = T_req;
            o.T_cmd = T_cmd;
            o.caps = caps;
            o.omega = omega_new;
            o.v = v_new;
            o.wheelOmega = omega_w;
            out(k) = o;

            % Update state for next step
            state.last_T = T_cmd;
            state.omega  = omega_new;
        end
    end

%% INNER HELPER: compute slip series from output
    function s = slip_series(out, P)
        %% SLIP_SERIES
        % slip = (w_wheel * R_wheel – vehicle_speed) / vehicle_speed
        v = [out.v];
        omega_w = [out.wheelOmega]; % actual wheel speed incl slip
        s = (omega_w * P.Vehicle.r_wheel - v) ./ max(v, 0.1);
    end

end




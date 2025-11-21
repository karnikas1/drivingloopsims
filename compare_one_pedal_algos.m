function compare_one_pedal_algos()
%% COMPARE_ONE_PEDAL_ALGOS

%% Compare two One-Pedal Driving (OPD) strategies:
% -Algorithm A: linear pedal → torque
% -Algorithm B: adaptive pedal → torque (speed-aware fading, etc.)
%
%% Runs both algorithms over a fixed pedal/brake scenario and
% plots:
%   1) Vehicle speed vs time
%   2) Torque commands vs time
%   3) Cumulative recovered energy (regen)
%
%% Purpose:
%   Visual, high-level comparison of OPD feel, regen strength,
%   and speed control between linear and adaptive logic.

%% SETUP
P = params_default(); % Load baseline parameters
dt = 0.01; T = 16; % Sim length + step size
n = round(T/dt);

%% DEFINE TWO ALGORITHMS
% OPD behavior lives under: P.Mode.autoX.*

A = P;  
A.Mode.autoX.type = 'linear';
A.Mode.autoX.pedal_gain = 1.0;

B = P;
B.Mode.autoX.type        = 'adaptive';
B.Mode.autoX.pedal_gain  = 1.0;

%% SCENARIO DEFINITION
% Time sequence:
%0–4s:ramp pedal to 1.0 (full accel)
%4–8s:pedal at 0 (regen)
%8–12s:pedal 0, brake 0 (true coast region)
%12–14s:step to pedal = 0.5 (mild accel)
%14–16s:pedal = 0.2 (creep that shows adaptive fade behavior)

pedal = zeros(1,n);
brake = zeros(1,n);
t = (0:n-1)*dt;

% Ramp from 0 → 1 over first 4 seconds
pedal(t <= 4) = min(1, t(t<=4)/4);

% Nothing from 4 → 12

% Step input from 12–14
pedal(t > 12 & t <= 14) = 0.5;

% Small creep afterwards
pedal(t > 14) = 0.2;

%% RUN BOTH MODES 
% runMode(P) → runs full vehicle loop for that algorithm
runMode = @(P_) run_trace(P_, pedal, brake, dt, 'autoX');

[OA, EA] = runMode(A);   % outputs + per-step regen energy
[OB, EB] = runMode(B);

%% PLOTTING 
figure('Name','One-Pedal: algorithm comparison');
tiledlayout(3,1,'TileSpacing','compact');

%% 1) Vehicle Speed 
nexttile; hold on;
plot(t, [OA.v], 'LineWidth',1.4, 'DisplayName','Alg A (linear)');
plot(t, [OB.v], 'LineWidth',1.4, 'DisplayName','Alg B (adaptive)');
ylabel('Speed v [m/s]');
legend('Location','best'); grid on;
title('Vehicle speed');

%% 2) Torque Command
nexttile; hold on;
plot(t, [OA.T_cmd], 'LineWidth',1.4, 'DisplayName','Alg A');
plot(t, [OB.T_cmd], 'LineWidth',1.4, 'DisplayName','Alg B');
yline(0,'k-'); 
ylabel('Axle torque T [Nm]');
grid on;
title('Torque commands (+=drive, -=regen)');

%% 3) Recovered Energy 
nexttile; hold on;
plot(t, cumsum(EA)/1000, 'LineWidth',1.6, 'DisplayName','Alg A');
plot(t, cumsum(EB)/1000, 'LineWidth',1.6, 'DisplayName','Alg B');

ka = sum(EA)/1000;     % total recovered energy in kJ
kb = sum(EB)/1000;
txt = sprintf('Total: A = %.1f kJ,  B = %.1f kJ', ka, kb);

ylabel('Recovered energy [kJ]');
xlabel('Time [s]');
grid on;
title(txt);
legend('Location','best');

end

%% HELPER: run the full driving loop for a provided pedal/brake trace
function [out, E_reg] = run_trace(P, pedal, brake, dt, mode)
% RUN_TRACE
% Given:
% -Parameter set P
% -Pedal + brake arrays (size n)
% -dt timestep
% -mode string for selecting OPD algorithm
%
% This function:
% -Runs run_driving_loop() each timestep
% -Logs output into struct array `out`
% -Calculates regen energy per step (E_reg)
%
% Returns:
% out(k).T_cmd to torque command
% out(k).v to vehicle speed
% E_reg(k) to recovered energy at timestep k [J]

%% INITIAL STATE
state.last_T = 0;
v0 = 0; % start at rest
state.omega = v0 / P.Vehicle.r_wheel; % corresponding wheel ω

n = numel(pedal);

% Template for logging
tmpl = struct('T_req',0,'T_cmd',0,'F_front',0,'F_rear',0,...
              'omega',0,'v',0,'caps',struct('rpm',0,'power',0,'derate',1));

out(1:n) = tmpl;
E_reg = zeros(1,n);

%% MAIN TIME LOOP 
for k = 1:n
    % Current vehicle speed from wheel angular speed
    v_k = state.omega * P.Vehicle.r_wheel;
    % Build the input structure for one timestep
    u = struct( 'pedal', pedal(k), ...
        'brake',        brake(k), ...
        'wheelSpeeds',  state.omega, ...
        'soc',          0.6, ...       % fixed SOC for this comparison
        'vpack',        400, ...       % fixed DC voltage
        'temps',        [30 30 30], ... % motor, inverter, pack temps
        'dt',           dt, ...
        'v',            v_k, ...
        'omega',        state.omega);

    % Run the OPD algorithm + torque caps + vehicle update
    [out(k), state] = run_driving_loop(u, P, state, mode);

    % Regen energy calculation 
    % P_mech = ω * T  (but only negative torque counts as regen)
    P_mech = -min(out(k).T_cmd,0) * out(k).omega; % mechanical power [W]

    eta = P.Drivetrain.eta_m; % regen efficiency
    E_reg(k) = P_mech * eta * dt; % Joules added to pack

end

end


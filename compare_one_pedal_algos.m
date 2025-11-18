function compare_onepedal_algos()
% One-Pedal comparison: Linear vs Adaptive
P = params_default();
dt = 0.01; T = 16; n = round(T/dt);

% Define “algorithms” by setting mode behavior
A = P;  A.Mode.autoX.type='linear';   A.Mode.autoX.pedal_gain=1.0;
B = P;  B.Mode.autoX.type='adaptive'; B.Mode.autoX.pedal_gain=1.0;

% Scenario (time in seconds):
% 0–4s: pedal ramp to 1.0, no brake
% 4–8s: pedal 0 (lift), regen only
% 8–12s: pedal 0, brake 0 (coast)
% 12–14s: pedal step to 0.5 (mild accel)
% 14–16s: pedal 0.2 (creep, shows adaptive fade)
pedal = zeros(1,n); brake = zeros(1,n);
t = (0:n-1)*dt;
pedal(t<=4) = min(1, t(t<=4)/4);
% nothing 4–12
pedal(t>12 & t<=14) = 0.5;
pedal(t>14) = 0.2;

% helper to run a full loop
runMode = @(P_) run_trace(P_, pedal, brake, dt, 'autoX');

[OA, EA] = runMode(A);
[OB, EB] = runMode(B);

%Plot
figure('Name','One-Pedal: algorithm comparison'); tiledlayout(3,1,'TileSpacing','compact');

% Speed
nexttile; hold on;
plot(t, [OA.v], 'LineWidth',1.4, 'DisplayName','Alg A (linear)');
plot(t, [OB.v], 'LineWidth',1.4, 'DisplayName','Alg B (adaptive)');
ylabel('Speed v [m/s]'); legend('Location','best'); grid on; title('Vehicle speed');

% Torque
nexttile; hold on;
plot(t, [OA.T_cmd], 'LineWidth',1.4, 'DisplayName','Alg A');
plot(t, [OB.T_cmd], 'LineWidth',1.4, 'DisplayName','Alg B');
yline(0,'k-'); ylabel('Axle torque T [Nm]'); grid on; title('Torque commands (+=drive, -=regen)');

% Recovered energy (kJ)
nexttile; hold on;
plot(t, cumsum(EA)/1000, 'LineWidth',1.6, 'DisplayName','Alg A');
plot(t, cumsum(EB)/1000, 'LineWidth',1.6, 'DisplayName','Alg B');
ka = sum(EA)/1000; kb = sum(EB)/1000;
txt = sprintf('Total: A = %.1f kJ,  B = %.1f kJ', ka, kb);
ylabel('Recovered energy [kJ]'); xlabel('Time [s]'); grid on; title(txt); legend('Location','best');

end

%helper: run the driving loop for a trace, return out[] and regen energy per step
function [out, E_reg] = run_trace(P, pedal, brake, dt, mode)
state.last_T = 0;
v0 = 0;  % start from rest for this comparison
state.omega = v0 / P.Vehicle.r_wheel;

n = numel(pedal);
tmpl = struct('T_req',0,'T_cmd',0,'F_front',0,'F_rear',0,'omega',0,'v',0,'caps',struct('rpm',0,'power',0,'derate',1));
out(1:n) = tmpl;
E_reg = zeros(1,n);

for k = 1:n
    v_k = state.omega * P.Vehicle.r_wheel;
    u = struct('pedal',pedal(k),'brake',brake(k), ...
               'wheelSpeeds',state.omega,'soc',0.6,'vpack',400,'temps',[30 30 30], ...
               'dt',dt,'v',v_k,'omega',state.omega);
    [out(k), state] = run_driving_loop(u,P,state,mode);

    % electrical power recovered (positive when T_cmd is negative)
    P_mech = -min(out(k).T_cmd,0) * out(k).omega;  % [W] (Nm * rad/s)
    eta = P.Drivetrain.eta_m;
    E_reg(k) = P_mech * eta * dt;                  % [J] added back to pack
end
end

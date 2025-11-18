%% run_min.m — minimal runner for Alg A vs Alg B
clear; clc; close all;
addpath(genpath(fileparts(mfilename('fullpath'))));  % ensure subfolders on path

% --- Params & scenario
P  = buildParams();                 % all tunables live here
dt = P.sim.dt; 
t  = 0:dt:P.sim.t_end; 
n  = numel(t);
apps = P.scenario(t);               % accelerator profile [0..1]

% --- Regen hysteresis state (separate for each controller)
stateA.regen_en = true;  
stateB.regen_en = true;

% --- Logs (Alg A and Alg B)
R1 = struct('v',zeros(1,n),'T',zeros(1,n),'Erec',zeros(1,n));
R2 = R1;

% Optional: limiter diagnostics for Alg B
TreqB_log = zeros(1,n);
capsB     = struct('speed',zeros(1,n),'dc',zeros(1,n),'mu',zeros(1,n), ...
                   'phys',zeros(1,n),'final',zeros(1,n));

% --- Main loop
for k = 2:n
    vA = R1.v(k-1);  vB = R2.v(k-1);

    % Controller requests
    [TreqA, dA] = linearOPD(  apps(k), vA, P, stateA);
    [TreqB, dB] = adaptiveOPD(apps(k), vB, P, stateB);

    % Limit stacks
    [TA, ~, ~, PregA] = applyLimitStack(TreqA, vA, P, stateA);
    [TB, cB, ~, PregB] = applyLimitStack(TreqB, vB, P, stateB);

    % Plant integration
    [R1.v(k), ~, ~] = longitudinalPlant(vA, TA, P, dt);
    [R2.v(k), ~, ~] = longitudinalPlant(vB, TB, P, dt);

    % Book-keeping
    R1.T(k)    = TA; 
    R2.T(k)    = TB;
    R1.Erec(k) = R1.Erec(k-1) + max(PregA,0)*dt;
    R2.Erec(k) = R2.Erec(k-1) + max(PregB,0)*dt;

    % Update regen-gate latches
    stateA.regen_en = dA.regen_ok;
    stateB.regen_en = dB.regen_ok;

    % Diagnostics (Alg B)
    TreqB_log(k)   = TreqB;
    capsB.speed(k) = cB.speed;
    capsB.dc(k)    = cB.dc;
    capsB.mu(k)    = cB.mu;
    capsB.phys(k)  = cB.phys;
    capsB.final(k) = cB.final;
end

% --- Comparison plots
plotResults(t, R1, R2, 'Alg A (linear)', 'Alg B (adaptive)');

% --- Optional: limiter envelope plot for Alg B (which cap is binding?)
figure('Color','w','Name','Alg B limiter diagnostics');
plot(t, TreqB_log,'k--','LineWidth',1.0); hold on;
plot(t, capsB.speed,'LineWidth',1.2);
plot(t, capsB.dc,'LineWidth',1.2);
plot(t, capsB.mu,'LineWidth',1.2);
plot(t, capsB.final,'LineWidth',1.6);
grid on; xlabel('Time [s]'); ylabel('Torque [Nm]');
legend('T_{req}','P/\omega cap','DC cap','\mu cap','final cap','Location','best');
title('Limiter stack (Alg B): request vs caps');

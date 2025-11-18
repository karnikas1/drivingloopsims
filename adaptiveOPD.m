function [T_req, dbg] = adaptiveOPD(apps, v, P, state)
% Adaptive OPD: target decel + μFz + paddle shaping

m = P.veh.m; g = P.veh.g; Rw = P.veh.Rw; mu = P.veh.mu;
dead = P.maps.deadband;
x  = max(0, min(1, (apps - dead)/(1 - dead)));
u  = max(0, min(1, (dead - apps)/max(dead,1e-6)));

% Regen gate (throttle hysteresis)
if ~state.regen_en
    if apps < P.opd.apps_enable,  regen_ok = true;  else, regen_ok = false; end
else
    if apps > P.opd.apps_disable, regen_ok = false; else, regen_ok = true;  end
end

if x > 0
    T_req =  P.mot.Tmax_const * P.maps.f_drive(x);              % + torque
elseif regen_ok
    % decel target fades near crawl; tune 0.45..0.55 g and "v/(v+τ)"
    j_target = -(0.50*9.81) * (v/(v+6));
    F_need   = m * j_target;                      % negative
    Fz_rear  = 0.5*m*g;  F_mu = mu*Fz_rear;       % envelope
    F_req    = max(-F_mu, min(0, F_need));
    T_req    = max(-P.mot.Tmax_const, F_req*Rw);
    % paddle: magnitude + early-lift shaping
    T_req    = T_req * P.opd.k_regen * (u^P.opd.gammaR_user);
else
    T_req = 0;
end

dbg = struct('regen_ok',regen_ok,'omega', v/max(Rw,1e-6));
end
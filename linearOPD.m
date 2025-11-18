function [T_req, dbg] = linearOPD(apps, v, P, state)
% Simple linear OPD: fixed regen proportional to lift

Rw = P.veh.Rw;  dead = P.maps.deadband;
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
    T_req = -0.65*P.mot.Tmax_const * (P.maps.f_regen(u)^P.opd.gammaR_user) ...
            * P.opd.k_regen;
else
    T_req = 0;
end

dbg = struct('regen_ok',regen_ok,'omega', v/max(Rw,1e-6));
end
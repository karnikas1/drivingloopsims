function [T_cmd, caps, slip, Pregen] = applyLimitStack(T_req, v, P, state)
% Min-based physical caps + multiplicative soft derates + simple slip gate

Rw = P.veh.Rw;  w  = v/max(Rw,1e-6);
m  = P.veh.m;   g  = P.veh.g;

% 1) Torque-speed (Pmax/ω) and HW torque cap
T_cap_speed = sign(T_req) * min(abs(P.mot.Tmax_const), P.mot.Pmax_mech/max(w,P.mot.w_floor));

% 2) DC caps (discharge vs charge)
if T_req >= 0
    V = max(P.pack.Vmin, P.pack.Voc - P.pack.Idc_dis_max*P.pack.R);
    Pmax_elec = P.mot.eta_mot * V * P.pack.Idc_dis_max;
    T_cap_dc  =  Pmax_elec / max(w, P.mot.w_floor);
else
    V = min(P.pack.Vmax, P.pack.Voc + P.pack.Idc_chg_max*P.pack.R);
    Pmax_elec = P.mot.eta_gen * V * P.pack.Idc_chg_max;
    T_cap_dc  = -Pmax_elec / max(w, P.mot.w_floor);
end

% 3) μ envelope (rear axle)
T_cap_mu = P.veh.mu * (0.5*m*g) * Rw;
if T_req < 0, T_cap_mu = -T_cap_mu; end

% Combine physical caps
if T_req >= 0
    T_cap_phys = min([T_cap_speed, T_cap_dc, T_cap_mu]);
else
    T_cap_phys = max([T_cap_speed, T_cap_dc, T_cap_mu]);
end

% 4) Soft derates multiply the cap (temp/SOC/track)
soft = P.derate.f_temp * P.derate.f_soc * P.derate.f_track;
T_cap = soft * T_cap_phys;

% 5) Slip gate (toy: use rim=car speed → slip≈0; placeholder)
slip = 0;
if (v > 0.5) && (T_req < 0) && (slip < -0.20)
    T_cap = 0;
end

% Final torque
T_cmd = sign(T_req) * min(abs(T_req), abs(T_cap));

% Regen electrical power (≥0 into pack)
Pmech = w*T_cmd;
Pregen = (Pmech<0) * (-P.mot.eta_gen*Pmech);

caps = struct('speed',T_cap_speed,'dc',T_cap_dc,'mu',T_cap_mu,'soft',soft,'phys',T_cap_phys,'final',T_cap);
end
function T_regen = regen_limit(omega, soc, vpack, P)
%% Regen Limit: compute how much regen torque is safe? 
% how much regen torque is safe
if omega < P.Regen.cutoff_rpm*2*pi/60 || soc > P.Regen.soc_disable
    T_regen = 0; return;
end
%inverter power?
I_avail = P.Regen.i_chg_max;
%pack power? 
P_avail = vpack * I_avail;
%final regen torque, will be negative torque
T_regen = min(P_avail / max(omega,1), P.Regen.max_nm);
end

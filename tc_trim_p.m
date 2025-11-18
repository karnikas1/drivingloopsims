function T_out = tc_trim_p(T_in, u, P)
% slip = (ωR - v)/v; trim torque if |slip| above target (simple P)
v_est   = max(u.v, 0.1);
omega_w = mean(u.wheelSpeeds);
v_wheel = omega_w * P.Vehicle.r_wheel;
slip    = (v_wheel - v_est)/v_est;

if v_est*3.6 < P.TC.min_speed
    T_out = T_in; return;
end

e = abs(slip) - P.TC.slip_target;   % >0 means too slippy
if e > 0
    T_out = T_in * max(0, 1 - P.TC.gain*e);
else
    T_out = T_in; % below target then don’t add torque, just leave it
end
end


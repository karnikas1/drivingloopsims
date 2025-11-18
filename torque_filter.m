function T = torque_filter(T_now, T_last, F)
% smooth torque steps
if T_now > T_last
    T = T_last + F.tipin*(T_now-T_last);
else
    T = T_last + F.tipout*(T_now-T_last);
end
end

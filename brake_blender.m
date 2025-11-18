function [T_req, F_front, F_rear] = brake_blender(u, P)
% split braking, try to fill rear with regen

F_tot = u.brake * 1.2 * P.Vehicle.mass * 9.81;
k_front = 0.6;
F_front = k_front * F_tot;
F_rear  = (1-k_front) * F_tot;

T_rear = F_rear * P.Vehicle.r_wheel / P.Vehicle.gear;
T_regen = regen_limit(u.omega, u.soc, u.vpack, P);

T_req = -min(T_rear, T_regen);
end

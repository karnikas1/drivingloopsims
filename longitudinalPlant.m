function [v_next, a, forces] = longitudinalPlant(v, T_axle, P, dt)
% Rear-axle torque → acceleration (1D longitudinal)

Rw = P.veh.Rw; m = P.veh.m; g = P.veh.g;
FAxle = T_axle / Rw;

Fdrag = 0.5*P.veh.rho*P.veh.CxA*v^2;
Frr   = m*g*P.veh.Crr*sign(v + 1e-6);

a = (FAxle - Fdrag - Frr)/m;
v_next = max(0, v + a*dt);

forces = struct('axle',FAxle,'drag',Fdrag,'roll',Frr);
end

function [omega_new, v_new] = longitudinalPlant(T_cmd, omega, dt, P)
% 1D longitudinal plant with aero drag & rolling resistance.

% states -> current speed
v  = max(0, omega * P.Vehicle.r_wheel); % m/s

% drive force from torque
F_drive = (T_cmd * P.Vehicle.gear) / P.Vehicle.r_wheel;  % N

% resistances
rho = P.Plant.rho; CdA = P.Plant.CdA; Crr = P.Plant.Crr; m = P.Vehicle.mass;
F_aero = 0.5 * rho * CdA * v^2; % opposes motion
F_roll = Crr * m * 9.81 * (v>0); % very small near standstill

% net force & integrate
F_net = F_drive - F_aero - F_roll;
a     = F_net / m;

v_new     = max(0, v + a*dt); % no backwards for now
omega_new = v_new / P.Vehicle.r_wheel; % rad/s
end



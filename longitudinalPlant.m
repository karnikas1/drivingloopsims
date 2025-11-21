function [omega_new, v_new] = longitudinalPlant(T_cmd, omega, dt, P)
%% LONGITUDINALPLANT
% Simple 1-D longitudinal vehicle model:
%  -Converts axle torque → drive force
%  -Applies aero drag + rolling resistance
%  -Integrates acceleration → new speed and wheel omega
%% Assumptions:
%  -No wheel slip model here (pure kinematic link: v = ω * R)
%  -No drivetrain elasticity (rigid connection)
%  -No reverse motion (v is clamped ≥ 0)
%% Inputs:
%   T_cmd = torque at motor shaft (post caps/filters) [Nm]
%   omega = wheel rotational speed [rad/s]
%   dt = timestep [s]
%   P = parameter struct (Vehicle + Plant)
%% Outputs:
%   omega_new = updated wheel angular speed [rad/s]
%   v_new = updated vehicle longitudinal speed [m/s]

%% CURRENT SPEED FROM WHEEL OMEGA
% Convert wheel rotational speed to vehicle speed
v = max(0, omega * P.Vehicle.r_wheel);   % m/s
% (Use max() to avoid small negative numerical artifacts)

%% DRIVE FORCE FROM TORQUE 
% Torque to wheel force:
% F = (T * gear_ratio) / wheel_radius
F_drive = (T_cmd * P.Vehicle.gear) / P.Vehicle.r_wheel;   % N

%% RESISTIVE FORCES
% Extract constants
rho = P.Plant.rho; % air density [kg/m^3]
CdA = P.Plant.CdA; % drag area [m^2]
Crr = P.Plant.Crr; % rolling resistance coefficient
m   = P.Vehicle.mass; % vehicle mass [kg]

% Aerodynamic drag: F = 0.5 * rho * CdA * v^2
F_aero = 0.5 * rho * CdA * v^2; % opposes motion

% Rolling resistance: F = Crr * m * g
% Only applied if moving (avoids fighting start-up)
F_roll = Crr * m * 9.81 * (v > 0);       % N

%% NET FORCE & ACCELERATION
F_net = F_drive - F_aero - F_roll; % total longitudinal force
a = F_net / m; % acceleration [m/s^2]

%% INTEGRATE SPEED (NO REVERSE FOR NOW) 
v_new = max(0, v + a*dt); % integrate forward Euler

%% UPDATE WHEEL OMEGA 
omega_new = v_new / P.Vehicle.r_wheel; % simple v = wR relation

end




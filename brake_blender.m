function [T_req, F_front, F_rear] = brake_blender(u, P)
%% BRAKE_BLENDER
% Simple brake-force allocator:
% -Compute total braking force from pedal input
% -Split braking between front/rear (fixed bias)
% -Convert rear braking force to torque at the motor
% -Limit rear braking by available regen capability
%
%% Inputs (u):
%   u.brake   = brake pedal input (0–1)
%   u.omega   = wheel speed [rad/s]
%   u.soc     = pack state-of-charge
%   u.vpack   = battery voltage
%
%% Inputs (P):
%   P.Vehicle.mass     = vehicle mass [kg]
%   P.Vehicle.r_wheel  = wheel radius [m]
%   P.Vehicle.gear     = gear ratio motor→wheel
%
% regen_limit(...) is assumed to return the maximum *negative* torque 
% the motor–battery system can pull based on speed, SOC, and pack voltage.
%
%% Output:
%   T_req   = requested regen torque (negative)
%   F_front = front braking force [N]
%   F_rear  = rear braking force [N]

%% 1) Compute total braking force from pedal input 
% "1.2 * m * g" is the nominal maximum decel target (~1.2g braking)
F_tot = u.brake * 1.2 * P.Vehicle.mass * 9.81;   % [N]

%% 2) Fixed brake bias front/rear 
% Simple fixed -% split (e.g., 60% front, 40% rear)
k_front = 0.6; % fraction of braking sent to front axle
F_front = k_front   * F_tot;
F_rear  = (1-k_front) * F_tot;

%% 3) Convert rear brake force to rear wheel torque 
% T = F * r / gear  (gear reduces motor torque at the wheel)
T_rear = F_rear * P.Vehicle.r_wheel / P.Vehicle.gear;

%% 4) Compute available regen torque 
% regen_limit() returns the *maximum available* regen torque magnitude.
% Units should match T_rear (Nm). Positive number means "available torque".
T_regen = regen_limit(u.omega, u.soc, u.vpack, P);

%% 5) Final regen request
% apply whichever limit is lower: mechanical brake demand or regen capability.
% Negate because regen torque is negative (braking).
T_req = -min(T_rear, T_regen);

end


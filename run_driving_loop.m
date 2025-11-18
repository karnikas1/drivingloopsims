function [out, state] = run_driving_loop(u, P, state, mode)
% pedal/brake → torque req → regen + TC + caps → torque cmd → plant

% stable output shape
out = struct('T_req',0,'T_cmd',0,'F_front',0,'F_rear',0,'omega',0,'v',0, ...
             'caps',struct('rpm',0,'power',0,'derate',1));

% pedal to torque request (map is per-mode)
modeParams = P.Mode.(mode); 
modeParams.v = u.v;                                   % adaptive uses speed
T_drive_cap = min(P.Torque.max, (P.Power.max*1000)/max(u.omega,1)); % Nm
T_req = pedal_map(u.pedal, modeParams, T_drive_cap);  % <-- pass cap

% APPS+BPS safety inhibit
if (u.brake > 0.05) && (u.pedal > 0.25), T_req = 0; end

% 3) braking / lift-regen
if u.brake > 0
    [T_req, out.F_front, out.F_rear] = brake_blender(u, P);
elseif u.pedal < 0.05
    T_req = -min(P.Regen.max_nm, regen_limit(u.omega, u.soc, u.vpack, P));
end

% traction control
%if P.TC.enable, T_req = tc_trim(T_req, u, P); end
% new:
if P.TC.enable
    T_req = tc_trim(T_req, u, P, mode);
end

% caps, 6) smoothing, 7) plant, logs, state (unchanged)
[T_cmd, caps] = apply_caps(T_req, u, P);
T_cmd = torque_filter(T_cmd, state.last_T, P.Filter);
[out.omega, out.v] = longitudinalPlant(T_cmd, state.omega, u.dt, P);
out.T_req = T_req; out.T_cmd = T_cmd; out.caps = caps;
state.last_T = T_cmd; state.omega = out.omega;
end





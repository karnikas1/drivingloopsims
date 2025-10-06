function T = pedal_map(pedal, modeParams, T_drive_cap)
% Map pedal% to actual torque using a chosen map shape.
% Inputs:
%   pedal         : scalar in [0..1]
%   modeParams    : struct with fields (some optional depending on type)
%       .type        : 'linear' | 'adaptive' | 'power' | 'smoothstep' | 'bezier'
%       .pedal_gain  : scalar gain (default 1.0)
%       .v           : vehicle speed [m/s] (used by 'adaptive')
%       .curve_s     : exponent for 'power' (e.g., 1.3 softer, 0.8 snappier)
%       .bezier      : 2x2 control points for cubic Bezier:
%                      [c1x c1y; c2x c2y], all in [0..1], monotonic along y
%       .deadband    : optional deadband on pedal (0..1), default 0
%
%   T_drive_cap   : available positive torque cap (Nm) for “drive side”
%
% Output:
%   T             : real wheel torque command (Nm), >= 0 here (drive side)

% defaults
if ~isfield(modeParams,'type'),       modeParams.type       = 'linear'; end
if ~isfield(modeParams,'pedal_gain'), modeParams.pedal_gain = 1.0;      end
if ~isfield(modeParams,'v'),          modeParams.v          = 0;        end
if ~isfield(modeParams,'curve_s'),    modeParams.curve_s    = 1.0;      end
if ~isfield(modeParams,'deadband'),   modeParams.deadband   = 0.0;      end

% apply a simple deadban
p = max(0, (pedal - modeParams.deadband) / max(1e-6, 1 - modeParams.deadband));
p = min(1, p);

% choose map 
switch lower(modeParams.type)

  case 'linear'
    alpha = modeParams.pedal_gain * p;                               % 0..>1

  case 'adaptive'
    % softer at low speed: fade ≈ v / v_fade, your original min(1, v/5)
    fade  = min(1, modeParams.v / 5);
    alpha = modeParams.pedal_gain * p * fade;

  case 'power'
    % power-law shaping: s>1 softer tip-in; s<1 snappier
    s     = max(0.1, modeParams.curve_s);
    alpha = modeParams.pedal_gain * (p.^s);

  case 'smoothstep'
    % cubic smoothstep: 3p^2 - 2p^3 (nice gentle start/finish)
    sm    = 3*p.^2 - 2*p.^3;
    alpha = modeParams.pedal_gain * sm;

  case 'bezier'
    % cubic Bezier on unit square: (0,0) -> c1 -> c2 -> (1,1)
    % We treat pedal as the curve parameter (monotonic preset).
    if ~isfield(modeParams,'bezier') || ~isequal(size(modeParams.bezier),[2 2])
      error('Bezier map requires modeParams.bezier = [c1x c1y; c2x c2y].');
    end
    c1 = modeParams.bezier(1,:);   % [x y]
    c2 = modeParams.bezier(2,:);   % [x y]
    t  = p;                         % treat pedal as parameter
    % Bernstein form for y(t) only (x is ignored – assumed monotonic UI)
    B0 = (1-t).^3;
    B1 = 3*(1-t).^2.*t;
    B2 = 3*(1-t).*(t.^2);
    B3 = t.^3;
    y  = B0*0 + B1*c1(2) + B2*c2(2) + B3*1;      % y ∈ [0..1]
    alpha = modeParams.pedal_gain * y;

  otherwise
    error('Unknown pedal map type: %s', modeParams.type);
end

% clamp and scale by available drive cap
alpha = max(0, alpha);
T     = min(alpha, 1) * T_drive_cap;     % ensure we don’t exceed cap here
end

%testing
% clear
% clear functions
% clear; close all; clear functions
% P = params_default();
% P.Mode.accel.type       = 'linear';
% P.Mode.accel.pedal_gain = 1.0;
% pedal_map_sweep(P,'accel',[10 40 80], true,  true);
% P.Mode.accel.pedal_gain = 1.5;
% pedal_map_sweep(P,'accel',[10 40 80], true,  true);
% P.Mode.accel.type       = 'adaptive';
% P.Mode.accel.pedal_gain = 1.0;
% pedal_map_sweep(P,'accel',[10 40 80], true,  true);
% P.Mode.accel.type       = 'adaptive';
% P.Mode.accel.pedal_gain = 1.0;
% pedal_map_sweep(P,'accel',[10 40 80],true);
% P.Mode.accel.type       = 'adaptive';
% P.Mode.accel.pedal_gain = 1.5;
% pedal_map_sweep(P,'accel',[10 40 80],true);
% P.Mode.accel.type       = 'power';
% P.Mode.accel.curve_s    = 1.7;          % try 1.3 / 1.7 / 2.2
% P.Mode.accel.pedal_gain = 1.0;
% pedal_map_sweep(P,'accel',[10 40 80], false, true);  % map only
% pedal_map_sweep(P,'accel',[10 40 80], true,  true);  % map + caps
% P.Mode.accel.type       = 'power';
% P.Mode.accel.curve_s    = 1.3;          % try 1.3 / 1.7 / 2.2
% P.Mode.accel.pedal_gain = 1.0;
% pedal_map_sweep(P,'accel',[10 40 80], false, true);  % map only
% pedal_map_sweep(P,'accel',[10 40 80], true,  true);  % map + caps
% P.Mode.accel.type       = 'power';
% P.Mode.accel.curve_s    = 2.2;          % try 1.3 / 1.7 / 2.2
% P.Mode.accel.pedal_gain = 1.0;
% pedal_map_sweep(P,'accel',[10 40 80], false, true);  % map only
% pedal_map_sweep(P,'accel',[10 40 80], true,  true);  % map + caps
% P.Mode.accel.type       = 'smoothstep';
% P.Mode.accel.pedal_gain = 1.0;
% pedal_map_sweep(P,'accel',[10 40 80], false, true);
% P.Mode.accel.type       = 'smoothstep';
% P.Mode.accel.pedal_gain = 1.5;
% pedal_map_sweep(P,'accel',[10 40 80], false, true);
% P.Mode.accel.type       = 'bezier';
% P.Mode.accel.bezier     = [0.25 0.85; 0.70 0.95];    % [c1x c1y; c2x c2y], in [0,1]
% P.Mode.accel.pedal_gain = 1.0;
% pedal_map_sweep(P,'accel',[10 40 80], false, true);
% P.Mode.accel.type       = 'bezier';
% P.Mode.accel.bezier     = [0.25 0.85; 0.70 0.95];    % [c1x c1y; c2x c2y], in [0,1]
% P.Mode.accel.pedal_gain = 1.5;
% pedal_map_sweep(P,'accel',[10 40 80], false, true);

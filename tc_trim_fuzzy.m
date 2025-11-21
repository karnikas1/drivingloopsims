function T_out = tc_trim_fuzzy(T_in, u, P)
%% Tiny fuzzy TC: should have smoother torque transition and less ocsillation
% Inputs: slip error e = |slip|-target; slip rate r = d|e|/dt (discrete)
% Output: trim factor f∈[0..1] multiplying T_in.

persistent e_prev
if isempty(e_prev); e_prev = 0; end

v_est   = max(u.v, 0.1);
omega_w = mean(u.wheelSpeeds);
v_wheel = omega_w * P.Vehicle.r_wheel;
slip    = (v_wheel - v_est)/v_est;

% ignore at crawl speeds
if v_est*3.6 < P.TC.min_speed
    T_out = T_in; e_prev = 0; return;
end

e = max(0, abs(slip) - P.TC.slip_target);  % >0 means too much slip
r = e - e_prev;                             % discrete derivative
e_prev = e;

% membership functions (triangles)
% error (LOW, MED, HIGH) over [0..0.30]
muE.LOW  = tri(e, 0.00, 0.02, 0.08);
muE.MED  = tri(e, 0.02, 0.08, 0.14);
muE.HIGH = tri(e, 0.10, 0.20, 0.30);

% rate (NEG, ZERO, POS) over [-0.10..0.10]
muR.NEG  = tri(r, -0.10, -0.06, -0.02);
muR.ZERO = tri(r, -0.03,  0.00,  0.03);
muR.POS  = tri(r,  0.02,  0.06,  0.10);

% rules 
% STRONG: e HIGH or (e MED & r POS)
w_STRONG = max(muE.HIGH, min(muE.MED, muR.POS));
% MED: e MED & (r ZERO or NEG)
w_MED    = min(muE.MED, max(muR.ZERO, muR.NEG));
% LIGHT: e LOW & r NEG (we're recovering, ease off trim)
w_LIGHT  = min(muE.LOW, muR.NEG);
% ZERO: whatever is left
w_ZERO   = max(0, 1 - max([w_STRONG, w_MED, w_LIGHT]));

% output (trim factor f): 1=no trim, 0=kill torque
f = (w_ZERO*1.00 + w_LIGHT*0.85 + w_MED*0.60 + w_STRONG*0.30) ...
    / (w_ZERO + w_LIGHT + w_MED + w_STRONG + eps);

T_out = T_in * max(0, min(1, f));
end

function m = tri(x, a, b, c)
% triangular membership
    if x<=a || x>=c
      m=0;
    elseif x==b     
        m=1;
    elseif x<b      
        m=(x-a)/(b-a);
    else             
        m=(c-x)/(c-b);
    end
end


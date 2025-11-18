function T_out = tc_trim(T_in, u, P, mode)
% Pick which TC to use: P or Fuzzy. Mode override beats global.
% Usage from run_driving_loop:  tc_trim(T_req, u, P, mode)

% default = global setting
tc_type = 'p';
if isfield(P,'TC') && isfield(P.TC,'type') && ~isempty(P.TC.type)
    tc_type = lower(P.TC.type);
end

% per-mode override if provided
if nargin>=4 && isfield(P,'Mode') && isfield(P.Mode,(mode))
    m = P.Mode.(mode);
    if isfield(m,'tc_type') && ~isempty(m.tc_type)
        tc_type = lower(m.tc_type);
    end
end

switch tc_type
    case {'p','pctrl','prop'}
        T_out = tc_trim_p(T_in, u, P);
    case {'fuzzy','fz'}
        T_out = tc_trim_fuzzy(T_in, u, P);
    case {'off','none'}
        T_out = T_in;                 % no TC
    otherwise
        % unknown type -> be safe, apply P
        T_out = tc_trim_p(T_in, u, P);
end
end

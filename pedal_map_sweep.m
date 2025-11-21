function pedal_map_sweep(P, mode, speeds_kmh, with_caps, show_per_speed)
%% PEDAL_MAP_SWEEP
% Generates T_req and T_cmd vs pedal-position curves at multiple vehicle
% speeds for a chosen driving mode (accel / autoX / enduro / etc.).
%
%% Useful for:
%  -visualizing pedal-map shape
%  -verifying drive/regen cap behavior
%  -tuning adaptive OPD response
%  -producing plots for design docs / DRs
%
%% Outputs:
%  -One combined figure: T_cmd vs pedal (one line per speed)
%  -Optional: one subplot per speed with T_req and T_cmd separated
%
%% Usage:
%   pedal_map_sweep(P,'accel',[10 40 80], true,  true)
%   pedal_map_sweep(P,'accel',[10 40 80], false, true)
% -------------------------------------------------------------------------

%% INPUT HANDLING
if nargin < 2 || isempty(mode)
    mode = 'accel';
end
if nargin < 3 || isempty(speeds_kmh)
    speeds_kmh = [10 40 80];
end
if nargin < 4 || isempty(with_caps)
    with_caps = true;
end
if nargin < 5 || isempty(show_per_speed)
    show_per_speed = true;
end

% Validate mode
if ~isfield(P,'Mode') || ~isfield(P.Mode, mode)
    error('P.Mode.%s not found. Check params_default.m', mode);
end

% Extract mode parameters (pedal gain, map type, adaptive settings, etc.)
MP = P.Mode.(mode);
if ~isfield(MP,'type'),       MP.type       = 'linear'; end
if ~isfield(MP,'pedal_gain'), MP.pedal_gain = 1.0;      end

%% PREALLOCATIONS
ped      = linspace(0,1,101);      % pedal positions
ns       = numel(speeds_kmh);

T_all    = nan(ns, numel(ped));    % T_cmd for each speed row
req_all  = T_all;                  % T_req for reference plot

P = ensure_defaults(P);            % fill missing parameters with defaults

%% MAIN LOOP: SWEEP SPEEDS
for i = 1:ns
    v_kmh = speeds_kmh(i);
    v_ms  = v_kmh / 3.6;

    % Build input struct "u" with fields expected by pedal_map + caps
    u = struct();
    u.pedal = 0;
    u.brake = 0;
    u.v     = v_ms;
    u.omega = max(v_ms / P.Vehicle.r_w, 1);
    u.soc   = P.Batt.soc0;
    u.vpack = P.Batt.vpack0;
    u.temps = P.Temp.init(:).';
    u.dt    = P.dt;

    % Pass speed into mode parameters (adaptive maps need this)
    MP.v = v_ms;

    % Pass coast-zone configuration to map if present
    if isfield(P,'opd') && isfield(P.opd,'coast')
        MP.coast = P.opd.coast;
    else
        MP.coast = struct('enable',false);
    end

    % Limit the drive region based on torque-speed envelope
    T_drive_cap = local_drive_cap(P, u.omega);

    %% Pedal sweep at this speed
    for k = 1:numel(ped)
        u.pedal = ped(k);

        % Raw pedal-to-torque (no caps)
        T_req = pedal_map(u.pedal, MP, T_drive_cap);
        req_all(i,k) = T_req;

        % Apply torque caps (drive envelope, regen envelope, derates, etc.)
        if with_caps
            na = nargin('apply_caps');
            no = nargout('apply_caps');

            if na <= 3
                if no >= 2
                    [T_cmd,~] = apply_caps(T_req, u, P);
                else
                    T_cmd = apply_caps(T_req, u, P);
                end
            else
                % apply_caps variant that needs "state"
                state = struct('omega', u.omega, 'last_T', 0);
                if no >= 2
                    [T_cmd,~] = apply_caps(T_req, u, P, state);
                else
                    T_cmd = apply_caps(T_req, u, P, state);
                end
            end
        else
            T_cmd = T_req;
        end

        T_all(i,k) = T_cmd;
    end
end

%% FIGURE 1: OVERLAID MAPS
figure('Name','Tcmd vs pedal (all speeds)');
hold on; box on; grid on;

clr = lines(ns);
for i = 1:ns
    plot(ped, T_all(i,:), 'LineWidth',2, 'Color',clr(i,:));
end

xlabel('pedal [%]');
ylabel('T_{cmd} [Nm]');
title(sprintf('T_{cmd} vs pedal (map%s caps), mode=%s', ...
      tern(with_caps,' +',' w/o'), mode));
legend(arrayfun(@(x) sprintf('%g km/h',x), speeds_kmh,'uniformOutput',false), ...
       'Location','SouthEast');
set(gca,'XLim',[0 1]);

%% FIGURE 2: PER-SPEED PANELS
if show_per_speed
    figure('Name','Tcmd vs pedal per speed');
    ncol = max(1, ceil(sqrt(ns)));
    nrow = ceil(ns/ncol);

    for i = 1:ns
        subplot(nrow,ncol,i);
        plot(ped, req_all(i,:), '--','Color',[.6 .6 .6],'LineWidth',1); hold on;
        plot(ped, T_all(i,:), 'b','LineWidth',2);
        grid on; box on;
        title(sprintf('%g km/h', speeds_kmh(i)));
        xlabel('pedal [%]');
        ylabel('Nm');
        legend('T_{req}','T_{cmd}','Location','SouthEast');
        set(gca,'XLim',[0 1]);
    end
end
end


%% HELPER FUNCTIONS
function P = ensure_defaults(P)
% Fill missing fields with safe defaults so pedal_map_sweep never errors.

if ~isfield(P,'dt'), P.dt = 0.01; end
if ~isfield(P,'Vehicle'), P.Vehicle = struct(); end
if ~isfield(P.Vehicle,'r_w'),      P.Vehicle.r_w = 0.3; end

if ~isfield(P,'Batt'),          P.Batt = struct(); end
if ~isfield(P.Batt,'soc0'),     P.Batt.soc0   = 0.7; end
if ~isfield(P.Batt,'vpack0'),   P.Batt.vpack0 = 400; end

if ~isfield(P,'Temp'),          P.Temp = struct(); end
if ~isfield(P.Temp,'init'),     P.Temp.init = [30 30 30]; end

if ~isfield(P,'Torque'),        P.Torque = struct(); end
if ~isfield(P.Torque,'max'),    P.Torque.max = 300; end

if ~isfield(P,'Power'),         P.Power = struct(); end
if ~isfield(P.Power,'max'),     P.Power.max = 80; end   % kW

if ~isfield(P,'Drivetrain'),    P.Drivetrain = struct(); end
if ~isfield(P.Drivetrain,'eta_m'), P.Drivetrain.eta_m = 0.9; end
end


function Tcap = local_drive_cap(P, omega)
% Drive cap = min( max torque , power-limited torque )
Tcap = P.Torque.max;

if isfield(P,'Power') && isfield(P.Power,'max')
    T_power = (P.Power.max * 1000) / max(omega,1);
    Tcap = min(Tcap, T_power);
end
end


function s = tern(tf, A, B)
% Tiny ternary helper: s = A if tf else B
if tf, s = A; else, s = B; end
end

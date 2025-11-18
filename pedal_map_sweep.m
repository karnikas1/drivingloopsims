function pedal_map_sweep(P, mode, speeds_kmh, with_caps, show_per_speed)
% PEDAL_MAP_SWEEP  Plot T_cmd vs pedal for a given driving "mode".
% Usage:
%   pedal_map_sweep(P,'accel',[10 40 80], true,  true)
%   pedal_map_sweep(P,'accel',[10 40 80], false, true)

if nargin < 2 || isempty(mode),           mode = 'accel';           end
if nargin < 3 || isempty(speeds_kmh),     speeds_kmh = [10 40 80];  end
if nargin < 4 || isempty(with_caps),      with_caps = true;         end
if nargin < 5 || isempty(show_per_speed), show_per_speed = true;    end

if ~isfield(P,'Mode') || ~isfield(P.Mode, mode)
    error('P.Mode.%s not found. Check params_default.m', mode);
end
MP = P.Mode.(mode);
if ~isfield(MP,'type'),       MP.type       = 'linear'; end
if ~isfield(MP,'pedal_gain'), MP.pedal_gain = 1.0;      end

ped   = linspace(0,1,101);
ns    = numel(speeds_kmh);
T_all = nan(ns, numel(ped));   % T_cmd vs pedal
req_all = T_all;               % T_req vs pedal

P = ensure_defaults(P);

for i = 1:ns
    v_kmh = speeds_kmh(i);
    v_ms  = v_kmh / 3.6;

    u = struct();
    u.pedal = 0; u.brake = 0;
    u.v = v_ms; u.omega = max(v_ms / P.Vehicle.r_w, 1);
    u.soc = P.Batt.soc0; u.vpack = P.Batt.vpack0;
    u.temps = P.Temp.init(:).'; u.dt = P.dt;

    MP.v = v_ms;   % let adaptive map see the speed

    %pass coast parameters into map
    if isfield(P,'opd') && isfield(P.opd,'coast')
        MP.coast = P.opd.coast;       % pedal_map checks modeParams.coast.enable
    else
        % (optional) default: coast off if not defined in P
        MP.coast = struct('enable',false);
    end

    T_drive_cap = local_drive_cap(P, u.omega);

    for k = 1:numel(ped)
        u.pedal = ped(k);

        T_req = pedal_map(u.pedal, MP, T_drive_cap);
        req_all(i,k) = T_req;

        if with_caps
            % call your (now patched) apply_caps
            na = nargin('apply_caps'); no = nargout('apply_caps');
            if na <= 3
                if no >= 2, [T_cmd,~] = apply_caps(T_req, u, P);
                else,       T_cmd     = apply_caps(T_req, u, P);
                end
            else
                state = struct('omega', u.omega, 'last_T', 0);
                if no >= 2, [T_cmd,~] = apply_caps(T_req, u, P, state);
                else,       T_cmd     = apply_caps(T_req, u, P, state);
                end
            end
        else
            T_cmd = T_req;
        end

        T_all(i,k) = T_cmd;
    end
end

% One combined figure (overlaid)
figure('Name','Tcmd vs pedal (all speeds)');
hold on; box on; grid on;
clr = lines(ns);
for i = 1:ns, plot(ped, T_all(i,:), 'LineWidth',2, 'Color',clr(i,:)); end
xlabel('pedal [%]'); ylabel('T_{cmd} [Nm]');
title(sprintf('T_{cmd} vs pedal (map%s caps), mode=%s', tern(with_caps,' +',' w/o'), mode));
legend(arrayfun(@(x) sprintf('%g km/h',x), speeds_kmh,'uni',0),'Location','SouthEast');
set(gca,'XLim',[0 1]);

% Small multiples per speed (optional)
if show_per_speed
    figure('Name','Tcmd vs pedal per speed');
    ncol = max(1, ceil(sqrt(ns))); nrow = ceil(ns/ncol);
    for i = 1:ns
        subplot(nrow,ncol,i);
        plot(ped, req_all(i,:), '--','Color',[.6 .6 .6],'LineWidth',1); hold on;
        plot(ped, T_all(i,:), 'b','LineWidth',2);
        grid on; box on;
        title(sprintf('%g km/h', speeds_kmh(i)));
        xlabel('pedal [%]'); ylabel('Nm');
        legend('T_{req}','T_{cmd}','Location','SouthEast');
        set(gca,'XLim',[0 1]);
    end
end
end

% ---------- helpers ----------
function P = ensure_defaults(P)
if ~isfield(P,'dt'), P.dt = 0.01; end
if ~isfield(P,'Vehicle'), P.Vehicle = struct(); end
if ~isfield(P.Vehicle,'r_w'),      P.Vehicle.r_w = 0.3; end
if ~isfield(P,'Batt'),   P.Batt = struct(); end
if ~isfield(P.Batt,'soc0'),    P.Batt.soc0 = 0.7; end
if ~isfield(P.Batt,'vpack0'),  P.Batt.vpack0 = 400; end
if ~isfield(P,'Temp'),   P.Temp = struct(); end
if ~isfield(P.Temp,'init'),    P.Temp.init = [30 30 30]; end
if ~isfield(P,'Torque'), P.Torque = struct(); end
if ~isfield(P.Torque,'max'),   P.Torque.max = 300; end
if ~isfield(P,'Power'),  P.Power = struct(); end
if ~isfield(P.Power,'max'),    P.Power.max = 80; end % kW
if ~isfield(P,'Drivetrain'), P.Drivetrain = struct(); end
if ~isfield(P.Drivetrain,'eta_m'), P.Drivetrain.eta_m = 0.9; end
end

function Tcap = local_drive_cap(P, omega)
Tcap = P.Torque.max;
if isfield(P,'Power') && isfield(P.Power,'max')
    T_power = (P.Power.max*1000) / max(omega,1);
    Tcap = min(Tcap, T_power);
end
end

function s = tern(tf, A, B), if tf, s = A; else, s = B; end, end





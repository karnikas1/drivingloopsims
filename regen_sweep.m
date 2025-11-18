function regen_sweep(P, mode, rpm_list, varargin)
% regen_sweep  Sweep brake% -> regen torque at fixed RPMs (single-step, no plant)
%
% To run:
%   P = params_default();
%   regen_sweep(P, 'accel', [1000 2000 4000]);
%
% This uses run_driving_loop() for a single compute step with
% fixed wheel speed; it does not advance the vehicle model.

% options
soc    = getOpt(varargin,'soc',   existOr(getfield_safe(P,'Batt','soc0'), 0.6));
vpack  = getOpt(varargin,'vpack', existOr(getfield_safe(P,'Batt','vpack0'), 400));
temps  = getOpt(varargin,'temps', [30 30 30]);
nSamp  = getOpt(varargin,'n',     101);

rpm_list = rpm_list(:)';
nR = numel(rpm_list);
br = linspace(0,1,nSamp);                 % brake percentage 0..1

%storage
Treq = zeros(nR, nSamp);
Tcmd = zeros(nR, nSamp);

%sweep
for i = 1:nR
    rpm   = rpm_list(i);
    omega = rpm * 2*pi/60;                % rad/s
    v     = omega * P.Vehicle.r_wheel;    % m/s

    % keep a tiny dt; we’re not advancing plant, filter effects minimal
    dt = 0.01;

    state.last_T = 0;
    state.omega  = omega;

    for k = 1:nSamp
        u = struct( ...
            'pedal',        0.0, ...      % foot off throttle for pure regen
            'brake',        br(k), ...
            'wheelSpeeds',  omega, ...    % WSS ~= motor ω (stub)
            'soc',          soc, ...
            'vpack',        vpack, ...
            'temps',        temps, ...
            'dt',           dt, ...
            'v',            v, ...
            'omega',        omega);

        [out, ~] = run_driving_loop(u, P, state, mode);

        Treq(i,k) = out.T_req;
        Tcmd(i,k) = out.T_cmd;
    end
end

% plots: per-RPM panel
figure('Name','Regen caps (per RPM)'); clf;
ncol = 2;
nrow = ceil(nR/ncol);
for i = 1:nR
    subplot(nrow,ncol,i); hold on; grid on;
    plot(br*100, Treq(i,:), 'k--', 'LineWidth', 1.1, 'DisplayName','T_{req}');
    plot(br*100, Tcmd(i,:), 'b-',  'LineWidth', 1.8, 'DisplayName','T_{cmd}');
    xlabel('brake [%]');
    ylabel('Nm');
    title(sprintf('RPM = %d', rpm_list(i)));
    legend('Location','southwest');
end
pretty();  % if your pretty.m is in path; otherwise comment out

% plots: overlay
figure('Name','Regen overlay (all RPM)'); clf; hold on; grid on;
cols = lines(nR);
for i = 1:nR
    plot(br*100, Tcmd(i,:), '-', 'Color', cols(i,:), 'LineWidth', 1.8, ...
         'DisplayName', sprintf('%d rpm', rpm_list(i)));
end
xlabel('brake [%]'); ylabel('T_{cmd} [Nm]');
title(sprintf('Regen T_{cmd} vs brake (mode = %s)', mode));
legend('Location','southwest');
pretty();  % optional

end

% helpers
function val = getOpt(args, name, defaultVal)
    ix = find(cellfun(@ischar,args),1);
    val = defaultVal;
    if ~isempty(args)
        for k = 1:2:numel(args)
            if ischar(args{k}) && strcmpi(args{k}, name)
                val = args{k+1}; return;
            end
        end
    end
end

function v = existOr(x,alt)
    if exist('x','var') && ~isempty(x), v=x; else, v=alt; end
end

function v = getfield_safe(S, f1, f2)
    v = [];
    if isfield(S,f1) && isfield(S.(f1),f2)
        v = S.(f1).(f2);
    end
end

%To Run
% P = params_default();
% regen_sweep(P, 'accel', [1000 2000 4000]);
% P = params_default();
% regen_sweep(P, 'accel', [1000 2000 4000]);
% P = params_default();
% regen_sweep(P, 'accel', [1500 3000], 'soc', 0.95);
% P = params_default();
% P.Regen.cutoff_rpm = 600;           % raise cutoff to see the knee clearly
% regen_sweep(P, 'autoX', [300 800 1500]);  % include a below-cutoff RPM
% P = params_default();
% P.Temp.Inv.warn = 70;  P.Temp.Inv.limit = 85;
% regen_sweep(P, 'enduro', [2000 4000], 'temps', [30 80 30]);
% 

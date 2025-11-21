function plot_caps_stack(results, varargin)
%% Plot Caps Stack: combined drive-side torque cap visualization for all modes
%% To run:
%   plot_caps_stack(results)
%   plot_caps_stack(results, 'accel','autoX','enduro')
%   plot_caps_stack(results.accel, 'accel')   % also works for single mode

% Detect input type
is_single_mode = isstruct(results) && isfield(results, 'T_cmd') && ~isfield(results, 'accel');
if is_single_mode
    dataStruct.single = results;
    order = {'single'};
else
    dataStruct = results;
    if ~isempty(varargin)
        order = varargin;
    else
        order = intersect({'accel','autoX','enduro'}, fieldnames(results), 'stable');
        if isempty(order)
            order = fieldnames(results);
        end
    end
end

% Guard
if isempty(order)
    warning('plot_caps_stack: No modes to plot.');
    return;
end

% Layout
figure('Name','Torque caps (drive side) — all modes');
tiledlayout(numel(order), 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:numel(order)
    m = order{i};
    if ~isfield(dataStruct, m) || isempty(dataStruct.(m))
        nexttile; axis off;
        text(0.5,0.5,sprintf('(No data: %s)', m),'HorizontalAlignment','center');
        continue;
    end

    out = dataStruct.(m);

    % Defensive checks: some results structs may store "Tcmd" or miss "T_req"
    if ~isfield(out, 'T_cmd')
        warning('Skipping %s: missing T_cmd', m);
        continue;
    end
    if ~isfield(out, 'T_req')
        % Some runs may store Treq in caps struct instead
        try
            out(1).T_req = 0;  % placeholder
            for k=1:numel(out)
                if isfield(out(k).caps, 'T_req')
                    out(k).T_req = out(k).caps.T_req;
                else
                    out(k).T_req = NaN;
                end
            end
        catch
            warning('Could not reconstruct T_req for %s', m);
            continue;
        end
    end

    % Extract series
    n = numel(out);
    t = 1:n;

    get = @(p,def) arrayfun(@(x) tryget(x.caps,p,def), out);

    Treq   = [out.T_req];
    Tcmd   = [out.T_cmd];

    Pcap   = nan2zero(get('env.T_power_drive',NaN));
    PackI  = nan2zero(get('env.T_pack_drive',NaN));
    PhaseI = nan2zero(get('env.T_phase_drive',NaN));
    RPMcap = nan2zero(get('env.T_rpm',NaN));

    Dtherm = get('derate.Dtherm',1);
    EnvDrv = nan2zero(get('env.C_drive',NaN));
    Final  = EnvDrv .* Dtherm;

    % Plotting
    nexttile; hold on; grid on;
    Z = zeros(1,n);

    patch_band(t, Z, Pcap,   [0.80 0.90 1.00], 'Power cap');
    patch_band(t, Z, PackI,  [0.90 0.80 1.00], 'Pack I + V sag');
    patch_band(t, Z, PhaseI, [0.90 1.00 0.80], 'Phase I cap');
    patch_band(t, Z, RPMcap, [1.00 0.90 0.80], 'RPM cap');

    plot(t, Final,'-','Color',[0.3 0.3 0.3],'LineWidth',2.0,'DisplayName','Envelope × D_{therm}');
    plot(t, Treq ,'k:','LineWidth',1.2,'DisplayName','T_{req}');
    plot(t, Tcmd ,'k-','LineWidth',1.4,'DisplayName','T_{cmd}');

    ylabel('Nm');
    title(sprintf('Drive caps — %s', m));
    if i == 1, legend('Location','eastoutside'); end
    if i == numel(order), xlabel('time step'); end
end
end

% helper functions 
function patch_band(x, y0, y1, col, name)
xx = [x, fliplr(x)];
yy = [y0, fliplr(y1)];
p  = patch(xx, yy, col, 'EdgeColor','none', 'FaceAlpha',0.25);
set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
p.DisplayName = name;
end

function v = tryget(s, path, def)
try
    seg = strsplit(path,'.');
    v = s.(seg{1});
    for i = 2:numel(seg)
        v = v.(seg{i});
    end
catch
    v = def;
end
end

function y = nan2zero(y)
y(~isfinite(y)) = 0;
end



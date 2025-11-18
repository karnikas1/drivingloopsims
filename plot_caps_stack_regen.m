function plot_caps_stack_regen(results, varargin)
% Combined REGEN-side torque caps for all modes (stacked subplots).
% Works with:
%   plot_caps_stack_regen(results)                      % accel/autoX/enduro
%   plot_caps_stack_regen(results,'accel','autoX')      % choose order
%   plot_caps_stack_regen(results.autoX,'autoX')        % single mode

% -- detect input type (results struct vs single out-array)
is_single_mode = isstruct(results) && isfield(results,'T_cmd') && ~isfield(results,'accel');
if is_single_mode
    dataStruct.single = results;
    order = {'single'};
else
    dataStruct = results;
    if ~isempty(varargin)
        order = varargin;
    else
        order = intersect({'accel','autoX','enduro'}, fieldnames(results), 'stable');
        if isempty(order), order = fieldnames(results); end
    end
end
if isempty(order), warning('plot_caps_stack_regen: no modes to plot.'); return; end

figure('Name','Torque caps (regen side) — all modes');
tiledlayout(numel(order),1,'TileSpacing','compact','Padding','compact');

for i = 1:numel(order)
    m = order{i};
    if ~isfield(dataStruct,m) || isempty(dataStruct.(m))
        nexttile; axis off; text(0.5,0.5,sprintf('(No data: %s)',m),'HorizontalAlignment','center'); continue;
    end
    out = dataStruct.(m);
    if ~isfield(out,'T_cmd'), nexttile; axis off; text(0.5,0.5,sprintf('(Missing T_cmd: %s)',m),'HorizontalAlignment','center'); continue; end

    % reconstruct T_req if needed
    if ~isfield(out,'T_req')
        try
            for k=1:numel(out)
                if isfield(out(k),'caps') && isfield(out(k).caps,'T_req')
                    out(k).T_req = out(k).caps.T_req;
                else
                    out(k).T_req = NaN;
                end
            end
        catch
            nexttile; axis off; text(0.5,0.5,sprintf('(No T_{req}: %s)',m),'HorizontalAlignment','center'); continue;
        end
    end

    n = numel(out);
    t = 1:n;

    get = @(p,def) arrayfun(@(x) tryget(x.caps,p,def), out);

    Treq = [out.T_req];
    Tcmd = [out.T_cmd];

    % regen envelopes (negative values by convention)
    PcapR   = nan2zero(get('env.T_power_regen',NaN));     % power-limited regen torque (≤0)
    ChgIcap = nan2zero(get('env.T_chg_current',NaN));     % charge-current + V rise (≤0)
    MechR   = nan2zero(get('env.T_regen_mech',NaN));      % mechanical regen cap (≤0)
    EnvReg  = nan2zero(get('env.C_regen',NaN));           % min/max regen envelope (≤0)

    Dtherm  = get('derate.Dtherm',1);
    Final   = EnvReg .* Dtherm;                           % envelope × thermal (≤0)

    Z = zeros(1,n);

    ax = nexttile; hold(ax,'on'); grid(ax,'on');

    % draw negative bands (from 0 down to the cap)
    patch_band(ax, t, Z, PcapR,   [0.80 0.90 1.00], 'P cap (regen)');
    patch_band(ax, t, Z, ChgIcap, [0.90 0.80 1.00], 'Charge I + V rise');
    patch_band(ax, t, Z, MechR,   [0.90 1.00 0.80], 'Mech regen cap');

    % overlays
    plot(ax, t, Final,'-','Color',[0.30 0.30 0.30],'LineWidth',2.0, 'DisplayName','Envelope × D_{therm}');
    plot(ax, t, Treq ,'k:','LineWidth',1.2,'DisplayName','T_{req}');
    plot(ax, t, Tcmd ,'k-','LineWidth',1.4,'DisplayName','T_{cmd}');

    ylabel(ax,'Nm'); title(ax, sprintf('Regen caps — %s', m));
    if i==1, legend(ax,'Location','eastoutside'); end
    if i==numel(order), xlabel(ax,'time step'); end
end
end

% ----- helpers -----
function patch_band(ax, x, y0, y1, col, name)
% works for positive or negative y1; fills between y0 and y1
xx = [x, fliplr(x)];
yy = [y0, fliplr(y1)];
p  = patch(ax, xx, yy, col, 'EdgeColor','none', 'FaceAlpha',0.25);
set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','on');
p.DisplayName = name;
end

function v = tryget(s, path, def)
try
    seg = strsplit(path,'.');
    v = s.(seg{1});
    for i=2:numel(seg), v = v.(seg{i}); end
catch
    v = def;
end
end

function y = nan2zero(y)
y(~isfinite(y)) = 0;
end

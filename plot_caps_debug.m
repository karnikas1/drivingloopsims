function plot_caps_debug(out, tag)
%% Plot which cap limits torque + show derates & battery behavior.
%% To run:
%   plot_caps_debug(results.autoX,'autoX')

if nargin<2, tag = ''; end
t = 1:numel(out);

% pull series safely
get = @(f,def) arrayfun(@(x) tryget(x.caps,f,def), out);
sgnT = sign([out.T_req]);

% envelopes (drive)
T_power_drive = get('env.T_power_drive',NaN);
T_rpm         = get('env.T_rpm',NaN);
T_phase_drive = get('env.T_phase_drive',NaN);
T_pack_drive  = get('env.T_pack_drive',NaN);
C_drive       = get('env.C_drive',NaN);

% envelopes (regen)
T_power_regen = get('env.T_power_regen',NaN);
T_chg_current = get('env.T_chg_current',NaN);
T_regen_mech  = get('env.T_regen_mech',NaN);
C_regen       = get('env.C_regen',NaN);

% derates
Dtherm     = get('derate.Dtherm',1);
Dsoc_dis   = get('derate.Dsoc_dis',1);
Dsoc_chg   = get('derate.Dsoc_chg',1);
Dsoc_blend = get('derate.Dsoc_blend',1);

% battery
Voc       = get('batt.Voc',NaN);
V_at_Idis = get('batt.V_at_Idis',NaN);
V_at_Ichg = get('batt.V_at_Ichg',NaN);
Idis_cap  = get('batt.Idis_cap',NaN);
Ichg_cap  = get('batt.Ichg_cap',NaN);

Treq = [out.T_req];  Tcmd = [out.T_cmd];

%% 1) Drive envelope & dominant limiter (counts)
figure('Name',['Caps debug (drive) ',tag]); tiledlayout(2,1,'TileSpacing','compact');

nexttile; hold on; grid on;
plot(t,Treq,'k:','DisplayName','T_{req}');
plot(t,Tcmd,'k-','DisplayName','T_{cmd}');
plot(t,T_power_drive,'-','DisplayName','P cap (drive)');
plot(t,T_rpm,'-','DisplayName','RPM cap');
plot(t,T_phase_drive,'-','DisplayName','Phase I cap');
plot(t,T_pack_drive,'-','DisplayName','Pack I + V sag');
plot(t,C_drive,'LineWidth',1.6,'DisplayName','Drive envelope (min)');
ylabel('Nm'); title(['Drive-side limits ',tag]); legend('Location','eastoutside');

% dominant limiter label (drive timesteps only) → numeric counts (no warnings)
drive_mask = (sgnT>=0);
dom = dominator_drive(T_power_drive,T_rpm,T_phase_drive,T_pack_drive);
dom(~drive_mask) = "—";

nexttile; cla; grid on; hold on;
cats = categorical({'Power','RPM','PhaseI','PackI+Vsag'});
counts = [sum(dom=="Power"), sum(dom=="RPM"), ...
          sum(dom=="PhaseI"), sum(dom=="PackI+Vsag")];
bar(cats, counts, 'FaceAlpha',0.85);
title('Dominant drive limiter (count)'); ylabel('samples');

%% 2) Regen envelope & dominant limiter (counts)
figure('Name',['Caps debug (regen) ',tag]); tiledlayout(2,1,'TileSpacing','compact');

nexttile; hold on; grid on;
plot(t,Treq,'k:','DisplayName','T_{req}');
plot(t,Tcmd,'k-','DisplayName','T_{cmd}');
plot(t,T_power_regen,'-','DisplayName','P cap (regen)');
plot(t,T_chg_current,'-','DisplayName','Charge I + V rise');
plot(t,T_regen_mech,'-','DisplayName','Mech regen cap');
plot(t,C_regen,'LineWidth',1.6,'DisplayName','Regen envelope (max)');
ylabel('Nm'); title(['Regen-side limits ',tag]); legend('Location','eastoutside');

regen_mask = (sgnT<0);
domR = dominator_regen(T_power_regen,T_chg_current,T_regen_mech);
domR(~regen_mask) = "—";

nexttile; cla; grid on; hold on;
catsR   = categorical({'Power','ChargeI+Vrise','Mech'});
countsR = [sum(domR=="Power"), sum(domR=="ChargeI+Vrise"), sum(domR=="Mech")];
bar(catsR, countsR, 'FaceAlpha',0.85);
title('Dominant regen limiter (count)'); ylabel('samples');

%% 3) Derates + battery
figure('Name',['Derates & battery ',tag]); tiledlayout(3,1,'TileSpacing','compact');

nexttile; hold on; grid on;
plot(t,Dtherm,'-','DisplayName','D_{therm}');
plot(t,Dsoc_dis,'-','DisplayName','D_{soc,dis}');
plot(t,Dsoc_chg,'-','DisplayName','D_{soc,chg}');
plot(t,Dsoc_blend,'-','DisplayName','D_{soc,blend}');
ylim([0 1.05]); ylabel('multiplier'); title('Derate multipliers'); legend('Location','eastoutside');

nexttile; hold on; grid on;
plot(t,Voc,'-','DisplayName','V_{oc}');
plot(t,V_at_Idis,'-','DisplayName','V@I_{dis}');
plot(t,V_at_Ichg,'-','DisplayName','V@I_{chg}');
ylabel('Volts'); title('Battery voltage (sag/rise)'); legend('Location','eastoutside');

nexttile; hold on; grid on;
plot(t,Idis_cap,'-','DisplayName','I_{dis,cap}');
plot(t,Ichg_cap,'-','DisplayName','I_{chg,cap}');
ylabel('Amps'); xlabel('time step'); title('Current caps'); legend('Location','eastoutside');

end

%helpers
function v = tryget(s, path, def)
% path like 'env.T_power_drive'
try
    seg = strsplit(path,'.');
    v = s.(seg{1});
    for i=2:numel(seg), v = v.(seg{i}); end
catch
    v = def;
end
end

function lab = dominator_drive(Pcap, RPMcap, PhIcap, PackIcap)
% choose the smallest cap (most restrictive) each timestep
M = [Pcap(:), RPMcap(:), PhIcap(:), PackIcap(:)];
[~,idx] = min(M,[],2,'omitnan');
names = ["Power","RPM","PhaseI","PackI+Vsag"];
lab = names(idx)';
end

function lab = dominator_regen(Pcap, ChgIcap, Mechcap)
% choose the most negative (max magnitude) cap (remember they are <=0)
M = [Pcap(:), ChgIcap(:), Mechcap(:)];
[~,idx] = min(M,[],2,'omitnan');   % most negative = min
names = ["Power","ChargeI+Vrise","Mech"];
lab = names(idx)';
end


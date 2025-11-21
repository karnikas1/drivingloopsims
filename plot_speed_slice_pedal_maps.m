function plot_speed_slice_pedal_maps()
%% Plot Speed Slice Pedal Maps: plot stuff
P = params_default();

% Speeds to slice (m/s)
v_list = [10 20 30 40] / 3.6; % 10..40 km/h for visuals
pedal = linspace(0,1,101);
lift  = linspace(0,1,101); % below deadband, for regen

% drive map (torque limited by motor curve & power)
driveT = zeros(numel(v_list), numel(pedal));
for i = 1:numel(v_list)
    v = v_list(i);
    omega = max(1e-3, v / P.Vehicle.r_wheel);
    T_cap = min(P.Torque.max, (P.Power.max*1000)/omega);
    % choose a mapping (this matches your main loop “driver intent”)
    gain = 1.0;  % visualize raw map before other caps
    driveT(i,:) = gain * pedal * T_cap;  % simple linear map to cap
end

% adaptive OPD: regen torque vs lift, fades at low speed
regenT = zeros(numel(v_list), numel(lift));
for i = 1:numel(v_list)
    v = v_list(i);
    omega = max(1e-3, v / P.Vehicle.r_wheel);
    % available regen from electrical limits
    T_reg_lim = min(P.Regen.max_nm, (P.Regen.i_chg_max * 400) / omega);  % assume ~400V bus here
    fade = min(1, v/5);                       % adaptive fade (0→1 over ~5 m/s)
    regenT(i,:) = - T_reg_lim .* (lift) .* fade;
end

%% plot
figure('Name','Speed-slice pedal maps'); tiledlayout(2,1,'TileSpacing','compact');

%% top: drive map
nexttile; hold on; lw=1.6;
for i=1:numel(v_list)
    plot(100*pedal, driveT(i,:), 'LineWidth', lw, 'DisplayName', sprintf('v = %d km/h', round(v_list(i)*3.6)));
end
xlabel('Accelerator pedal p [%]'); ylabel('Drive torque T [Nm]');
title('Drive map: T_d(p,v) with torque/power caps'); grid on; legend('Location','southeast');

%% bottom: adaptive OPD lift then regen torque
nexttile; hold on;
for i=1:numel(v_list)
    plot(100*lift, regenT(i,:), 'LineWidth', lw, 'DisplayName', sprintf('v = %d km/h', round(v_list(i)*3.6)));
end
xlabel('Lift (below deadband) u [%]'); ylabel('Regen torque T_{regen} [Nm]');
title('Adaptive OPD: T_{regen}(u,v) (more regen as speed rises)'); grid on; legend('Location','northeast');
end

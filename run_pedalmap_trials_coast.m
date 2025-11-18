%% Run pedal-map trials with 0-torque coast zone
clear; close all; clc;

P = params_default();
P.opd.coast.enable = true;     % make sure coast is active
speeds = [10 40 80];           % km/h

%% Trial definitions (same as before)
trials = {
    'linear',     1.0, NaN, NaN;   % 1
    'linear',     1.5, NaN, NaN;   % 2
    'adaptive',   1.0, NaN, NaN;   % 3
    'adaptive',   1.5, NaN, NaN;   % 4
    'power',      1.0, 1.7, NaN;   % 5
    'power',      1.0, 1.3, NaN;   % 6
    'power',      1.0, 2.2, NaN;   % 7
    'smoothstep', 1.0, NaN, NaN;   % 8
    'smoothstep', 1.5, NaN, NaN;   % 9
    'bezier',     1.0, NaN, [0.25 0.85; 0.70 0.95]; % 10
    'bezier',     1.5, NaN, [0.25 0.85; 0.70 0.95]; % 11
};

%% Run all trials
for i = 1:size(trials,1)
    type = trials{i,1};
    kp   = trials{i,2};
    s    = trials{i,3};
    bz   = trials{i,4};

    P.Mode.accel.type        = type;
    P.Mode.accel.pedal_gain  = kp;
    if ~isnan(s),  P.Mode.accel.curve_s = s; end
    if ~isnan(bz), P.Mode.accel.bezier  = bz; end

    fprintf('Running Trial %02d – %s (Kp=%.1f)\n', i, type, kp);

    % keep workspace clean & auto-save figure
    close all;
    pedal_map_sweep(P,'accel',speeds,true,true);
    saveas(gcf, sprintf('trial_%02d_coast.png', i));
end

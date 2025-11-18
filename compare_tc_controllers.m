function compare_tc_controllers()
% TC comparison: P vs fuzzy under an injected slip patch

pretty();  

P  = params_default();
P.TC.slip_target = 0.08;   % target lower than injected slip
dt = 0.01;  T = 10;  n = round(T/dt);
mode = 'autoX';

% driver inputs
pedal = [linspace(0,0.6,200), 0.6*ones(1,n-200)];
brake = zeros(1,n);

% slip patch: wheel spins 20% faster for 1.5 s
slip_on = 3.5; slip_off = 5.0;
idx_on = round(slip_on/dt); idx_off = round(slip_off/dt);
slip_gain = 1.20;

outP = run_with_tc(@tc_trim_p);
outF = run_with_tc(@tc_trim_fuzzy);

% plots
t = (0:n-1)*dt;
figure('Name','TC comparison'); tiledlayout(3,1,'TileSpacing','compact');

% torque trims
nexttile; hold on;
plot(t,[outP.T_cmd],'LineWidth',1.8,'DisplayName','P controller');
plot(t,[outF.T_cmd],'LineWidth',1.8,'DisplayName','Fuzzy controller');
ylabel('T_{cmd} [Nm]'); title('Torque trims under slip'); legend; grid on;
xline([slip_on slip_off],'r--',{'slip on','slip off'});

% slip tracking
nexttile; hold on;
plot(t,slip_series(outP,P),'LineWidth',1.6,'DisplayName','P slip');
plot(t,slip_series(outF,P),'LineWidth',1.6,'DisplayName','Fuzzy slip');
ylabel('slip ratio'); title('Slip tracking'); legend; grid on;
xline([slip_on slip_off],'r--');

% speed
nexttile; hold on;
plot(t,[outP.v]*3.6,'LineWidth',1.6,'DisplayName','P speed');
plot(t,[outF.v]*3.6,'LineWidth',1.6,'DisplayName','Fuzzy speed');
ylabel('km/h'); xlabel('time [s]'); grid on; legend;
xline([slip_on slip_off],'r--');

    % ---------- inner helpers ----------
    function out = run_with_tc(tc_fun)
        state.last_T = 0;
        v0 = 10/3.6; state.omega = v0 / P.Vehicle.r_wheel;

        tmpl = struct('T_req',0,'T_cmd',0,'F_front',0,'F_rear',0, ...
                      'omega',0,'v',0,'caps',struct('rpm',0,'power',0,'derate',1), ...
                      'wheelOmega',0);
        out(1:n) = tmpl;

        for k = 1:n
            v_k = state.omega * P.Vehicle.r_wheel;

            % base wheel = motor omega, then inject slip in window
            omega_w = state.omega;
            if k>=idx_on && k<=idx_off, omega_w = omega_w * slip_gain; end

            u = struct('pedal',pedal(k),'brake',brake(k), ...
                       'wheelSpeeds',omega_w,'soc',0.6,'vpack',400,'temps',[30 30 30], ...
                       'dt',dt,'v',v_k,'omega',state.omega);

            % pedal map → torque (Nm), with drive cap
            mp = P.Mode.(mode); mp.v = u.v;
            T_drive_cap = min(P.Torque.max, (P.Power.max*1000)/max(u.omega,1));
            T_req = pedal_map(u.pedal, mp, T_drive_cap);

            % safety inhibit (no APPS+BPS in this scenario anyway)
            if (u.brake>0.05) && (u.pedal>0.25), T_req=0; end

            % lift-regen if off pedal
            if u.pedal < 0.05
                T_req = -min(P.Regen.max_nm, regen_limit(u.omega, u.soc, u.vpack, P));
            end

            % TRACTION CONTROL (swap implementation here)
            T_req = tc_fun(T_req, u, P);

            % caps + smoothing + plant
            [T_cmd, caps] = apply_caps(T_req,u,P);
            T_cmd = torque_filter(T_cmd, state.last_T, P.Filter);
            [omega_new, v_new] = longitudinalPlant(T_cmd, state.omega, dt, P);

            % log
            o = tmpl; o.T_req=T_req; o.T_cmd=T_cmd; o.caps=caps;
            o.omega=omega_new; o.v=v_new; o.wheelOmega=omega_w;
            out(k)=o;

            state.last_T=T_cmd; state.omega=omega_new;
        end
    end

    function s = slip_series(out,P)
        v = [out.v];
        omega_w = [out.wheelOmega];                 % wheel (incl slip), not motor
        s = (omega_w*P.Vehicle.r_wheel - v) ./ max(v,0.1);
    end
end



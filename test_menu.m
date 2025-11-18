P = params_default();
state.last_T = 0; state.omega = 1000;
dt = 0.01;

% lift-off regen test
u = struct('pedal',0,'brake',0,'wheelSpeeds',1000,...
           'soc',0.5,'vpack',400,'temps',[25 25 25],...
           'dt',dt,'v',20,'omega',1000);

for k=1:500
    [out(k), state] = run_driving_loop(u,P,state,'enduro');
end

plotResults(out,'Lift-off regen');

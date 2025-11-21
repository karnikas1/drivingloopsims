function plotResults(out,titleStr)
%% Plot Results: plot the requested v commanded torque
figure; hold on;
plot([out.T_req],'--','DisplayName','Torque req');
plot([out.T_cmd],'-','DisplayName','Torque cmd');
legend show; title(titleStr);
xlabel('time step'); ylabel('Torque (Nm)');
end

function plotResults(t, R1, R2, name1, name2)
figure('Color','w','Name','Compare controllers');

subplot(3,1,1)
plot(t,R1.v,'LineWidth',1.6); hold on; plot(t,R2.v,'LineWidth',1.6);
ylabel('Speed [m/s]'); grid on; legend(name1,name2,'Location','best'); title('Vehicle speed');

subplot(3,1,2)
plot(t,R1.T,'LineWidth',1.4); hold on; plot(t,R2.T,'LineWidth',1.4);
yline(0,'k-'); ylabel('Axle torque [Nm]'); grid on; legend(name1,name2); 
title('Torque (+drive, −regen)');

subplot(3,1,3)
plot(t,R1.Erec/1000,'LineWidth',1.6); hold on; plot(t,R2.Erec/1000,'LineWidth',1.6);
ylabel('Recovered energy [kJ]'); xlabel('Time [s]'); grid on; legend(name1,name2);
title(sprintf('Total: %s = %.1f kJ,  %s = %.1f kJ',name1,R1.Erec(end)/1000,name2,R2.Erec(end)/1000));
end
